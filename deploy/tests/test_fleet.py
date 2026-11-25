from pathlib import Path
import json
import os
import pwd
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import Mock, call, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'fleet'))
import moboterra_fleet as updater

OLD = 'a' * 40
NEW = 'b' * 40
OLD_ID = 'sha256:' + 'c' * 64
NEW_ID = 'sha256:' + 'd' * 64
REPO = 'ghcr.io/moboticde/moboterra'
IMAGE = REPO + '@sha256:' + 'e' * 64


def done(stdout='', code=0):
    return subprocess.CompletedProcess([], code, stdout)


class FleetTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.state = Path(temporary.name)
        (self.state / '.git').mkdir()
        account = Mock(pw_uid=os.getuid())
        with patch.object(pwd, 'getpwnam', return_value=account):
            self.fleet = updater.Fleet(
                dict(checkout=str(self.state), owner='operator',
                     robot='MoboTerra_2', image_repository=REPO), state=self.state)
        self.fleet.git = Mock(return_value=done())

    def image_metadata(self, architecture):
        return {'Id': NEW_ID, 'Os': 'linux', 'Architecture': architecture, 'Config': {'Labels': {
            'org.opencontainers.image.revision': NEW,
            'com.mobotic.stack': 'four-micontrol-bundled-romb-v1',
        }}}

    def test_stage_downloads_the_host_platform_without_cutover(self):
        release = {'commit': NEW, 'image': IMAGE}
        for host, architecture in (('x86_64', 'amd64'), ('aarch64', 'arm64')):
            with self.subTest(host=host):
                self.fleet.git.side_effect = [done(), done(), done(json.dumps(release)), done()]
                results = [done('linux/' + host), done(code=1), done(),
                           done(json.dumps(self.image_metadata(architecture)))]
                with patch.object(updater, 'run', side_effect=results) as run:
                    self.assertEqual(self.fleet.stage(), dict(release, id=NEW_ID))
                self.assertEqual(updater.read_json(self.fleet.staged), dict(release, id=NEW_ID))
                self.assertIn(call('docker', 'pull', '--platform', f'linux/{architecture}', IMAGE),
                              run.call_args_list)
                self.fleet.git.assert_any_call('ls-remote', '--exit-code', '--refs', 'origin',
                                               'refs/heads/fleet/MoboTerra_2', capture=True, check=False)
                self.fleet.git.assert_any_call('merge-base', '--is-ancestor', NEW, updater.MAIN_REF)
                self.assertTrue(all(command.args[0] == 'docker' for command in run.call_args_list))

    def test_stage_rejects_an_image_for_the_wrong_platform(self):
        release = {'commit': NEW, 'image': IMAGE}
        self.fleet.git.side_effect = [done(), done(), done(json.dumps(release)), done()]
        results = [done('linux/x86_64'), done(), done(json.dumps(self.image_metadata('arm64')))]
        with patch.object(updater, 'run', side_effect=results):
            with self.assertRaisesRegex(ValueError, 'architecture'):
                self.fleet.stage()
        self.assertFalse(self.fleet.staged.exists())

    def test_removed_approval_clears_staged_record(self):
        self.fleet.staged.touch()
        self.fleet.git.return_value = done(code=2)
        with patch.object(updater, 'run') as run:
            self.assertIsNone(self.fleet.stage())
        self.assertFalse(self.fleet.staged.exists())
        run.assert_not_called()

    def test_release_can_precede_docs_and_fleet_tooling(self):
        self.fleet.git.side_effect = [done(code=1), done(), done()]
        self.fleet.check_release_order(NEW, OLD)
        self.fleet.git.assert_any_call('merge-base', '--is-ancestor', OLD, NEW)
        self.fleet.git.assert_called_with('diff', '--quiet', OLD, NEW, '--', '.',
                                          ':(exclude,glob)docs/**', ':(exclude,glob)**/*.md',
                                          ':(exclude,glob)deploy/fleet/**',
                                          ':(exclude)deploy/tests/test_fleet.py', check=False)

    def test_release_cannot_downgrade_installed_software(self):
        self.fleet.git.side_effect = [done(code=1), done(), done(code=1)]
        with self.assertRaisesRegex(RuntimeError, 'older than the installed software'):
            self.fleet.check_release_order(NEW, OLD)

    def prepare_apply(self):
        for method in ('clean_checkout', 'stop', 'select', 'wait_for_health', 'restore_boot_setting'):
            setattr(self.fleet, method, Mock())
        self.fleet.stage = Mock(return_value={'commit': NEW, 'image': IMAGE, 'id': NEW_ID})
        self.fleet.revision = Mock(return_value=OLD)
        self.fleet.installed_image = Mock(return_value=OLD_ID)

    def test_new_approval_does_not_cut_over_under_an_old_confirmation(self):
        self.prepare_apply()
        with self.assertRaisesRegex(RuntimeError, 'does not match'):
            self.fleet.apply(OLD)
        self.fleet.stop.assert_not_called()
        self.fleet.select.assert_not_called()

    def test_apply_records_previous_pair_only_after_new_image_is_healthy(self):
        self.prepare_apply()

        def wait(image_id):
            self.assertEqual(image_id, NEW_ID)
            self.assertTrue(self.fleet.pending.exists())
            self.assertFalse(self.fleet.previous.exists())

        self.fleet.wait_for_health.side_effect = wait
        with patch.object(updater, 'run', side_effect=[done(), done('disabled'), done()]):
            self.fleet.apply(NEW)
        self.fleet.select.assert_called_once_with(NEW, IMAGE)
        self.fleet.restore_boot_setting.assert_called_once_with(False)
        self.assertFalse(self.fleet.pending.exists())
        self.assertEqual(updater.read_json(self.fleet.previous),
                         {'commit': OLD, 'image': OLD_ID, 'enabled': False})

    def test_failed_health_restores_old_pair_and_leaves_stack_stopped(self):
        self.prepare_apply()
        self.fleet.wait_for_health.side_effect = RuntimeError('unhealthy')
        with patch.object(updater, 'run', side_effect=[done(), done('enabled'), done(), done()]) as run:
            with self.assertRaisesRegex(RuntimeError, 'Previous release restored and stopped'):
                self.fleet.apply(NEW)
        self.assertEqual(self.fleet.select.call_args_list,
                         [call(NEW, IMAGE), call(OLD, OLD_ID)])
        self.fleet.restore_boot_setting.assert_called_once_with(True)
        self.assertEqual(self.fleet.stop.call_count, 2)
        self.assertEqual(sum(c.args[:2] == ('/usr/local/bin/moboterra-ctl', 'start')
                             for c in run.call_args_list), 1)
        self.assertFalse(self.fleet.pending.exists())


if __name__ == '__main__':
    unittest.main()
