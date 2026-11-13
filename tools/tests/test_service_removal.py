"""Regression for systemctl disable unlinking an externally linked unit."""
from pathlib import Path
import json
import os
import subprocess

SOURCE = Path(__file__).resolve().parents[2] / 'scripts/remove_managed_service.sh'


def test_disable_removing_both_links_and_repeated_removal_succeed(tmp_path):
    project = tmp_path / 'project'
    scripts = project / 'scripts'
    scripts.mkdir(parents=True)
    target = project / 'config/systemd/moboterra-supervisor.service'
    target.parent.mkdir(parents=True)
    target.write_text('[Service]\nExecStart=/bin/true\n')
    unit = tmp_path / 'systemd/moboterra-supervisor.service'
    unit.parent.mkdir()
    unit.symlink_to(target)
    wants = tmp_path / 'systemd/multi-user.target.wants/moboterra-supervisor.service'
    wants.parent.mkdir()
    wants.symlink_to(unit)
    script = scripts / SOURCE.name
    # Sandbox only the administrative path and privilege gate. The removal
    # sequence runs unchanged, against a fake systemctl, never the host bus.
    content = SOURCE.read_text().replace('unit_path=/etc/systemd/system/moboterra-supervisor.service',
                                        'unit_path="' + str(unit) + '"')
    content = content.replace('"$EUID" -ne 0', '0 -ne 0')
    script.write_text(content)
    binary_dir = tmp_path / 'bin'
    binary_dir.mkdir()
    fake = binary_dir / 'systemctl'
    fake.write_text('''#!/usr/bin/python3
import json,os,sys
from pathlib import Path
args=sys.argv[1:]
with open(os.environ['TEST_CALLS'],'a') as f:f.write(json.dumps(args)+'\\n')
unit=Path(os.environ['TEST_UNIT'])
if args[0]=='show':
 print('loaded' if unit.exists() else 'not-found')
elif args[0]=='stop':
 if not unit.exists():raise SystemExit('Unit not loaded')
elif args[0]=='disable':
 unit.unlink(missing_ok=True)
 Path(os.environ['TEST_WANTS']).unlink(missing_ok=True)
 if '--now' in args:raise SystemExit('Unit not loaded after disable')
elif args[0] not in ('daemon-reload','reset-failed'):
 raise SystemExit('Unexpected fake systemctl command: '+repr(args))
''')
    fake.chmod(0o755)
    calls = tmp_path / 'calls.jsonl'
    env = dict(os.environ, PATH=str(binary_dir) + ':' + os.environ['PATH'],
               TEST_CALLS=str(calls), TEST_UNIT=str(unit), TEST_WANTS=str(wants))
    for _ in range(2):
        result = subprocess.run(['bash', str(script)], env=env, text=True, capture_output=True)
        assert result.returncode == 0, result.stderr
        assert 'MoboTerra removed from systemd' in result.stdout
    recorded = [json.loads(line) for line in calls.read_text().splitlines()]
    actions = [row[0] for row in recorded]
    assert actions.count('stop') == actions.count('disable') == 1
    assert actions.index('stop') < actions.index('disable')
    assert not unit.is_symlink() and not wants.is_symlink()
    assert target.exists()
    state = json.loads((project / 'runtime/system-settings-installation.json').read_text())
    assert state['managed_service_removed'] is True
