"""Filesystem-only regressions: preserve settings and recover failed linking."""
import importlib.util
from pathlib import Path
import pytest

SOURCE = Path(__file__).resolve().parents[2] / 'scripts/install_project_settings.py'
spec = importlib.util.spec_from_file_location('settings_migration', SOURCE)
migration = importlib.util.module_from_spec(spec)
spec.loader.exec_module(migration)


def test_original_is_preserved_and_linking_is_idempotent(tmp_path):
    original = tmp_path / 'old'
    original.mkdir()
    (original / 'platform.yaml').write_text('node_id: 10\n')
    target = tmp_path / 'project/settings'
    target.mkdir(parents=True)
    (target / 'platform.yaml').write_text('node_id: 10\n')
    backup = tmp_path / 'project/backups/original'
    assert migration.relocate_link(original, target, backup)
    assert original.is_symlink()
    assert (original / 'platform.yaml').read_text() == 'node_id: 10\n'
    assert (backup / 'platform.yaml').read_text() == 'node_id: 10\n'
    assert not migration.relocate_link(original, target, backup)
    migration.restore_link(original, backup)
    assert not original.is_symlink()
    assert (original / 'platform.yaml').read_text() == 'node_id: 10\n'


def test_failed_link_creation_restores_original(tmp_path, monkeypatch):
    original = tmp_path / 'original.yaml'
    original.write_text('bitrate: 500000\n')
    target = tmp_path / 'target.yaml'
    target.write_text('bitrate: 500000\n')
    backup = tmp_path / 'backup.yaml'
    def refuse_link(*args, **kwargs):
        raise OSError('simulated permission failure')
    monkeypatch.setattr(Path, 'symlink_to', refuse_link)
    with pytest.raises(OSError, match='permission failure'):
        migration.relocate_link(original, target, backup)
    assert original.read_text() == 'bitrate: 500000\n'
    assert not backup.exists()


def test_changed_original_is_rejected_before_relocation(tmp_path, monkeypatch):
    import hashlib
    import json
    original = tmp_path / 'external.yaml'
    original.write_text('node_id: 10\n')
    target = tmp_path / 'project/settings.yaml'
    target.parent.mkdir()
    target.write_bytes(original.read_bytes())
    manifest_dir = target.parent / 'config/managed'
    manifest_dir.mkdir(parents=True)
    digest = hashlib.sha256(original.read_bytes()).hexdigest()
    (manifest_dir / 'migration-sources.json').write_text(json.dumps({str(original): {'.': digest}}))
    monkeypatch.setattr(migration, 'PROJECT_ROOT', target.parent)
    monkeypatch.setattr(migration, 'LINKS', {original: target})
    migration.validate_sources()
    original.write_text('node_id: 11\n')
    with pytest.raises(RuntimeError, match='Original changed since staging'):
        migration.validate_sources()
    assert original.read_text() == 'node_id: 11\n'
    assert not original.is_symlink()
