"""Render the description from the same validated physical profile as control."""
from pathlib import Path


def render_description(cfg, path=None):
    import xacro
    if path is None:
        try:
            from ament_index_python.packages import get_package_share_directory
            directory = Path(get_package_share_directory('mobotic_description'))
        except ImportError:
            directory = Path(__file__).resolve().parents[2] / 'mobotic_description'
        path = directory / 'urdf/moboterra.urdf.xacro'
    return xacro.process_file(str(path), mappings={'platform_config': str(cfg.path)}).toxml()
