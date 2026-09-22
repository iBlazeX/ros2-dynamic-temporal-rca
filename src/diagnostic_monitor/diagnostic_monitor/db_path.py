"""Single, shared resolution of the SQLite event-store path.

Every process that opens the event store (diagnostic monitor, rca_cli,
experiment controller) must resolve the *same* default path regardless of the
directory it was started from. Resolution order:

    1. explicit argument            (CLI ``--db``)
    2. explicit ROS parameter       (``db_path`` on the monitor; '' means unset)
    3. environment variable         ``RCA_DB_PATH``
    4. workspace-aware default      ``<colcon workspace>/events.db`` where the
                                    workspace is derived from the installed
                                    location of this package
                                    (``<ws>/install/diagnostic_monitor/...``)
    5. last resort                  ``~/.ros/rca/events.db``

A relative path given explicitly (1-3) is kept relative to the current working
directory, because that is what the caller asked for. The defaults (4-5) are
always absolute, so they never depend on the working directory.
"""

import os
from typing import Mapping, Optional

ENV_VAR = 'RCA_DB_PATH'
DB_FILENAME = 'events.db'
PACKAGE_NAME = 'diagnostic_monitor'
FALLBACK_DIR = os.path.join('~', '.ros', 'rca')

HELP_TEXT = (
    'Path to the SQLite event store. Default resolution: --db > ROS parameter db_path > '
    f'${ENV_VAR} > <workspace>/{DB_FILENAME} (workspace derived from the installed package) > '
    f'{FALLBACK_DIR}/{DB_FILENAME}. The default never depends on the current directory.'
)


def _package_share_dir(package: str = PACKAGE_NAME) -> Optional[str]:
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory(package)
    except Exception:
        return None


def workspace_root_from_share_dir(share_dir: Optional[str]) -> Optional[str]:
    """Derives ``<ws>`` from ``<ws>/install/<pkg>/share/<pkg>`` (any install layout).

    Walks up from the share directory looking for a component named ``install``;
    the parent of that directory is the colcon workspace root. Returns None if the
    package is not installed inside a colcon workspace (e.g. a system install).
    """
    if not share_dir:
        return None
    path = os.path.abspath(share_dir)
    parts = path.split(os.sep)
    for i in range(len(parts) - 1, 0, -1):
        if parts[i] == 'install':
            root = os.sep.join(parts[:i]) or os.sep
            return root
    return None


def default_db_path(share_dir: Optional[str] = None, home: Optional[str] = None) -> str:
    """Workspace-aware default (rule 4) with the ``~/.ros/rca`` fallback (rule 5)."""
    if share_dir is None:
        share_dir = _package_share_dir()
    ws = workspace_root_from_share_dir(share_dir)
    if ws:
        return os.path.join(ws, DB_FILENAME)
    home = home or os.path.expanduser('~')
    return os.path.join(home, '.ros', 'rca', DB_FILENAME)


def resolve_db_path(
    explicit: Optional[str] = None,
    ros_param: Optional[str] = None,
    env: Optional[Mapping[str, str]] = None,
    share_dir: Optional[str] = None,
) -> str:
    """Applies the resolution order documented in this module."""
    env = os.environ if env is None else env
    for candidate in (explicit, ros_param, env.get(ENV_VAR)):
        if candidate is not None and str(candidate).strip() != '':
            return os.path.expanduser(str(candidate))
    return default_db_path(share_dir=share_dir)


def ensure_parent_dir(path: str) -> str:
    """Creates the parent directory of a DB path if needed (for the fallback dir)."""
    parent = os.path.dirname(os.path.abspath(path))
    if parent and not os.path.isdir(parent):
        os.makedirs(parent, exist_ok=True)
    return path
