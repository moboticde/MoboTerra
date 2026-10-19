"""Read the host supervisor's atomic state without depending on CAN or Docker."""
import json
import math
from pathlib import Path


def read_host_health(path, now_seconds, timeout=5.0):
    """Return (valid, fault, reason); missing/stale/malformed state fails closed."""
    try:
        state = json.loads(Path(path).read_text(encoding='utf-8'))
        updated = state['updated_at']
        if (type(updated) not in (int, float) or not math.isfinite(updated)
                or not 0 <= now_seconds - updated <= timeout):
            return False, True, 'HOST_STATE_STALE'
        if state['state'] != 'RUNNING':
            return True, True, 'HOST_ERROR: ' + str(state.get('detail', state['state']))
        return True, False, ''
    except (OSError, ValueError, TypeError, KeyError):
        return False, True, 'HOST_STATE_UNAVAILABLE'
