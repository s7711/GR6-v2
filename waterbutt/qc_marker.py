"""Stores the single saved "ideal position" QC marker record used to
sanity-check the robot's pose (via a live ArUco reading, independent of
GNSS) before an automatic fill - see waterbutt-prd.md's "QC marker"
section. Deliberately just one record, not a list keyed by name like
navigate/jobs's paths - only one waterbutt, one marker allowed, always
overwritten on save.
"""

import copy
import sys
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from shared.file_cache import FileCache, load_yaml  # noqa: E402

# load() runs for every aruco message (app.py's _qc_loop, a few Hz, all
# day) - cached so that's a stat(), not a read and a YAML parse, unless the
# marker has actually been re-saved (shared/file_cache.py).
_cache = FileCache()


def load(path) -> dict | None:
    path = Path(path)
    try:
        saved = _cache.get(path, load_yaml)
    except FileNotFoundError:
        return None
    return copy.deepcopy(saved)  # the cached object is shared - see file_cache.py


def save(path, marker_id: int, tvec_camera_frame: list, rvec_camera_frame: list) -> None:
    # Raw tvec/rvec, not just the body-frame displacement - the rotation-
    # aware comparison (coords.qc_marker_delta_body_frame) needs both
    # readings' actual orientation, not just their derived positions.
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        yaml.safe_dump(
            {"marker_id": marker_id, "tvec_camera_frame": tvec_camera_frame, "rvec_camera_frame": rvec_camera_frame},
            sort_keys=False,
        )
    )
