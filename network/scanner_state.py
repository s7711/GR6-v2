"""Persisted "which wifi device (if any) is set to Scanner mode" choice.

A Scanner-mode device is otherwise indistinguishable from a plain "Off"
one at the NetworkManager level — both are just `nmcli device set
<dev> managed no` (see app.py's `_apply_batch`). This tiny file is the
only thing telling them apart: it's what app.py's own background scan
loop reads to know what (if anything) to scan, and what the shared
header's "Wifi"/"Wifi+" badge reads (via shared/sysstats.py) to know
whether scanning/logging is currently happening. Deliberately a small
side-file rather than a config.yaml value or new database, same
reasoning as wheelspeed's gad_switch.py: this needs to change live,
without a restart, and be visible to a second process (sysstats.py)
without a network round-trip.
"""

import json
from pathlib import Path


class ScannerState:
    def __init__(self, path: Path):
        self.path = path

    def get_device(self) -> str | None:
        try:
            return json.loads(self.path.read_text()).get("device")
        except (FileNotFoundError, json.JSONDecodeError):
            return None

    def set_device(self, device: str | None) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.path.write_text(json.dumps({"device": device}))

    def clear(self) -> None:
        self.set_device(None)
