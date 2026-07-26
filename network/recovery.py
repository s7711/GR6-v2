"""Per-interface recovery profiles and the auto-revert safety net (see
network-prd.md's "Recovery / safe-apply") — applying a network change
over the network being changed risks locking yourself out, so every
apply starts a countdown; if it's not confirmed in time, every
interface with a defined recovery profile gets that profile
re-activated.

Recovery is "which already-existing NetworkManager connection should
this device fall back to", not a duplicate copy of its settings — so
reverting is just `nm.activate_connection(name)`, and this module never
needs to know *what* a recovery profile actually configures.
"""

import json
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import nm  # noqa: E402

RECOVERY_FILE = Path(__file__).resolve().parent / "recovery.json"
REVERT_UNIT = "gr6-network-revert"  # fixed name so a pending timer can be found/cancelled


def load_recovery() -> dict:
    """{device: connection_name} — a device with no entry (or an
    explicit null) means "don't care, leave alone" on revert (this is
    what eth0 gets, per Ben's call — see network-prd.md)."""
    if not RECOVERY_FILE.exists():
        return {}
    return json.loads(RECOVERY_FILE.read_text())


def set_recovery(device: str, connection_name: str | None) -> None:
    recovery = load_recovery()
    if connection_name is None:
        recovery.pop(device, None)
    else:
        recovery[device] = connection_name
    RECOVERY_FILE.write_text(json.dumps(recovery, indent=2))


def revert_now() -> None:
    """Re-activate every device's stored recovery connection. This is
    what fires if a change is never confirmed — see schedule_revert."""
    for device, connection_name in load_recovery().items():
        try:
            nm.activate_connection(connection_name)
        except nm.NmError:
            # Best-effort across all devices — one failing (e.g. the
            # recovery connection itself got deleted) shouldn't stop the
            # others from reverting.
            pass


def schedule_revert(timeout_s: int) -> None:
    """Start (replacing any existing) countdown — after timeout_s with
    no confirm_pending() call, revert_now() runs. Uses systemd-run
    rather than an in-process thread/timer so the revert still fires
    even if this Flask process itself is what a bad change knocks
    over."""
    cancel_pending()
    subprocess.run(
        [
            "sudo", "systemd-run",
            f"--unit={REVERT_UNIT}",
            f"--on-active={timeout_s}s",
            "--description=GR6 network auto-revert",
            sys.executable, str(Path(__file__).resolve().parent / "revert_cli.py"),
        ],
        check=True,
    )


def cancel_pending() -> None:
    """Confirming a change calls this — stops the countdown before it
    fires. Safe to call when nothing is pending."""
    subprocess.run(["sudo", "systemctl", "stop", f"{REVERT_UNIT}.timer"], capture_output=True)
    subprocess.run(["sudo", "systemctl", "reset-failed", f"{REVERT_UNIT}.timer"], capture_output=True)


def revert_pending() -> bool:
    result = subprocess.run(
        ["systemctl", "is-active", f"{REVERT_UNIT}.timer"], capture_output=True, text=True
    )
    return result.stdout.strip() == "active"
