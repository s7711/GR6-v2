"""Wire-format for `drive/firmware/GR6_motor.ino` (v250819) — parsing
telemetry lines and building command lines. Deliberately knows nothing
about real-world units (m/s, metres) or which caller is allowed to send
what — that's `serial_link.py`/`control.py`/`app.py`'s job. This module
is just the raw contract with the microcontroller, same role
`ncomrx.py` plays for the xNAV650's NCOM protocol.

See drive-prd.md ("Firmware interface") for the full command/telemetry
table this implements.
"""

# Tuning parameters the firmware encodes as float x100 (both when
# echoing telemetry and when accepting a command).
SCALED_TUNING_PARAMS = {"Kp", "Ki", "Kd", "Kf", "Ka", "Kb", "Id"}

# Tuning parameters the firmware treats as plain integers.
RAW_TUNING_PARAMS = {"Db", "Mi", "Mj", "Am"}

ALL_TUNING_PARAMS = SCALED_TUNING_PARAMS | RAW_TUNING_PARAMS

_PAIR_FIELDS_SCALED = {
    "SV": ("LM_setvel", "RM_setvel"),
    "FV": ("LM_vel_filt", "RM_vel_filt"),
    "ER": ("LM_err", "RM_err"),
    "EI": ("LM_integral", "RM_integral"),
    "ED": ("LM_derr", "RM_derr"),
}

_PAIR_FIELDS_RAW = {
    "EN": ("LM_position", "RM_position"),
    "MO": ("LM_out", "RM_out"),
}


def parse_line(line: str) -> dict:
    """Parse one telemetry line into a dict of field updates. Returns an
    empty dict for a blank, malformed, or unrecognised line — callers
    should just merge whatever comes back into their running state."""
    parts = line.strip().split()
    if not parts:
        return {}
    tag = parts[0]

    try:
        if tag in _PAIR_FIELDS_SCALED and len(parts) == 3:
            left_field, right_field = _PAIR_FIELDS_SCALED[tag]
            return {left_field: int(parts[1]) / 100.0, right_field: int(parts[2]) / 100.0}

        if tag in _PAIR_FIELDS_RAW and len(parts) == 3:
            left_field, right_field = _PAIR_FIELDS_RAW[tag]
            return {left_field: int(parts[1]), right_field: int(parts[2])}

        if tag == "WP" and len(parts) == 2:
            return {"pump": bool(int(parts[1]))}

        if tag == "GO" and len(parts) == 2:
            return {"ctrl_enabled": int(parts[1])}

        if len(tag) == 2 and tag[0] == "U" and tag[1].isdigit() and len(parts) == 2:
            return {f"ultrasonic_{tag[1]}_mm": int(parts[1])}

        if tag in ALL_TUNING_PARAMS and len(parts) == 3:
            if tag in SCALED_TUNING_PARAMS:
                left, right = int(parts[1]) / 100.0, int(parts[2]) / 100.0
            else:
                left, right = int(parts[1]), int(parts[2])
            return {f"tuning_{tag}_L": left, f"tuning_{tag}_R": right}

        if tag == "Version" and len(parts) >= 2:
            return {"firmware_version": parts[1]}
    except ValueError:
        return {}

    return {}


def encode_set_velocity(left_counts_s: float, right_counts_s: float) -> str:
    """`SV <left> <right>` — firmware clamps to +/-200 itself, and treats
    `SV 0 0` specially (disables the control loop rather than commanding
    zero through the PID loop) — see drive-prd.md."""
    return f"SV {int(round(left_counts_s))} {int(round(right_counts_s))}\n"


def encode_pump(on: bool) -> str:
    return f"WP {1 if on else 0}\n"


def encode_tuning(name: str, left: float, right: float) -> str:
    if name not in ALL_TUNING_PARAMS:
        raise ValueError(f"Unknown tuning parameter: {name!r}")
    if name in SCALED_TUNING_PARAMS:
        left, right = round(left * 100), round(right * 100)
    else:
        left, right = round(left), round(right)
    return f"{name} {int(left)} {int(right)}\n"
