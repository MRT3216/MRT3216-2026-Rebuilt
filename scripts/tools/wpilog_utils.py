"""
wpilog_utils.py — Shared utilities for reading and analyzing WPILog files.

Provides:
    - Log entry parsing and AdvantageKit double decoding
    - Match phase detection (AUTO / TELEOP / DISABLED)
    - Struct decoding for SwerveModuleState[] and ChassisSpeeds
    - Common device / side classification for MRT3216 robot

Usage:
    from wpilog_utils import LogReader
    lr = LogReader("path/to/match.wpilog")
    phases = lr.get_phases()
    auto_data = lr.get_data_in_phase("AUTO", "/RealOutputs/SwerveChassisSpeeds/Setpoints")
"""

import math
import re
import struct as _struct
from collections import defaultdict
from pathlib import Path
from typing import Any

from wpiutil.log import DataLogReader


# ═══════════════════════════════════════════════════════════════════
#  AdvantageKit encoding helpers
# ═══════════════════════════════════════════════════════════════════

def ak_double(raw_int: int) -> float:
    """Decode an AdvantageKit double that was encoded as int64 bit-pattern."""
    return _struct.unpack("d", _struct.pack("q", raw_int))[0]


def decode_swerve_states(raw_bytes: bytes) -> list[tuple[float, float]]:
    """Decode SwerveModuleState[] → list of (speed_mps, angle_rad)."""
    n = len(raw_bytes) // 16
    return [
        _struct.unpack_from("<dd", raw_bytes, i * 16)
        for i in range(n)
    ]


def decode_chassis_speeds(raw_bytes: bytes) -> tuple[float, float, float] | None:
    """Decode ChassisSpeeds → (vx, vy, omega) or None if too short."""
    if len(raw_bytes) >= 24:
        return _struct.unpack_from("<ddd", raw_bytes, 0)
    return None


# ═══════════════════════════════════════════════════════════════════
#  MRT3216 device identity maps
# ═══════════════════════════════════════════════════════════════════

MODULE_NAMES = {
    "Module0": ("FL", "Front Left  — CAN 11/12/13"),
    "Module1": ("FR", "Front Right — CAN 21/22/23"),
    "Module2": ("BL", "Back Left   — CAN 41/42/43"),
    "Module3": ("BR", "Back Right  — CAN 31/32/33"),
}

CAMERA_NAMES = {
    "Camera0": ("Front", "Front_Cam"),
    "Camera1": ("Left",  "Left_Cam"),
    "Camera2": ("Right", "Right_Cam"),
    "Camera3": ("Rear",  "Rear_Cam"),
}

SIDE_MAP = {
    "Module0": "LEFT",  "Module1": "RIGHT",
    "Module2": "LEFT",  "Module3": "RIGHT",
    "Camera0": "CENTER", "Camera1": "LEFT",
    "Camera2": "RIGHT",  "Camera3": "CENTER",
}


def classify_device(name: str) -> tuple[str, str, str]:
    """Return (short_key, friendly_name, side) for a log key containing a device."""
    for key, (short, desc) in MODULE_NAMES.items():
        if key in name:
            sub = name.split(key + "/")[-1].replace("Connected", "").rstrip("/") or "Module"
            return f"{short}/{sub}", f"{key} {sub} ({desc})", SIDE_MAP[key]
    for key, (short, desc) in CAMERA_NAMES.items():
        if key in name:
            return f"Cam/{short}", f"{key} ({desc})", SIDE_MAP[key]
    if "OrangePI-Left" in name:
        return "OPi/Left", "OrangePI-Left (coprocessor)", "LEFT"
    if "OrangePI-Right" in name:
        return "OPi/Right", "OrangePI-Right (coprocessor)", "RIGHT"
    if "Pigeon" in name:
        return "Pigeon", "Pigeon2 IMU (CAN 10)", "CENTER"
    short = name.split("/")[-1].replace("Connected", "")[:15]
    return short, name, "?"


def classify_side(name: str) -> str:
    """Return LEFT / RIGHT / CENTER / ? for a device name."""
    return classify_device(name)[2]


def extract_match_label(filename: str) -> str:
    """Extract 'Q6', 'Q55' etc. from a log filename."""
    m = re.search(r"_q(\d+)", filename, re.IGNORECASE)
    return f"Q{m.group(1)}" if m else Path(filename).stem[:20]


# ═══════════════════════════════════════════════════════════════════
#  LogReader — main entry point
# ═══════════════════════════════════════════════════════════════════

class LogReader:
    """High-level reader for AdvantageKit WPILog files.

    Example::

        lr = LogReader("match.wpilog")
        for phase_name, start, end in lr.get_phases():
            print(f"{phase_name}: {start:.1f}s – {end:.1f}s")
        setp = lr.get_timeseries("/RealOutputs/SwerveChassisSpeeds/Setpoints")
    """

    def __init__(self, path: str):
        self.path = Path(path)
        if not self.path.exists():
            raise FileNotFoundError(f"Log not found: {self.path}")

        # ── Pass 1: build entry map ────────────────────────────
        self.entries: dict[int, tuple[str, str]] = {}
        reader = DataLogReader(str(self.path))
        for rec in reader:
            if rec.isStart():
                d = rec.getStartData()
                self.entries[d.entry] = (d.name, d.type)

        # ── Pass 2: extract all data ──────────────────────────
        self._data: dict[str, list[tuple[float, Any]]] = defaultdict(list)
        reader2 = DataLogReader(str(self.path))
        for rec in reader2:
            if rec.isStart() or rec.isFinish() or rec.isControl() or rec.isSetMetadata():
                continue
            eid = rec.getEntry()
            if eid not in self.entries:
                continue
            name, typ = self.entries[eid]
            ts = rec.getTimestamp() / 1e6
            try:
                val = self._decode_record(rec, name, typ)
                if val is not None:
                    self._data[name].append((ts, val))
            except Exception:
                pass

        # ── Cache phases ──────────────────────────────────────
        self._phases: list[tuple[str, float, float]] | None = None

    def _decode_record(self, rec, name: str, typ: str) -> Any:
        """Decode a single record based on its type string."""
        tl = typ.lower()
        if tl == "boolean":
            return rec.getBoolean()
        elif tl == "int64" or tl == "integer":
            return rec.getInteger()
        elif tl == "float":
            return rec.getFloat()
        elif tl == "double":
            return rec.getDouble()
        elif tl == "string":
            return rec.getString()
        elif "struct:swervemodulestate" in tl:
            raw = bytes(rec.getRaw())
            if len(raw) >= 16:
                return decode_swerve_states(raw)
            return None
        elif "struct:chassisspeeds" in tl:
            raw = bytes(rec.getRaw())
            return decode_chassis_speeds(raw)
        elif "struct:" in tl:
            return bytes(rec.getRaw())
        elif tl == "raw":
            return bytes(rec.getRaw())
        else:
            # Many AK keys store doubles as int64
            try:
                raw_int = rec.getInteger()
                val = ak_double(raw_int)
                if -1e6 < val < 1e6:  # sanity check
                    return val
            except Exception:
                pass
            try:
                return rec.getDouble()
            except Exception:
                return None

    # ── Key discovery ──────────────────────────────────────────

    def list_keys(self, pattern: str | None = None) -> list[tuple[str, str, int]]:
        """Return [(name, type, sample_count)] for all keys, optionally filtered by regex."""
        results = []
        for eid, (name, typ) in self.entries.items():
            if pattern and not re.search(pattern, name, re.IGNORECASE):
                continue
            count = len(self._data.get(name, []))
            results.append((name, typ, count))
        results.sort(key=lambda x: x[0])
        return results

    # ── Time series access ─────────────────────────────────────

    def get_timeseries(self, key: str) -> list[tuple[float, Any]]:
        """Return raw [(timestamp, value)] for a key."""
        return self._data.get(key, [])

    def get_doubles(self, key: str) -> list[tuple[float, float]]:
        """Return [(timestamp, float)] — handles AK int64-encoded doubles."""
        raw = self._data.get(key, [])
        result = []
        for ts, val in raw:
            if isinstance(val, (int,)):
                val = ak_double(val)
            if isinstance(val, float) and -1e6 < val < 1e6:
                result.append((ts, val))
        return result

    # ── Phase detection ────────────────────────────────────────

    def get_phases(self) -> list[tuple[str, float, float]]:
        """Return [(phase_name, start_ts, end_ts)] for AUTO/TELEOP/DISABLED."""
        if self._phases is not None:
            return self._phases

        en_data = self._data.get("/DriverStation/Enabled", [])
        au_data = self._data.get("/DriverStation/Autonomous", [])

        events = [(t, "e", v) for t, v in en_data] + [(t, "a", v) for t, v in au_data]
        events.sort()

        enabled = False
        autonomous = False
        phases = []
        phase_start = None
        last_phase = None

        for t, kind, val in events:
            if kind == "e":
                enabled = val
            elif kind == "a":
                autonomous = val

            if enabled and autonomous:
                phase = "AUTO"
            elif enabled:
                phase = "TELEOP"
            else:
                phase = "DISABLED"

            if phase != last_phase:
                if last_phase and phase_start is not None:
                    phases.append((last_phase, phase_start, t))
                phase_start = t
                last_phase = phase

        if last_phase and phase_start is not None:
            last_ts = max(ts for ts, _ in en_data) if en_data else phase_start + 1
            phases.append((last_phase, phase_start, last_ts))

        self._phases = phases
        return phases

    def get_phase_range(self, phase_name: str) -> list[tuple[float, float]]:
        """Return [(start, end)] for all segments of the given phase."""
        return [(s, e) for n, s, e in self.get_phases() if n == phase_name]

    def in_phase(self, ts: float, phase_name: str) -> bool:
        """Check if a timestamp falls within a given phase."""
        return any(s <= ts <= e for s, e in self.get_phase_range(phase_name))

    def get_data_in_phase(self, phase_name: str, key: str) -> list[tuple[float, Any]]:
        """Return timeseries data filtered to only the given phase."""
        ranges = self.get_phase_range(phase_name)
        return [
            (ts, val)
            for ts, val in self._data.get(key, [])
            if any(s <= ts <= e for s, e in ranges)
        ]

    # ── Convenience: auto window ───────────────────────────────

    def get_auto_window(self) -> tuple[float, float] | None:
        """Return (start, end) for the first AUTO phase, or None."""
        for name, s, e in self.get_phases():
            if name == "AUTO":
                return (s, e)
        return None
