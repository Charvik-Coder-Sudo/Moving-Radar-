"""Decoder for the RDP SystemTrack packet.

The wire format is defined by the producer in
``D:\\MSDF Phase 1\\RDP\\Display\\system_track_packet.py`` (class SystemTrackPacket,
lines 43-53), imported by ``MultiTrack/FusionManager.py``. It is reproduced
here byte for byte; nothing on the producer side is changed.

    network byte order (big-endian), packed, 436 bytes, MAX_SENSORS = 4:

    systemTrackId : int32
    time          : double
    latestSensor  : int32
    Xfused        : 10 x double      [x, vx, ax, y, vy, ay, w, z, vz, az]
    numSensors    : int32            populated slots, 0..MAX_SENSORS
    sensorIds     : 4 x int32
    sensorStates  : 4 x 10 x double  same ordering as Xfused

Time is passed through exactly as sent. The RDP code treats it as seconds.

Frame (from the RDP's own plot conversion, MultiTrack/imm3dc.py Plot.calculateCartesian):
x = r cos(el) sin(az), y = r cos(el) cos(az), z = r sin(el) from the sensor's radar
range / azimuth / elevation - i.e. SENSOR-RELATIVE: x = Right, y = Forward, z = Up.
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass

FORMAT = "!idi10di4i40dBI"
PACKET_SIZE = struct.calcsize(FORMAT)
MAX_SENSORS = 4
STATE_LEN = 10
STATE_FIELDS = ("x", "vx", "ax", "y", "vy", "ay", "w", "z", "vz", "az")
assert PACKET_SIZE == 441, PACKET_SIZE

LEGACY_FORMAT = "!idi10di4i40d"
LEGACY_PACKET_SIZE = struct.calcsize(LEGACY_FORMAT)
assert LEGACY_PACKET_SIZE == 436, LEGACY_PACKET_SIZE

# The producer's own spellings (system_track_packet.AUTH_* / ICAO_NONE). Mirrored, not
# imported: that module lives in the producer tree and importing it would couple the display
# to a path outside its own project.
AUTH_UNKNOWN, AUTH_FRIEND = 0, 1
AUTH_NAMES = {AUTH_UNKNOWN: "unknown", AUTH_FRIEND: "friend"}
ICAO_NONE = 0xFFFFFFFF

#: 0x000000 is not an assignable ICAO 24-bit address, and it is also what an uninitialised
#: field contains. Treating it as "no address" keeps a zeroed packet field from being shown
#: as though it were a real aircraft identity. See docs/IDENTITY.md, upstream note 3.
ICAO_UNSET = 0x000000


def decode_auth(code: int) -> str:
    """Wire auth code -> the producer's own word for it. Unrecognised codes stay unknown."""
    return AUTH_NAMES.get(int(code), "unknown")


def decode_icao(value: int) -> str | None:
    """Wire ICAO -> '0x000FA0' as the source spells it, or None when the track has none."""
    v = int(value)
    return None if v in (ICAO_NONE, ICAO_UNSET) else f"0x{v:06X}"


class PacketError(ValueError):
    """A packet that must be rejected. ``reason`` is a short code for the statistics."""

    def __init__(self, reason: str, detail: str):
        super().__init__(f"{reason}: {detail}")
        self.reason = reason
        self.detail = detail


@dataclass(frozen=True, slots=True)
class SystemTrackState:
    """One decoded SystemTrack packet (display-side; the RDP's SystemTrack is untouched)."""

    system_track_id: int
    time: float                                   # packet time, as sent (RDP: seconds)
    latest_sensor: int
    fused_state: tuple                            # 10 floats, STATE_FIELDS order
    num_sensors: int
    sensor_ids: tuple                             # num_sensors ints
    sensor_states: tuple                          # num_sensors tuples of 10 floats
    auth: str = "unknown"                         # IFF result as sent: "friend" / "unknown"
    icao: str | None = None                       # "0x000FA0", or None when not provided
    received_wall: float = 0.0                    # receiver monotonic clock, s
    sender: str = ""


def components(state) -> dict:
    """{x, vx, ax, y, vy, ay, w, z, vz, az} of a 10-element state."""
    return dict(zip(STATE_FIELDS, state))


def decode(data: bytes, received_wall: float = 0.0, sender: str = "") -> SystemTrackState:
    """Decode one datagram, or raise PacketError. Never guesses a layout."""
    if len(data) == PACKET_SIZE:
        v = struct.unpack(FORMAT, data)
        track_id, t, latest = v[0], v[1], v[2]
        fused = tuple(v[3:13])
        num = v[13]
        ids = v[14:18]
        flat = v[18:58]
        auth_raw, icao_raw = v[58], v[59]
        auth = decode_auth(auth_raw)
        icao = decode_icao(icao_raw)
    elif len(data) == LEGACY_PACKET_SIZE:
        v = struct.unpack(LEGACY_FORMAT, data)
        track_id, t, latest = v[0], v[1], v[2]
        fused = tuple(v[3:13])
        num = v[13]
        ids = v[14:18]
        flat = v[18:58]
        auth = "unknown"
        icao = None
    else:
        raise PacketError("length", f"{len(data)} bytes, expected {PACKET_SIZE} (or legacy {LEGACY_PACKET_SIZE})")

    if not 0 <= num <= MAX_SENSORS:
        raise PacketError("sensor_count", f"numSensors = {num}, expected 0..{MAX_SENSORS}")
    if not math.isfinite(t):
        raise PacketError("numeric", f"time = {t}")
    if not all(math.isfinite(x) for x in fused):
        raise PacketError("numeric", f"Xfused contains non-finite values (systemTrackId {track_id})")
    states = tuple(tuple(flat[i * STATE_LEN:(i + 1) * STATE_LEN]) for i in range(num))
    for sid, st in zip(ids[:num], states):
        if not all(math.isfinite(x) for x in st):
            raise PacketError("numeric", f"sensor {sid} state contains non-finite values "
                                         f"(systemTrackId {track_id})")
    return SystemTrackState(track_id, t, latest, fused, num, tuple(ids[:num]), states,
                            auth, icao, received_wall, sender)


def encode_for_test(track_id: int, t: float, latest: int, fused, sensor_ids, sensor_states,
                    auth: int = AUTH_UNKNOWN, icao: int = ICAO_NONE) -> bytes:
    """Pack with the same format (used only by the unit tests to round-trip the decoder)."""
    ids = list(sensor_ids) + [0] * (MAX_SENSORS - len(sensor_ids))
    flat = [float(x) for st in sensor_states for x in st]
    flat += [0.0] * (MAX_SENSORS * STATE_LEN - len(flat))
    return struct.pack(FORMAT, track_id, t, latest, *fused, len(sensor_ids), *ids, *flat,
                       int(auth), int(icao))


def encode_legacy_for_test(track_id: int, t: float, latest: int, fused, sensor_ids, sensor_states) -> bytes:
    """Pack with the legacy 436-byte format (used by unit tests to verify backward compatibility)."""
    ids = list(sensor_ids) + [0] * (MAX_SENSORS - len(sensor_ids))
    flat = [float(x) for st in sensor_states for x in st]
    flat += [0.0] * (MAX_SENSORS * STATE_LEN - len(flat))
    return struct.pack(LEGACY_FORMAT, track_id, t, latest, *fused, len(sensor_ids), *ids, *flat)

