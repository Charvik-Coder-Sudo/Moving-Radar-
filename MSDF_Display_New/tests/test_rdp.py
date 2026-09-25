"""RDP SystemTrack packet decoding, checked against the producer's own definition.

The producer (``D:\\MSDF Phase 1\\RDP\\Display\\system_track_packet.py``, imported by
MultiTrack/FusionManager.py) is loaded read-only with bytecode writing disabled, so
nothing is written next to it. Its ``SystemTrackPacket.from_system_track`` builds the
bytes that the display must decode.
"""

import ctypes
import importlib.util
import os
import math
import struct
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

APP = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(APP))

from rdp.packet import (FORMAT, MAX_SENSORS, PACKET_SIZE, STATE_FIELDS, PacketError,  # noqa: E402
                        components, decode, encode_for_test)

# The producer's own ctypes definition, when this machine has the RDP project. It lives outside
# this project, so the path is taken from MSDF_PACKET_PRODUCER (or the usual places beside the
# project) and the cross-check is skipped when it is not there - it is a bonus, not a requirement.
_PRODUCER_CANDIDATES = [
    Path(p) for p in [os.environ.get("MSDF_PACKET_PRODUCER", "")] if p
] + [APP.parent / "Display" / "system_track_packet.py",
     APP.parent / "MultiTrack" / "system_track_packet.py"]
PRODUCER = next((p for p in _PRODUCER_CANDIDATES if p.is_file()), _PRODUCER_CANDIDATES[-1])


def load_producer():
    old = sys.dont_write_bytecode
    sys.dont_write_bytecode = True
    try:
        spec = importlib.util.spec_from_file_location("rdp_producer_system_track_packet", PRODUCER)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        return mod
    finally:
        sys.dont_write_bytecode = old


def state(base):
    return [base + i + 0.25 for i in range(10)]


def system_track(track_id=7, t=12.5, latest=2, fused=None, sensors=None):
    """A stand-in with the attributes SystemTrackPacket.from_system_track reads."""
    sensors = {1: state(100.0), 2: state(200.0)} if sensors is None else sensors
    return SimpleNamespace(systemTrackId=track_id, time=t, latestSensor=latest,
                           Xfused=fused if fused is not None else state(0.0), trackStateMap=sensors)


@unittest.skipUnless(PRODUCER.is_file(), f"producer definition not present: {PRODUCER}")
class TestAgainstProducer(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.mod = load_producer()

    def packet(self, **kw) -> bytes:
        return self.mod.SystemTrackPacket.from_system_track(system_track(**kw)).to_bytes()

    def test_size_and_cap(self):
        self.assertEqual(PACKET_SIZE, 436)
        self.assertEqual(ctypes.sizeof(self.mod.SystemTrackPacket), PACKET_SIZE)
        self.assertEqual(self.mod.MAX_SENSORS, MAX_SENSORS)
        self.assertEqual(self.mod.XFUSED_LEN, len(STATE_FIELDS))

    def test_decode_producer_bytes(self):
        fused = [1.5, -2.0, 0.1, 30000.0, 250.0, -0.2, 0.01, 5000.0, 3.0, 0.0]
        st = decode(self.packet(track_id=42, t=815.25, latest=1, fused=fused,
                                sensors={2: state(200.0), 1: state(100.0)}))
        self.assertEqual(st.system_track_id, 42)
        self.assertEqual(st.time, 815.25)                          # passed through as sent
        self.assertEqual(st.latest_sensor, 1)
        self.assertEqual(st.fused_state, tuple(fused))
        self.assertEqual(st.num_sensors, 2)
        self.assertEqual(st.sensor_ids, (1, 2))                    # producer sorts by sensorId
        self.assertEqual(st.sensor_states, (tuple(state(100.0)), tuple(state(200.0))))

    def test_big_endian(self):
        raw = self.packet(track_id=0x01020304, t=1.0)
        self.assertEqual(raw[:4], b"\x01\x02\x03\x04")
        self.assertEqual(raw[4:12], struct.pack(">d", 1.0))
        self.assertEqual(decode(raw).system_track_id, 0x01020304)

    def test_zero_and_max_sensors(self):
        self.assertEqual(decode(self.packet(sensors={})).sensor_ids, ())
        four = {i: state(i * 10.0) for i in (4, 3, 2, 1)}
        st = decode(self.packet(sensors=four))
        self.assertEqual(st.sensor_ids, (1, 2, 3, 4))
        self.assertEqual(st.sensor_states[3], tuple(state(40.0)))

    def test_test_encoder_matches_producer(self):
        ours = encode_for_test(7, 12.5, 2, state(0.0), [1, 2], [state(100.0), state(200.0)])
        self.assertEqual(ours, self.packet())


class TestDecoder(unittest.TestCase):
    def good(self, **kw):
        args = dict(track_id=3, t=10.0, latest=1, fused=state(0.0), sensor_ids=[1], sensor_states=[state(5.0)])
        args.update(kw)
        return encode_for_test(args["track_id"], args["t"], args["latest"], args["fused"],
                               args["sensor_ids"], args["sensor_states"])

    def assertRejected(self, data, reason):
        with self.assertRaises(PacketError) as cm:
            decode(data)
        self.assertEqual(cm.exception.reason, reason)

    def test_state_order(self):
        c = components(tuple(range(10)))
        self.assertEqual(c, {"x": 0, "vx": 1, "ax": 2, "y": 3, "vy": 4, "ay": 5, "w": 6, "z": 7, "vz": 8, "az": 9})

    def test_wrong_length(self):
        raw = self.good()
        for data in (b"", raw[:-1], raw + b"\x00", raw[:100]):
            self.assertRejected(data, "length")

    def test_sensor_count(self):
        raw = bytearray(self.good())
        offset = struct.calcsize("!idi10d")
        for bad in (5, -1, 1000):
            raw[offset:offset + 4] = struct.pack("!i", bad)
            self.assertRejected(bytes(raw), "sensor_count")

    def test_non_finite(self):
        self.assertRejected(self.good(t=math.nan), "numeric")
        self.assertRejected(self.good(t=math.inf), "numeric")
        fused = state(0.0)
        fused[7] = math.nan
        self.assertRejected(self.good(fused=fused), "numeric")
        s = state(5.0)
        s[1] = -math.inf
        self.assertRejected(self.good(sensor_states=[s]), "numeric")

    def test_unused_slots_are_ignored(self):
        v = list(struct.unpack(FORMAT, self.good()))
        v[18 + 10:] = [math.nan] * 30                              # slots 2..4 unused (numSensors = 1)
        v[15:18] = [99, 99, 99]
        st = decode(struct.pack(FORMAT, *v))
        self.assertEqual(st.sensor_ids, (1,))
        self.assertEqual(len(st.sensor_states), 1)


if __name__ == "__main__":
    unittest.main()
