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

from rdp.packet import (AUTH_FRIEND, AUTH_UNKNOWN, FORMAT, ICAO_NONE,  # noqa: E402
                        MAX_SENSORS, PACKET_SIZE, STATE_FIELDS, PacketError, components,
                        decode, decode_icao, encode_for_test)

# The producer's own ctypes definition, when this machine has the RDP project. It lives outside
# this project, so the path is taken from MSDF_PACKET_PRODUCER (or the usual places beside the
# project) and the cross-check is skipped when it is not there - it is a bonus, not a requirement.
_PRODUCER_CANDIDATES = [
    Path(p) for p in [os.environ.get("MSDF_PACKET_PRODUCER", "")] if p
] + [parent / folder / "system_track_packet.py"
     for parent in [APP.parent, *APP.parents]
     for folder in ("Display", "MultiTrack")]
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


def system_track(track_id=7, t=12.5, latest=2, fused=None, sensors=None,
                 auth="unknown", icao=None):
    """A stand-in with the attributes SystemTrackPacket.from_system_track reads."""
    sensors = {1: state(100.0), 2: state(200.0)} if sensors is None else sensors
    return SimpleNamespace(systemTrackId=track_id, time=t, latestSensor=latest,
                           Xfused=fused if fused is not None else state(0.0), trackStateMap=sensors,
                           auth=auth, icao=icao)


@unittest.skipUnless(PRODUCER.is_file(), f"producer definition not present: {PRODUCER}")
class TestAgainstProducer(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.mod = load_producer()

    def packet(self, **kw) -> bytes:
        return self.mod.SystemTrackPacket.from_system_track(system_track(**kw)).to_bytes()

    def test_size_and_cap(self):
        # 441 since the producer added the IFF identity fields (auth uint8 + icao uint32).
        # Pinned against the producer's own struct below, so the two can never drift.
        self.assertEqual(PACKET_SIZE, 441)
        self.assertEqual(ctypes.sizeof(self.mod.SystemTrackPacket), PACKET_SIZE)
        self.assertEqual(self.mod.MAX_SENSORS, MAX_SENSORS)
        self.assertEqual(self.mod.XFUSED_LEN, len(STATE_FIELDS))

    def test_identity_survives_the_real_producer(self):
        """The whole point of the interface: an IFF result decided in MultiTrack has to
        arrive here unchanged. Encoded by the PRODUCER'S OWN code, decoded by ours."""
        st = decode(self.packet(auth="friend", icao="0x000FA0"))
        self.assertEqual(st.auth, "friend")
        self.assertEqual(st.icao, "0x000FA0")                 # the source's own spelling

    def test_a_track_with_no_iff_reply_carries_no_address(self):
        st = decode(self.packet(auth="unknown", icao=None))
        self.assertEqual(st.auth, "unknown")
        self.assertIsNone(st.icao)

    def test_the_producers_absent_marker_is_the_one_we_expect(self):
        """Pins our ICAO_NONE against the producer's, so a change upstream fails here rather
        than silently turning 'no address' into a real-looking one."""
        self.assertEqual(self.mod.ICAO_NONE, ICAO_NONE)
        self.assertEqual(self.mod.AUTH_FRIEND, AUTH_FRIEND)
        self.assertEqual(self.mod.AUTH_UNKNOWN, AUTH_UNKNOWN)

    def test_a_zeroed_icao_field_is_not_shown_as_an_address(self):
        """0x000000 is not an assignable ICAO address, and it is what an unset field holds.
        Upstream can emit it (see docs/IDENTITY.md, upstream note 3), so it must never reach
        the operator looking like a real identity."""
        self.assertIsNone(decode_icao(0x000000))
        self.assertEqual(decode_icao(0x000FA0), "0x000FA0")

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

    def test_legacy_packet_backward_compat(self):
        """A legacy 436-byte packet without auth/icao must decode cleanly as unknown/None."""
        from rdp.packet import encode_legacy_for_test, LEGACY_PACKET_SIZE
        raw = encode_legacy_for_test(99, 100.5, 1, state(0.0), [1], [state(10.0)])
        self.assertEqual(len(raw), LEGACY_PACKET_SIZE)
        st = decode(raw)
        self.assertEqual(st.system_track_id, 99)
        self.assertEqual(st.time, 100.5)
        self.assertEqual(st.auth, "unknown")
        self.assertIsNone(st.icao)


if __name__ == "__main__":
    unittest.main()

