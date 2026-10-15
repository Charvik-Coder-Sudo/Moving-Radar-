"""Prove what the MultiTrack interface actually delivers to this display.

    python verify_multitrack_interface.py                     replay the recorded sensor logs
    python verify_multitrack_interface.py --save real.pkts    ... and keep the real packets
    python verify_multitrack_interface.py --listen            decode live UDP instead

Why the replay mode exists
--------------------------
A unit test can only prove the display decodes a packet that the TEST built. It proves nothing
about what MultiTrack puts on the wire. This drives the real chain:

    primarySensorTrackLogs.bin / secondarySensorTrackLogs.bin   (recorded by trackManager)
      -> config.TrackPacket                      MultiTrack's own sensor-track packet
      -> FusionManager / SystemTrack             MultiTrack's own fusion and identity handling
      -> system_track_packet.SystemTrackPacket   the producer's own encoder
      -> rdp.packet.decode                       this display's decoder

and reports, with counts, how much identity survives each stage.

NOTHING under D:\\Moving Radar\\MultiTrack is written. The logs are opened read-only and the
fusion objects are driven in-process: no socket is opened and no log is regenerated.
"""

from __future__ import annotations

import argparse
import collections
import contextlib
import ctypes
import io
import json
import sys
from pathlib import Path

APP_DIR = Path(__file__).resolve().parent
if str(APP_DIR) not in sys.path:
    sys.path.insert(0, str(APP_DIR))

import paths  # noqa: E402

DEFAULT_MULTITRACK = Path(r"D:\Moving Radar\MultiTrack")
DEFAULT_PRODUCER = Path(r"D:\Moving Radar\Display")


def load_upstream(multitrack: Path, producer: Path):
    """Import MultiTrack's own modules. Read-only: importing executes no fusion.

    The upstream paths are APPENDED, never put in front: ``MultiTrack/rdp.py`` has the same
    name as this display's ``rdp`` package, and putting MultiTrack first makes
    ``from rdp.packet import ...`` resolve to it. Importing the display's own modules first
    pins them in sys.modules so the shadowing cannot bite later either.
    """
    import rdp.packet            # noqa: F401  - pin the display's package before shadowing
    import visualization.style   # noqa: F401
    for p in (str(multitrack), str(producer)):
        if p not in sys.path:
            sys.path.append(p)
    # Importing normally would write .pyc files into MultiTrack/__pycache__. That directory is
    # upstream and this tool must leave no trace in it, so bytecode writing is disabled for the
    # duration of the import.
    was = sys.dont_write_bytecode
    sys.dont_write_bytecode = True
    try:
        import config
        import system_track_packet as stp
        from FusionManager import FusionManager
    finally:
        sys.dont_write_bytecode = was
    return config, stp, FusionManager


class Collector:
    """The two attributes FusionManager reaches through on its engine, and nothing else.

    FusionManager calls ``send_system_track(socket, track, address)``. Collecting the bytes
    here instead of sending them is what lets this run with no network and no second process.
    """

    def __init__(self):
        self.displaySocket = self
        self.displayAddress = ("collected", 0)
        self.sent: list[bytes] = []

    def sendto(self, data, _addr):
        self.sent.append(bytes(data))


def read_sensor_logs(multitrack: Path, config, limit: int | None = None):
    """The recorded sensor track packets, oldest first. Opened read-only."""
    size = ctypes.sizeof(config.TrackPacket)
    records = []
    for name in ("primarySensorTrackLogs.bin", "secondarySensorTrackLogs.bin"):
        path = multitrack / name
        if not path.is_file():
            print(f"  MISSING {path}")
            continue
        with path.open("rb") as fh:
            while True:
                raw = fh.read(size)
                if len(raw) != size:
                    break
                records.append(config.TrackPacket.from_buffer_copy(raw))
    records.sort(key=lambda p: p.header.timeStamp)
    return records[:limit] if limit else records


def safe_decode_auth(config_mod, p):
    if hasattr(p, "auth"):
        fn = getattr(config_mod, "decodeAuth", None)
        if callable(fn):
            return fn(p.auth)
        return "friend" if int(p.auth) == 1 else "unknown"
    return "unknown"


def safe_decode_icao(config_mod, p):
    addr = getattr(p, "icaoAddress", getattr(p, "icao", None))
    if addr is not None:
        fn = getattr(config_mod, "decodeIcao", None)
        if callable(fn):
            return fn(addr)
        v = int(addr)
        if v not in (0xFFFFFFFF, 0x000000):
            return f"0x{v:06X}"
    return None


def run_fusion(records, config, FusionManager):
    """Drive MultiTrack's own fusion with those records. Returns (collector, manager, errors).

    MultiTrack's fusion prints several lines per report; at this volume that is minutes of
    console I/O and nothing to do with what is being measured, so it is captured.
    """
    import numpy as np

    collector = Collector()
    manager = FusionManager(collector)
    Track = sys.modules["ci_ici_mulitgt"].Track
    errors: collections.Counter = collections.Counter()
    with contextlib.redirect_stdout(io.StringIO()):
        for p in records:
            t = Track(p.trackId, p.header.timeStamp)
            # exactly as FusionEngineMultiTrack.main builds it: shape (10,), not (10, 1)
            t.X = np.array(p.state)
            t.P = np.array(p.covariance).reshape(10, 10)
            t.time = p.header.timeStamp
            t.id = p.trackId
            t.sensorId = int(p.header.source)
            t.auth = safe_decode_auth(config, p)
            t.icao = safe_decode_icao(config, p)
            try:
                manager.processIncomingTrack(t)
            except Exception as exc:          # a fusion failure is a finding, not a crash
                errors[type(exc).__name__] += 1
    return collector, manager, errors


def save_packets(path: Path, packets) -> None:
    """Length-prefixed frames, so the file stays readable if the packet size ever changes."""
    with path.open("wb") as fh:
        for raw in packets:
            fh.write(len(raw).to_bytes(4, "big"))
            fh.write(raw)
    print(f"wrote {len(packets)} packet(s) to {path}")


def load_packets(path: Path) -> list[bytes]:
    out, data, i = [], path.read_bytes(), 0
    while i + 4 <= len(data):
        n = int.from_bytes(data[i:i + 4], "big")
        i += 4
        if i + n > len(data):
            break
        out.append(data[i:i + n])
        i += n
    return out


def replay(multitrack: Path, producer: Path, limit=None, save: Path | None = None) -> int:
    config, stp, FusionManager = load_upstream(multitrack, producer)
    from rdp.packet import PacketError, decode
    from visualization import style

    print(f"MultiTrack : {multitrack}")
    print(f"Producer   : {producer}")
    print(f"TrackPacket {ctypes.sizeof(config.TrackPacket)} bytes   "
          f"SystemTrackPacket {ctypes.sizeof(stp.SystemTrackPacket)} bytes\n")

    # ---- stage 1 --------------------------------------------------------------------
    records = read_sensor_logs(multitrack, config, limit)
    stage1 = collections.Counter()
    has_auth_field = records and hasattr(records[0], "auth")
    if not has_auth_field:
        print("  [NOTICE] config.TrackPacket does not contain 'auth' or 'icaoAddress' fields.")
        print("           Apply MULTITRACK_IFF_AUTH_ICAO_MANUAL_PATCH.md and regenerate sensor logs")
        print("           to enable full IFF/Auth/ICAO propagation.\n")
    for p in records:
        a_val = safe_decode_auth(config, p)
        i_val = safe_decode_icao(config, p)
        stage1[(int(p.header.source), a_val, i_val is not None)] += 1
    print(f"STAGE 1  sensor track logs -> config.TrackPacket   ({len(records)} records)")
    for (sensor, auth, has_icao), n in sorted(stage1.items()):
        print(f"           sensor {sensor}  auth={auth:<8} "
              f"icao={'yes' if has_icao else 'no':<3}  x{n}")
    friends_in = sum(n for (_s, a, _i), n in stage1.items() if a == "friend")
    print(f"           -> {friends_in} record(s) carry auth=friend\n")

    # ---- stage 2 --------------------------------------------------------------------
    collector, manager, errors = run_fusion(records, config, FusionManager)
    for kind, n in errors.items():
        print(f"           fusion raised {kind} on {n} record(s)")
    print(f"STAGE 2  FusionManager -> SystemTrack   "
          f"({len(manager.systemTrackList)} system track(s))")
    both = [st for st in manager.systemTrackList.values() if set(st.trackMap) >= {1, 2}]
    ends = collections.Counter(getattr(st, "auth", None)
                               for st in manager.systemTrackList.values())
    print(f"           final auth across system tracks: {dict(ends)}")
    print(f"           seen by BOTH sensors: {len(both)} -> "
          f"{dict(collections.Counter(getattr(s, 'auth', None) for s in both))}\n")

    # ---- stage 3 --------------------------------------------------------------------
    print(f"STAGE 3  send_system_track -> {len(collector.sent)} packet(s) -> display decoder")
    decoded, bad, last = collections.Counter(), collections.Counter(), {}
    for raw in collector.sent:
        try:
            st = decode(raw)
        except PacketError as exc:
            bad[exc.reason] += 1
            continue
        decoded[(st.auth, st.icao is not None)] += 1
        last[st.system_track_id] = st
    for (auth, has_icao), n in sorted(decoded.items()):
        print(f"           auth={auth:<8} icao={'yes' if has_icao else 'no':<3}  x{n}")
    for reason, n in bad.items():
        print(f"           REJECTED {reason} x{n}")
    friends_out = sum(n for (a, _i), n in decoded.items() if a == "friend")
    print(f"           -> {friends_out} packet(s) carry auth=friend\n")

    print("FINAL STATE PER SYSTEM TRACK (what the display would show)")
    for stid, st in sorted(last.items()):
        print(f"           ST{stid:<3} classification="
              f"{style.classification_from_auth(st.auth):<9} auth={st.auth:<8} "
              f"icao={st.icao or '-'}")

    if save is not None:
        print()
        save_packets(save, collector.sent)

    print()
    if friends_in and not friends_out:
        print(f"VERDICT  IDENTITY LOST IN FUSION: {friends_in} sensor record(s) carried "
              f"auth=friend, 0 reached the display.")
        return 1
    if not friends_in:
        print("VERDICT  No IFF in the recorded logs - nothing for the display to show.")
        return 2
    print(f"VERDICT  {friends_in} friend record(s) in, {friends_out} packet(s) out.")
    return 0


def listen(port: int | None) -> int:
    """Decode live packets from the fusion engine. Ctrl-C to stop."""
    import socket

    from rdp.packet import PACKET_SIZE, PacketError, decode

    cfg = json.loads((APP_DIR / "config" / "display_config.json").read_text(encoding="utf-8"))
    rdp = cfg.get("rdp", {})
    host, port = rdp.get("bind_host", "127.0.0.1"), port or int(rdp.get("port", 9000))
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((host, port))
    print(f"listening on {host}:{port}, expecting {PACKET_SIZE}-byte packets - Ctrl-C to stop\n")
    seen = 0
    try:
        while True:
            data, sender = sock.recvfrom(65535)
            seen += 1
            try:
                st = decode(data, sender=f"{sender[0]}:{sender[1]}")
            except PacketError as exc:
                print(f"  #{seen:<5} REJECTED {exc.reason}: {exc.detail}  ({len(data)} bytes)")
                continue
            print(f"  #{seen:<5} ST{st.system_track_id:<4} t={st.time:<12.3f} "
                  f"sensors={list(st.sensor_ids)}  auth={st.auth:<8} icao={st.icao or '-'}")
    except KeyboardInterrupt:
        print(f"\nstopped after {seen} packet(s)")
    finally:
        sock.close()
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--listen", action="store_true", help="decode live UDP instead of replaying")
    ap.add_argument("--port", type=int, default=None, help="UDP port for --listen")
    ap.add_argument("--multitrack", type=Path, default=DEFAULT_MULTITRACK)
    ap.add_argument("--producer", type=Path, default=DEFAULT_PRODUCER)
    ap.add_argument("--limit", type=int, default=None, help="only the first N sensor records")
    ap.add_argument("--save", type=Path, default=None,
                    help="write the captured packets, so a GUI test can replay the real bytes")
    args = ap.parse_args(argv)
    paths.ensure_writable_dirs()
    if args.listen:
        return listen(args.port)
    if not args.multitrack.is_dir():
        print(f"MultiTrack not found: {args.multitrack}", file=sys.stderr)
        return 2
    return replay(args.multitrack, args.producer, args.limit, args.save)


if __name__ == "__main__":
    sys.exit(main())
