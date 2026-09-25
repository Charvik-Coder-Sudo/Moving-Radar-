# RDP integration (SystemTrack over UDP)

How MSDF Display receives and interprets the SystemTrack stream produced by the RDP in
`D:\Moving Radar\MultiTrack`.

> ## The RDP is READ-ONLY
>
> `D:\Moving Radar\MultiTrack` is never modified, never imported at runtime and never written
> to — not even cosmetically. The display opens a **receive-only** UDP socket and decodes what
> arrives. It sends nothing back, it does not acknowledge, it does not request retransmission,
> and it never changes the packet format or the fusion behaviour. Everything below is an
> observation of the producer, not a requirement placed on it.

---

## 1. Chain

```
Moving_Radar.ipynb (cell 27)     ──► MultiTrack\Primary_Sensor.csv, Secondary_Sensor.csv
MultiTrack\TrackStreamer.py      ──► per-sensor trackers ──► *SensorTrackLogs.bin
                                  ──► UDP 127.0.0.1:5000  (sensor tracks, to the fusion engine)
MultiTrack\FusionEngineMultiTrack.py + FusionManager.py
                                  ──► SystemTrack packets ──► UDP 127.0.0.1:9000 (one destination)
MSDF_Display_New                  ──► one receiver thread ──► TrackStore ──► 3D / 2D / PPI / table
```

## 2. Endpoint

| | |
|---|---|
| Transport | UDP, datagram per packet, no framing, no handshake |
| Address | `rdp.bind_host` : `rdp.port` — default **127.0.0.1:9000** (`config/display_config.json`, or `main.py --rdp-port N`) |
| Direction | receive only |
| Sender | `FusionEngineMultiTrack.py`, which sends to exactly one (host, port) |

The Global Display (`D:\Moving Radar\Display`) listens on the same port by default, and the
fusion engine has a single destination, so **run one of the two on port 9000 at a time**. Giving
this display `--rdp-port` only helps if something is actually sending to that port.

`--no-rdp` starts the world views with no receiver at all (live state `RDP OFF`).

## 3. Packet

Producer: `D:\MSDF Phase 1\RDP\Display\system_track_packet.py`, class `SystemTrackPacket`
(`D:\Moving Radar\Display\system_track_packet.py` is a byte-identical copy). The decoder in
`rdp/packet.py` reproduces it exactly: **436 bytes, network byte order**, `struct` format
`!idi10di4i40d`.

| Field | Type | Meaning |
|---|---|---|
| `systemTrackId` | int32 | the fusion engine's system-track counter |
| `time` | double | seconds — the time of the sensor track that produced this update |
| `latestSensor` | int32 | `sensorId` of the latest contributing sensor |
| `Xfused` | 10 × double | fused state `[x, vx, ax, y, vy, ay, w, z, vz, az]` |
| `numSensors` | int32 | populated sensor slots, 0..4 (`MAX_SENSORS`) |
| `sensorIds` | 4 × int32 | 1 = primary, 2 = secondary |
| `sensorStates` | 4 × 10 × double | per-sensor state, same element order as `Xfused` |

**Not in the packet**, and therefore never shown as known: target ID, classification, IFF,
covariance, track status or deletion. The Track Details table shows `—` and `Unknown` for these.

## 4. Frame of the states

MultiTrack builds each state from the radar's own range / azimuth / elevation
(`imm3dc.Plot.calculateCartesian`): `x = r·cos(el)·sin(az)`, `y = r·cos(el)·cos(az)`,
`z = r·sin(el)`. With the project's measurement definitions (`Measurements_Geometry`:
`az = atan2(L_R, L_F)`, `el = atan2(−L_D, hypot(L_F, L_R))`) that is the radar FRD line of sight
relabelled:

```
x = L_R        y = L_F        z = −L_D        ⇒        L_FRD = (y, x, −z)
```

The display places it with the project's existing rotations only
(`processing/track_projection.py`):

```
aircraft FRD (PPI)   P_B = mount_xyz_frd + R_BR · L_FRD          no ownship pose needed
world ENU (3D, 2D)   P_W = P_WR(t) + R_WR(t) · L_FRD             exported radar pose at the
                                                                  packet time
```

Sensor tracks use their own sensor. The fused state combines radars whose mounts differ, so it
has no single radar frame: it is placed in `latestSensor`'s frame and its hover text says so.
Velocities are shown as sent (relative to the moving radar) and are never transformed into world
velocities; the PPI only rotates them onto the aircraft axes for drawing.

Full derivation and the validation tests: [COORDINATE_AND_ATTITUDE.md](COORDINATE_AND_ATTITUDE.md)
§5–§6.

## 5. Time

The packet `time` is **seconds** on the same timeline as the Scenario Export's `Time` (ms):
TrackStreamer replays the sensor CSVs in real time. While packets arrive, the display's world
clock follows the newest packet time, at most `live.max_lead_s` (2 s) ahead of it, and the
ownship / radar pose / beam / targets are looked up at that time. A packet time outside the
export's span is reported in the status line ("export and simulation differ") rather than
clamped silently. A backwards jump larger than `live.restart_jump_s` (5 s) is treated as an RDP
restart.

## 6. Threading

| Thread | Does | Never does |
|---|---|---|
| `UdpReceiverWorker` (QThread, `rdp/receiver.py`) | blocking `recvfrom` with a 50 ms timeout, then a non-blocking drain of up to 500 queued datagrams; decodes each one; emits `batch_ready(list)` | touch a widget, the store or the controller |
| GUI thread (`RdpClient`) | ingests batches into `TrackStore`, re-evaluates track lifetime, publishes `updated(views, stats)` on a 100 ms timer | block on the socket |

Nothing is shared for writing, so no locks are needed: decoded packets are handed over through a
queued signal and the worker forgets them. Because publication is capped at 10 Hz, a faster
stream costs no extra GUI work. Placement into world / aircraft frames happens once per
publication, not per rendered frame.

## 7. Rejected packets

`rdp/packet.py` refuses rather than guesses. A packet is rejected, counted by reason and never
drawn when:

| Reason | Condition |
|---|---|
| `length` | the datagram is not 436 bytes |
| `sensor_count` | `numSensors` outside 0..4 |
| `numeric` | `time`, `Xfused` or a sensor state contains a non-finite value |
| `decode_exception` | anything else raised while decoding (the receiver thread never dies) |
| `stale_packet` | the packet's time is older than the ACTIVE track it would update |

Counts and reasons appear in the status line ("Packets 1 234 (3 rejected: length 3)"), and the
last 500 rejects are kept for inspection. Unknown `sensorId`s are counted too and shown as
`SENSOR-<id>` rather than being dropped.

## 8. Live state

Derived from actual packet timing (`ViewController._update_live_state`):

| State | Condition |
|---|---|
| `WAITING FOR DATA` | receiver bound, no packet yet |
| `LIVE` | a valid packet within `rdp.sender_silent_timeout_s` (5 s) |
| `NO DATA / STALE` | packets were received, none for longer than that |
| `UDP UNAVAILABLE` | the port cannot be bound (retried every 2 s, with the OS error shown) |
| `RDP OFF` | started with `--no-rdp` or `rdp.enabled = false` |

## 9. Track lifetime

The packet carries no deletion, so the display decides what is still worth drawing:

| Status | Condition | Drawn |
|---|---|---|
| ACTIVE | updated within `rdp.stale_timeout_s` (3 s wall) **and** not more than `rdp.stream_stale_timeout_s` (5 s packet time) behind the newest packet | fully |
| STALE | otherwise, until `rdp.hide_timeout_s` (10 s) without an update | faded |
| HIDDEN | no update for longer than that | no |

The RDP's own track lifecycle is not second-guessed — this only governs rendering. An earlier
packet time for a track that is already stale or hidden is read as an RDP restart: the track
starts afresh and the restart is counted.

## 10. Identifiers and association

Associations come **only** from the same packet: a fused track and the sensor states carried in
its `sensorIds` / `sensorStates`. Nothing else is inferred. Labels are therefore:

```
FUSED-<systemTrackId>              the fused system track
S<sensorId>-ST<systemTrackId>      a sensor track (the packet has no per-sensor track id)
T<TgtId>                           a Scenario Export truth target (not an RDP object)
```

The display does not match RDP tracks to truth targets: the packet carries no target id, and
inventing the correspondence would be fabricating data.

## 11. Observations about the RDP (reported, not changed)

* Plot-to-track association uses the ground-truth `Target ID` column
  (`trackManager.correlate_helper`); sensor track IDs equal target IDs, but the SystemTrack
  packet does not carry them.
* `Xfused` is set when a system track is created and updated only when both sensors contribute;
  for single-sensor system tracks the fused position does not move.
* Primary and secondary states are fused as if they shared a frame, while the radar mounts differ
  (primary ypr −5/−5/−5°, secondary +5/+5/+5°). This produces extra system tracks — 39 for 4
  targets in the recorded run.
* `TrackStreamer.main()` passes scan times 1.5 s / 1.0 s; the sensors scan in 0.9 s / 1.8 s.
* `TrackStreamer` **appends** to `primarySensorTrackLogs.bin` / `secondarySensorTrackLogs.bin`;
  delete them before a run or they grow across runs.

These are observations for the RDP's owners. The display compensates for none of them — it shows
what arrives.

## 12. Troubleshooting

| Symptom | Likely cause |
|---|---|
| `UDP UNAVAILABLE` | another program holds the port — usually the Global Display or an earlier instance |
| Stays at `WAITING FOR DATA` | the fusion engine is not running, or it sends to a different destination |
| `LIVE` but no tracks in 3D/2D | packet time is far from the displayed time (world views use a ±`live.world_track_window_s` window) — the PPI and table still show them; or the packet time is outside the export span (status line says so) |
| Tracks in the PPI but not in the world views | no exported radar pose at that packet time — the hover text says "world position unknown" |
| Many rejected packets | a sender with a different build of the packet layout |
| Fused track does not move | single-sensor system track (§11) |

## 13. Tests

* `tests/test_rdp.py` — decoding against the producer's layout, every rejection reason.
* `tests/test_track_store.py` — lifetime model, stale packets, restart detection, statistics.
* `tests/test_receiver.py` — the socket thread, batching, bind failure and retry.
* `tests/test_track_projection.py` — the round trip of §4 to 1e-6 m, and placement without a pose.
* `tests/test_live.py` — the live clock, following, lead cap, replay and Go Live.

Run them with `"D:\Moving Radar\.venv\Scripts\python.exe" -m unittest discover -s tests -p "test_*.py" -t .`
— none of them needs the RDP to be running, and none of them writes to `MultiTrack`.
