# Stability: what the display guarantees, and how it is proved

The display has to run for the length of a scenario without freezing, drifting or growing.
This is what holds that up, and the tools that measure it.

---

## The three faults that caused the symptoms

### 1. The window stopped responding — label stacking

`world_style.stacked_offsets` places a track's label and steps it down when it would overlap
one already placed. It used to step to `other_label_y - LABEL_LINE_PX`, which moves the label
by however far *that neighbour* happens to be. For tracks a fraction of a pixel apart, that is
a fraction of a pixel per pass, so the number of passes is unbounded — **inside the render
path**. With 31 real system tracks clustered on screen a single `processEvents` ran for over
88 seconds, which Windows reports as "Not Responding".

It now steps a whole line per pass, with a hard cap (`MAX_LABEL_STACK`). Work per label is
bounded by how far it can be pushed, and the cap bounds it absolutely. Worst measured case is
2.5 ms for 60 labels piled on one point.

Caught by `faulthandler`, not by reading code. `tests/test_track_visuals.py` now runs the
pathological inputs behind a watchdog so it cannot come back.

### 2. Simulation time could be dragged backwards — the packet anchor

The world clock anchors on `newest_time` from the track store. That was assigned from the
**last packet accepted**, not the highest time seen. MultiTrack sends one packet per sensor
report and interleaves system tracks, so arrivals are not ordered in time even when no packet
is stale for its own track — a packet for ST5 at t=100 is legitimately followed by one for ST9
at t=92. `newest_time` therefore walked backwards, and a drop larger than `restart_jump_s`
reads as "the RDP restarted", which pulls the displayed time back with it.

`newest_time` is now a running maximum. `last_packet_time` still answers the different
question — what arrived most recently — and the status bar uses that.

### 3. The store grew for ever — no record retirement

`TrackStore.records` was never pruned. Hidden tracks kept their trail buffers indefinitely,
and fusion mints a fresh system-track id whenever a report fails to correlate (31 ids for 5
targets in this scenario), so a long run accumulated dead tracks that nothing would ever draw.

Tracks are now forgotten once they have been silent for `forget_timeout_s` (default 120 s,
well past the point where they stop being drawn), with a hard `max_records` ceiling for the
case where tracks arrive faster than they age out. A track that reappears after that long is
treated as new, which is the correct reading.

---

## Terrain is built once

`TERRAIN_LODS[0]` reaches 45 km in each direction. The terrain is centred on the **middle of
the ownship's own track** — not on the scene bounds, which distant targets pull about, and not
on where the aircraft happens to be at startup. The ownship then stays near the middle for the
whole run: measured maximum excursion for this scenario is 5.6 km against a 27 km rebuild
threshold, so **the terrain is never rebuilt during playback**.

It previously re-centred every 12 km of travel, which put a 1.2 s stall on the GUI thread
roughly every 80 s of flight. The test is now coverage, not distance: a scenario that really
does wander out of the tile still gets ground under it.

---

## What runs where

```
UDP socket        QThread (rdp/receiver.py)      never touches the GUI
  → decode        QThread                        rdp/packet.py
  → TrackStore    QThread                        bounded records, bounded trails
  → views+stats   QThread, ~10 Hz                snapshot handed over by signal
      ↓ Qt queued signal
  ViewController  GUI thread                     one QTimer, one clock
  → renderers     GUI thread                     update persistent scene objects
```

The Scenario Export loads in a worker and the window stays live while it does. There are
exactly **two** persistent `QTimer`s in the application — the world clock in
`ViewController` and the audio poll in `widgets/aircraft_audio.py` — both created once with a
parent. Qt restarts a running timer rather than creating a second one, so repeated Play cannot
multiply them.

---

## Measuring it yourself

| Tool | Answers |
|---|---|
| `tests/profile_display.py` | where frame time goes; whether any file is opened during frames |
| `tests/soak_display.py SECONDS` | time monotonicity, frame-time drift, memory, track growth, timer/thread growth, clean shutdown |
| `verify_multitrack_interface.py` | how much identity survives each upstream stage |
| `verify_multitrack_interface.py --listen` | decode live packets off the wire |
| `tests/live_iff_check.py` | real producer bytes into a real window |
| `tests/gui_smoke.py` | the window's own behaviour end to end |

Measured on this machine (Intel integrated graphics, no discrete GPU), 21 live tracks, real
Scenario Export:

```
FRAME TIME over 240 frames (ms)
  median   16.6    mean  16.7
  p90      17.9    p99   25.7
  worst    40.8
  frames over the 30 Hz budget (33 ms): 1 of 240
FILES OPENED DURING FRAMES: none
```

---

## What the display deliberately does NOT do

- **It does not latch identity.** If a packet says `unknown`, the display shows Unknown, even
  if that track was authenticated a moment ago. The packet is the record. What it does instead
  is *count* the downgrade and log it once per track, so an upstream identity reset is visible
  rather than looking like a display glitch (`docs/IDENTITY.md`).
- **It does not correct upstream data.** Transposed velocity columns, random ICAO addresses on
  unauthenticated tracks, and identity resets in fusion are all reported, never patched.
- **It does not use `processEvents` to stay responsive.** Blocking work is bounded or moved,
  not worked around.
