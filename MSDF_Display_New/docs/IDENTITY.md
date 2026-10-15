# Identity: classification, authentication and ICAO

Where the display's identity information comes from, and the one rule it follows:

> **The display never decides identity. It receives it.**

Every classification, every authentication state and every ICAO address shown anywhere in
this application is a value the RDP SystemTrack packet carried. Nothing here is inferred from
geometry, from target numbering, from a CSV, or from a default.

---

## The chain

```
Secondary_Sensor.csv            Auth, Icao_address          (sensor 2 only: it has the interrogator)
  └─ plotCluster.processDetectionsFile       plot.auth = 'friend' | 'unknown';  plot.icao
  └─ trackManager                            track.icao = plot.icao
  └─ config.packTrack                        TrackPacket.auth / .icaoAddress       905 bytes
  └─ TrackStreamer                           -> UDP :5000
  └─ FusionEngineMultiTrack                  track.auth / track.icao off the packet
  └─ ci_ici_mulitgt.SystemTrack              carried through fusion
  └─ FusionManager.updateSystemTrack         a 'friend' from any sensor wins
  └─ system_track_packet.SystemTrackPacket   auth uint8 + icao uint32             441 bytes
  └─ UDP :9000
=================================  the display starts here  =================================
  └─ rdp/receiver.py
  └─ rdp/packet.py        decode()            -> SystemTrackState.auth / .icao
  └─ rdp/track_store.py   _view()             -> TrackState.auth / .icao / .classification
  └─ models/track_state.py  TrackState        THE canonical model - one per track, shared
       └─ 3D (view_3d_quick)   2D (tracks_2d)   PPI (aircraft_ppi)
       └─ Track Details (track_table)          Inspector (hover)   Status bar (rdp_status)
```

All views read the same `TrackState`. There is no second classification path, and no view
works anything out for itself.

## The wire fields

| Field | Type | Meaning |
|---|---|---|
| `auth` | `uint8` | `0` = unknown, `1` = friend (`rdp.packet.AUTH_UNKNOWN` / `AUTH_FRIEND`) |
| `icao` | `uint32` | 24-bit address, or `ICAO_NONE` (`0xFFFFFFFF`) when the track has none |

Mirrored in `rdp/packet.py` rather than imported: the producer's module lives outside this
project and importing it would tie the display to a path it does not own.
`tests/test_rdp.py` pins our constants against the producer's, so a change upstream fails
loudly instead of drifting.

## MultiTrack has two identity states, not four

`plotCluster.processDetectionsFile` maps the secondary sensor's `Auth` column to `'friend'`,
and **everything else to `'unknown'`**. There is no hostile determination and no neutral
determination anywhere in the tracker.

So for a live track:

| packet `auth` | classification shown | authentication shown |
|---|---|---|
| `friend` | Friendly | Authenticated (IFF) |
| `unknown` | Unknown | No IFF reply |

The display supports **exactly these two classifications and no others**. `Hostile` and
`Neutral` were removed from `visualization/style.py`: nothing upstream can produce them, and
offering options the system cannot reach invites an operator to read meaning into them.
`style.classification_from_auth` is the single place the mapping happens.

A value the display does not support - a scenario file's `Foe`, say - resolves to **Unknown**,
never to Friendly. Unknown means "not established as friendly", which is the honest reading of
a value that cannot be interpreted; the other way round would assert something nothing
determined. The raw source value is never rewritten: it stays on the model and is shown
verbatim, labelled as a Scenario Export field.

**`unknown` is the absence of a reply, not evidence of hostility.** It maps to Unknown and
it maps to Unknown.

## Why authentication is shown separately from classification

MultiTrack decides both with one field, so they can never disagree. They are still shown as
two rows because they answer different questions — *what is it* and *how do we know* — and
an operator reading "Friendly" is entitled to see that it rests on an IFF reply rather than
on an assumption. The wording is deliberately different (`Friendly` vs `Authenticated (IFF)`)
so two rows never look like two independent confirmations.

## Truth targets are not tracks

Targets from the Scenario Export (`source = "TRUTH"`) carry the scenario's own `Auth` column
as their classification, because that is what the aircraft **is** — the yardstick for judging
whether the tracker identified it correctly.

They carry `auth = ""` and `icao = None`, always. No interrogation happened, and filling those
fields from truth would manufacture an authentication the system never performed. The
inspector says so in words: *"truth is not an IFF result"*.

This matters in the current scenario, where the two disagree:

| Target | Export truth `Auth` | MultiTrack IFF |
|---|---|---|
| 3 | Friend | `friend`, `0x000BB8` |
| 4 | **Unknown** | **`friend`, `0x000FA0`** |
| 6 | **Foe** | not interrogated; the tracker cannot produce "foe" |

Keeping them apart is what lets you see that.

## Upstream notes

Things the display defends against but cannot fix. None are display bugs.

1. **Only sensor 2 carries IFF.** `plotCluster` sets `auth`/`icao` for `sensorId == 2` only, so
   a track seen by the primary radar alone is always `unknown`. Correct — the primary has no
   interrogator — but it means identity depends on the secondary holding the track.

2. **Non-friend ICAO addresses are noise.** `plotCluster` assigns `plot.icao` unconditionally,
   and in the scenario data targets 2, 5 and 6 carry a *different random* `Icao_address` on
   every detection (208 of 345 rows have no `Auth` at all). Only targets 3 and 4 have stable
   addresses. The display shows what the packet sends; an ICAO beside an Unknown track is
   therefore not evidence of anything.

3. **A zero-sensor system track transmits a fabricated ICAO.** In
   `Display/system_track_packet.py`, `from_system_track` sets `packet.auth` / `packet.icao`
   *inside* the `for slot in enumerate(sensor_ids_sorted)` loop. When `trackStateMap` is empty
   the loop never runs, both fields stay `0`, and `0` is not the `ICAO_NONE` sentinel — so an
   authenticated friend is transmitted as `unknown` with ICAO `0x000000`.

   The display treats `0x000000` as "no address" (`rdp.packet.ICAO_UNSET`) because it is not
   an assignable ICAO 24-bit address, so a fabricated identity never reaches the operator.
   **The fix belongs upstream**: those two lines need de-indenting to the loop's own level.
