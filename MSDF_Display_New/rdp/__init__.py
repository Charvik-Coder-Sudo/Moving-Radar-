"""Read-only receiver for the RDP SystemTrack UDP stream.

    UDP socket -> receiver (worker thread) -> decoder -> SystemTrackState -> TrackStore -> views

Nothing in this package sends to, or modifies, the RDP in D:\\Moving Radar\\MultiTrack.
"""
