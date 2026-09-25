"""UDP receiver: real datagrams over loopback, worker thread -> GUI-thread store."""

import os
import socket
import sys
import time
import unittest
from pathlib import Path

APP = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(APP))
os.environ.setdefault("QT_API", "pyside6")

from PySide6 import QtCore  # noqa: E402

from rdp.packet import encode_for_test  # noqa: E402
from rdp.receiver import RdpClient      # noqa: E402

APP_Q = QtCore.QCoreApplication.instance() or QtCore.QCoreApplication([])
X = (1000.0, 11.0, 0.5, 2000.0, 22.0, 0.6, 0.02, 300.0, 3.0, 0.7)


def wait_for(cond, timeout=5.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        APP_Q.processEvents(QtCore.QEventLoop.AllEvents, 50)
        if cond():
            return True
        time.sleep(0.01)
    return False


def client(port=0):
    return RdpClient({"enabled": True, "bind_host": "127.0.0.1", "port": port}, {1: "PRIMARY", 2: "SECONDARY"})


class TestReceiver(unittest.TestCase):
    def setUp(self):
        self.rdp = client()
        self.updates = []
        self.update_threads = set()
        self.rdp.updated.connect(lambda v, s: (self.updates.append((v, s)),
                                               self.update_threads.add(QtCore.QThread.currentThread())))
        self.rdp.start()
        self.assertTrue(wait_for(lambda: self.rdp.connection == "listening"), self.rdp.connection_detail)
        self.port = self.rdp.bound_port
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def tearDown(self):
        self.tx.close()
        self.rdp.stop()

    def send(self, data):
        self.tx.sendto(data, ("127.0.0.1", self.port))

    def test_valid_and_invalid_packets(self):
        for i in range(20):
            self.send(encode_for_test(1 + i % 4, 10.0 + i, 1, X, [1, 2], [X, X]))
        self.send(b"\x00" * 100)                                   # wrong length
        self.send(encode_for_test(9, float("nan"), 1, X, [1], [X]))  # invalid numeric
        self.assertTrue(wait_for(lambda: self.rdp.stats.get("packets", 0) >= 22))
        st = self.rdp.stats
        self.assertEqual(st["invalid"], 2)
        self.assertEqual(st["invalid_by_reason"], {"length": 1, "numeric": 1})
        self.assertEqual(st["active_system_tracks"], 4)
        self.assertEqual(st["active_sensor_tracks"], 8)
        self.assertTrue(st["receiving"])
        self.assertEqual(st["connection"], "listening")
        self.assertEqual(len(self.rdp.views), 12)                  # 4 fused + 8 sensor
        self.assertEqual(self.update_threads, {APP_Q.thread()})    # views published on the GUI thread only

    def test_burst_is_batched(self):
        for i in range(2000):
            self.send(encode_for_test(i % 50, float(i), 1, X, [1], [X]))
        self.assertTrue(wait_for(lambda: self.rdp.stats.get("packets", 0) >= 1500, timeout=10.0),
                        self.rdp.stats.get("packets"))
        self.assertEqual(self.rdp.stats["invalid"], 0)
        self.assertLessEqual(len(self.updates), 200)               # 10 Hz publishing, not one per packet


class TestSocketUnavailable(unittest.TestCase):
    def test_port_in_use(self):
        blocker = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        blocker.bind(("127.0.0.1", 0))
        port = blocker.getsockname()[1]
        rdp = client(port)
        try:
            rdp.start()
            self.assertTrue(wait_for(lambda: rdp.connection == "unavailable"))
            self.assertIn(str(port), rdp.connection_detail)
            blocker.close()                                        # freed: the worker retries and binds
            self.assertTrue(wait_for(lambda: rdp.connection == "listening", timeout=6.0))
        finally:
            rdp.stop()
            blocker.close()

    def test_disabled(self):
        rdp = RdpClient({"enabled": False}, {})
        rdp.start()
        self.assertEqual(rdp.connection, "disabled")
        self.assertIsNone(rdp.bound_port)
        rdp.stop()


if __name__ == "__main__":
    unittest.main()
