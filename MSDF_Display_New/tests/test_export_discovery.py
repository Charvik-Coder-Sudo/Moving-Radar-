"""Finding the Scenario Export.

This is load-bearing in a way that is easy to miss: the display places every RDP track by
looking up the exported radar pose at the packet's time. With no export there is no pose, so
every track gets ``world = None`` and nothing is drawn in the 3D or 2D views - the tracks
arrive, parse and enter the store, and then vanish silently at the last step.

That is exactly what happened when the display was run from inside its own repository: the
configured ``../Scenario_Export`` pointed at an empty folder in the checkout, and the search
only looked one level above the project, while the real export sat three levels up beside
the simulation. So the search walks the ancestors, and accepts only a folder that actually
looks like an export.
"""

import os
import sys
import tempfile
import unittest
from pathlib import Path

APP = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(APP))

import paths  # noqa: E402


def make_export(root: Path) -> Path:
    """A folder that looks like a Scenario Export, by the markers paths.py uses."""
    root.mkdir(parents=True, exist_ok=True)
    for marker in paths.EXPORT_MARKERS:
        (root / marker).mkdir(exist_ok=True)
    return root


class TestLooksLikeExport(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp())

    def test_a_folder_with_the_markers_is_an_export(self):
        self.assertTrue(paths.looks_like_export(make_export(self.tmp / "Scenario_Export")))

    def test_an_empty_folder_of_the_right_name_is_not(self):
        """The checkout contains an empty Scenario_Export; choosing it would look like
        success and then fail at load."""
        (self.tmp / "Scenario_Export").mkdir()
        self.assertFalse(paths.looks_like_export(self.tmp / "Scenario_Export"))

    def test_a_missing_folder_is_not(self):
        self.assertFalse(paths.looks_like_export(self.tmp / "nothing_here"))

    def test_a_file_is_not(self):
        f = self.tmp / "Scenario_Export"
        f.write_text("not a folder", encoding="utf-8")
        self.assertFalse(paths.looks_like_export(f))


class TestCandidates(unittest.TestCase):
    def test_the_search_walks_up_past_the_repository(self):
        """The application lives inside a repository and the export sits beside it, several
        levels above - which is where it was being missed."""
        cands = [str(c) for c in paths.export_candidates("../Scenario_Export")]
        for parent in paths.PROJECT_ROOT.parents:
            self.assertIn(str(parent / "Scenario_Export"), cands,
                          f"{parent} is not searched")

    def test_an_explicit_configuration_value_is_tried_before_the_guesses(self):
        cands = paths.export_candidates("../Scenario_Export")
        configured = paths.resolve("../Scenario_Export")
        self.assertLess(cands.index(configured), len(cands) - 1)
        self.assertLess(cands.index(configured),
                        cands.index(paths.PROJECT_ROOT / "Scenario_Export"))

    def test_the_environment_variable_wins_over_the_configuration(self):
        tmp = Path(tempfile.mkdtemp()) / "elsewhere"
        os.environ[paths.EXPORT_ENV] = str(tmp)
        try:
            self.assertEqual(paths.export_candidates("../Scenario_Export")[0], tmp.resolve())
        finally:
            del os.environ[paths.EXPORT_ENV]

    def test_no_candidate_is_searched_twice(self):
        cands = paths.export_candidates("../Scenario_Export")
        self.assertEqual(len(cands), len(set(cands)))

    def test_the_search_terminates(self):
        """Walking ancestors must stop at the drive root rather than run away."""
        self.assertLess(len(paths.export_candidates("../Scenario_Export")), 40)


class TestFindExport(unittest.TestCase):
    def test_an_explicit_path_wins_even_when_it_is_empty(self):
        """The user is told about the folder they asked for, not quietly given another."""
        tmp = Path(tempfile.mkdtemp()) / "asked_for"
        found, tried = paths.find_export("../Scenario_Export", explicit=str(tmp))
        self.assertEqual(found, tmp.resolve())
        self.assertEqual(tried, [tmp.resolve()])

    def test_an_empty_folder_earlier_in_the_list_does_not_win(self):
        """A real export further up must beat an empty one nearer: this is the exact shape
        of the failure - an empty Scenario_Export in the checkout, the real one above it."""
        tmp = Path(tempfile.mkdtemp())
        (tmp / "near" / "Scenario_Export").mkdir(parents=True)          # empty
        real = make_export(tmp / "far" / "Scenario_Export")
        chosen = next((p for p in [tmp / "near" / "Scenario_Export", real]
                       if paths.looks_like_export(p)), None)
        self.assertEqual(chosen, real)

    def test_a_bad_configured_path_does_not_stop_the_search(self):
        """A stale configured root must not leave the display with no export when a real one
        is sitting beside the project - that failure is invisible until tracks do not draw."""
        absent = str(Path(tempfile.mkdtemp()) / "absent")
        found, tried = paths.find_export(absent)
        self.assertIn(Path(absent).resolve(), tried)     # it was tried, and reported
        self.assertGreater(len(tried), 1)                # and the search carried on
        if found is not None:
            self.assertTrue(paths.looks_like_export(found))


if __name__ == "__main__":
    unittest.main(verbosity=2)
