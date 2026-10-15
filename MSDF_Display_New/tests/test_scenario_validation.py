"""Choosing a Scenario Export folder: what is accepted, what is refused, and why.

The display can be pointed at a Scenario Export at run time, so a folder that is not one has
to be refused BEFORE anything is swapped - and the refusal has to say what is wrong with it,
because "invalid scenario" tells an operator nothing they can act on.

Every case here is built from the real export where one is present, so the fixtures are the
actual file layout rather than an invented one.
"""

import json
import os
import shutil
import sys
import tempfile
import unittest
from pathlib import Path

APP = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(APP))
os.environ.setdefault("QT_API", "pyside6")

import paths  # noqa: E402
from data_loader.scenario_export import validate  # noqa: E402

CONFIG = json.loads((APP / "config" / "display_config.json").read_text(encoding="utf-8"))
EXPORT, _tried = paths.find_export(CONFIG.get("scenario_export", {}).get("root"))
needs_export = unittest.skipUnless(EXPORT is not None, "no Scenario Export on this machine")


def reasons_text(reasons) -> str:
    return "\n".join(reasons).lower()


class TestAcceptance(unittest.TestCase):
    @needs_export
    def test_the_real_export_validates(self):
        ok, reasons = validate(EXPORT, CONFIG)
        self.assertTrue(ok, reasons)
        self.assertEqual(reasons, [])


class TestRefusal(unittest.TestCase):
    """Each refusal must name the thing that is wrong, not merely fail."""

    def test_a_path_that_is_not_a_folder(self):
        ok, reasons = validate(Path(tempfile.gettempdir()) / "definitely-not-here", CONFIG)
        self.assertFalse(ok)
        self.assertIn("not a folder", reasons_text(reasons))

    def test_an_unrelated_folder_names_the_files_it_wants(self):
        with tempfile.TemporaryDirectory() as tmp:
            (Path(tmp) / "holiday-photos").mkdir()
            ok, reasons = validate(tmp, CONFIG)
        self.assertFalse(ok)
        self.assertTrue(reasons, "a refusal with no reason is not actionable")
        self.assertIn("missing", reasons_text(reasons))

    @needs_export
    def test_a_missing_required_file_is_named(self):
        with tempfile.TemporaryDirectory() as tmp:
            dst = Path(tmp) / "export"
            shutil.copytree(EXPORT, dst)
            own = dst / CONFIG["scenario_export"]["ownship_file"]
            self.assertTrue(own.is_file(), "fixture is wrong: the ownship file should exist")
            own.unlink()
            ok, reasons = validate(dst, CONFIG)
        self.assertFalse(ok)
        self.assertIn(own.name.lower(), reasons_text(reasons))

    @needs_export
    def test_a_missing_required_column_is_named(self):
        with tempfile.TemporaryDirectory() as tmp:
            dst = Path(tmp) / "export"
            shutil.copytree(EXPORT, dst)
            own = dst / CONFIG["scenario_export"]["ownship_file"]
            lines = own.read_text(encoding="utf-8").splitlines()
            header = lines[0].split(",")
            drop = header.index("X")
            own.write_text("\n".join(",".join(c for i, c in enumerate(ln.split(",")) if i != drop)
                                     for ln in lines), encoding="utf-8")
            ok, reasons = validate(dst, CONFIG)
        self.assertFalse(ok)
        text = reasons_text(reasons)
        self.assertIn("column", text)
        self.assertIn("'x'", text)

    @needs_export
    def test_a_file_with_no_usable_rows_is_refused(self):
        with tempfile.TemporaryDirectory() as tmp:
            dst = Path(tmp) / "export"
            shutil.copytree(EXPORT, dst)
            own = dst / CONFIG["scenario_export"]["ownship_file"]
            own.write_text(own.read_text(encoding="utf-8").splitlines()[0] + "\n",
                           encoding="utf-8")        # header only
            ok, reasons = validate(dst, CONFIG)
        self.assertFalse(ok)
        self.assertIn("no usable rows", reasons_text(reasons))

    @needs_export
    def test_non_numeric_data_where_numbers_are_required_is_refused(self):
        with tempfile.TemporaryDirectory() as tmp:
            dst = Path(tmp) / "export"
            shutil.copytree(EXPORT, dst)
            own = dst / CONFIG["scenario_export"]["ownship_file"]
            lines = own.read_text(encoding="utf-8").splitlines()
            body = [",".join("not-a-number" for _ in ln.split(",")) for ln in lines[1:]]
            own.write_text("\n".join([lines[0]] + body), encoding="utf-8")
            ok, reasons = validate(dst, CONFIG)
        self.assertFalse(ok)
        self.assertIn("no usable rows", reasons_text(reasons))

    @needs_export
    def test_large_but_legitimate_values_are_not_refused(self):
        """The point is to catch malformed input, never to second-guess aerospace magnitudes:
        a target at 300 km and 20 km altitude is a normal scenario, not a bad file."""
        with tempfile.TemporaryDirectory() as tmp:
            dst = Path(tmp) / "export"
            shutil.copytree(EXPORT, dst)
            own = dst / CONFIG["scenario_export"]["ownship_file"]
            lines = own.read_text(encoding="utf-8").splitlines()
            header = lines[0].split(",")
            ix, iz = header.index("X"), header.index("Z")
            rows = []
            for ln in lines[1:]:
                cells = ln.split(",")
                if len(cells) == len(header):
                    cells[ix], cells[iz] = "300000.0", "20000.0"
                rows.append(",".join(cells))
            own.write_text("\n".join([lines[0]] + rows), encoding="utf-8")
            ok, reasons = validate(dst, CONFIG)
        self.assertTrue(ok, reasons)


class TestItDoesNotDisturbWhatIsLoaded(unittest.TestCase):
    @needs_export
    def test_validation_reads_only(self):
        """Checking a candidate folder must not touch it, or the configured one."""
        before = {p: p.stat().st_mtime_ns for p in EXPORT.rglob("*") if p.is_file()}
        validate(EXPORT, CONFIG)
        after = {p: p.stat().st_mtime_ns for p in EXPORT.rglob("*") if p.is_file()}
        self.assertEqual(before, after, "validation wrote to the export")

    def test_the_configured_root_is_not_changed_by_a_check(self):
        root_before = CONFIG.get("scenario_export", {}).get("root")
        validate(Path(tempfile.gettempdir()), CONFIG)
        self.assertEqual(CONFIG.get("scenario_export", {}).get("root"), root_before)


if __name__ == "__main__":
    unittest.main(verbosity=2)
