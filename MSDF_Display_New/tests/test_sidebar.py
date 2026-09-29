"""The sidebar's grouped layer switches, and the track table's column sets.

Two things must hold whatever else changes. Every display layer has to be reachable from the
panel - a layer that exists in the controller but appears in no group is invisible to the
operator. And hiding table columns must hide columns only: the rows the table lists are
decided by the layer switches, never by which columns are on show.
"""

import os
import sys
import unittest
from pathlib import Path

APP = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(APP))
os.environ.setdefault("QT_API", "pyside6")
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6 import QtCore, QtWidgets  # noqa: E402

from widgets.navigation_panel import GROUPS, LAYERS, Section  # noqa: E402
from widgets.track_table import (COLUMNS, COL_CLASS, COL_SOURCE, COLUMN_SETS,  # noqa: E402
                                 ESSENTIALS, column_indices)

QAPP = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


class TestGroupsCoverEveryLayer(unittest.TestCase):
    def test_every_layer_is_in_exactly_one_group(self):
        grouped = [k for _title, keys, _open in GROUPS for k in keys]
        self.assertEqual(sorted(grouped), sorted(k for k, _t, _tip in LAYERS))
        self.assertEqual(len(grouped), len(set(grouped)), "a layer appears in two groups")

    def test_no_group_names_a_layer_that_does_not_exist(self):
        known = {k for k, _t, _tip in LAYERS}
        for title, keys, _open in GROUPS:
            for k in keys:
                self.assertIn(k, known, f"{title} names an unknown layer {k}")

    def test_no_empty_groups(self):
        for title, keys, _open in GROUPS:
            self.assertTrue(keys, f"{title} has no switches")


class TestSection(unittest.TestCase):
    def section(self, states):
        boxes = [QtWidgets.QCheckBox(f"layer {i}") for i in range(len(states))]
        for cb, on in zip(boxes, states):
            cb.setChecked(on)
        return Section("GROUP", boxes, True), boxes

    def test_header_reports_all_on(self):
        sec, _ = self.section([True, True, True])
        self.assertEqual(sec.switch.checkState(), QtCore.Qt.Checked)

    def test_header_reports_all_off(self):
        sec, _ = self.section([False, False])
        self.assertEqual(sec.switch.checkState(), QtCore.Qt.Unchecked)

    def test_header_reports_a_mixed_group(self):
        sec, _ = self.section([True, False, False])
        self.assertEqual(sec.switch.checkState(), QtCore.Qt.PartiallyChecked)

    def test_switching_a_mixed_group_turns_all_on(self):
        sec, boxes = self.section([True, False, False])
        sec._switch_group()
        self.assertTrue(all(cb.isChecked() for cb in boxes))
        self.assertEqual(sec.switch.checkState(), QtCore.Qt.Checked)

    def test_switching_a_full_group_turns_all_off(self):
        sec, boxes = self.section([True, True])
        sec._switch_group()
        self.assertFalse(any(cb.isChecked() for cb in boxes))

    def test_the_header_follows_the_rows(self):
        sec, boxes = self.section([True, True])
        boxes[0].setChecked(False)
        self.assertEqual(sec.switch.checkState(), QtCore.Qt.PartiallyChecked)

    def test_folding_hides_the_rows_but_not_the_group_switch(self):
        sec, _ = self.section([True, True])
        sec.fold.setChecked(False)
        self.assertFalse(sec.body.isVisibleTo(sec))
        self.assertTrue(sec.switch.isVisibleTo(sec))
        sec.fold.setChecked(True)
        self.assertTrue(sec.body.isVisibleTo(sec))

    def test_a_title_with_an_ampersand_is_shown_in_full(self):
        """'&' in a button label is a mnemonic: LABELS & VECTORS would read as LABELS VECTORS
        with an underlined V."""
        sec = Section("LABELS & VECTORS", [QtWidgets.QCheckBox("a")], True)
        self.assertIn("LABELS && VECTORS", sec.fold.text())
        sec.fold.setChecked(False)
        self.assertIn("LABELS && VECTORS", sec.fold.text())

    def test_the_fold_arrow_says_which_way_it_goes(self):
        sec = Section("GROUP", [QtWidgets.QCheckBox("a")], True)
        self.assertTrue(sec.fold.text().startswith("▾"))
        sec.fold.setChecked(False)
        self.assertTrue(sec.fold.text().startswith("▸"))

    def test_folding_changes_no_layer(self):
        sec, boxes = self.section([True, False])
        before = [cb.isChecked() for cb in boxes]
        sec.fold.setChecked(False)
        sec.fold.setChecked(True)
        self.assertEqual([cb.isChecked() for cb in boxes], before)


class TestColumnSets(unittest.TestCase):
    def test_essentials_name_real_columns(self):
        headers = {c[0] for c in COLUMNS}
        for name in ESSENTIALS:
            self.assertIn(name, headers)

    def test_essentials_is_a_subset_and_actually_smaller(self):
        self.assertLess(len(ESSENTIALS), len(COLUMNS))
        self.assertEqual(len(set(ESSENTIALS)), len(ESSENTIALS))

    def test_the_indicator_columns_are_always_shown(self):
        """The classification swatch and the sensor / fused icon live in these two columns:
        hiding them would take the symbology out of the table."""
        idx = column_indices(ESSENTIALS)
        self.assertIn(COL_CLASS, idx)
        self.assertIn(COL_SOURCE, idx)

    def test_identity_and_currency_are_kept(self):
        for name in ("Track ID", "Status*", "Age* (s)", "Sensors / system"):
            self.assertIn(name, ESSENTIALS)

    def test_all_columns_means_all_of_them(self):
        self.assertEqual(column_indices(None), list(range(len(COLUMNS))))

    def test_the_offered_sets_are_the_two_we_document(self):
        self.assertEqual([label for label, _names in COLUMN_SETS], ["Essentials", "All columns"])
        self.assertIsNone(COLUMN_SETS[1][1])

    def test_indices_are_in_table_order(self):
        # the table cannot reorder columns by hiding them, so the indices must come out sorted
        idx = column_indices(ESSENTIALS)
        self.assertEqual(idx, sorted(idx))


if __name__ == "__main__":
    unittest.main(verbosity=2)
