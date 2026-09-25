"""Pop-out / maximise for the 2D view, the Aircraft PPI and the 3D view.

2D and PPI: the SAME widget instance is moved into its own top-level window and back.
Its signal connections to the one ViewController (one UDP receiver, one track store,
one cached Scenario Export) are untouched, so a popped-out view keeps updating and there
is no second data pipeline. While a view is out, a placeholder holds its place in the
main window; closing the pop-out window (or pressing Esc in it) returns the view there.
F11 toggles fullscreen in the pop-out window.

World view (3D / 2D): maximised INSIDE the main window instead (every other panel hidden;
F11 fullscreen; Esc or the button returns). The 3D view is one half of the single world
view, and moving its VTK OpenGL widget to another window makes Qt destroy and recreate
its GL context each time; maximising in place keeps both. (Measured over six cycles of
each, private bytes: in-place maximise no growth; PPI pop-out no growth.)
"""

from __future__ import annotations

from PySide6 import QtCore, QtGui, QtWidgets


class PopOutWindow(QtWidgets.QMainWindow):
    closed = QtCore.Signal()

    def __init__(self, title: str):
        super().__init__()
        self.setWindowTitle(title)
        QtGui.QShortcut(QtGui.QKeySequence(QtCore.Qt.Key_F11), self, self.toggle_fullscreen)
        QtGui.QShortcut(QtGui.QKeySequence(QtCore.Qt.Key_Escape), self, self.close)

    def toggle_fullscreen(self):
        if self.isFullScreen():
            self.showMaximized()
        else:
            self.showFullScreen()

    def closeEvent(self, e):
        self.closed.emit()
        super().closeEvent(e)


class PopOutManager(QtCore.QObject):
    changed = QtCore.Signal(str, bool)            # view name, popped out / maximised

    def __init__(self, parent=None):
        super().__init__(parent)
        self.views: dict[str, tuple[QtWidgets.QWidget, str]] = {}
        self.out: dict[str, tuple[PopOutWindow, QtWidgets.QWidget, QtWidgets.QSplitter]] = {}
        self.focus_name: str | None = None        # view maximised inside the main window
        self._focus_hidden: list[QtWidgets.QWidget] = []
        self._focus_other: list = []
        self.main: QtWidgets.QMainWindow | None = None
        self.in_window: set[str] = set()          # views maximised in place instead of moved

    def register_in_window(self, name: str, view: QtWidgets.QWidget, title: str,
                           main: QtWidgets.QMainWindow, hide: list[QtWidgets.QWidget]):
        """This view is maximised inside ``main`` by hiding ``hide`` (never re-parented)."""
        self.views[name] = (view, title)
        self.in_window.add(name)
        self.main = main
        self._focus_other = hide

    def register(self, name: str, view: QtWidgets.QWidget, title: str):
        """``view`` must sit directly in a QSplitter of the main window."""
        self.views[name] = (view, title)

    def is_out(self, name: str) -> bool:
        return name in self.out or self.focus_name == name

    def toggle(self, name: str):
        if name in self.in_window:
            self.restore(name) if self.focus_name == name else self.pop_out(name)
        elif name in self.out:
            self.out[name][0].close()             # restores through the closed signal
        else:
            self.pop_out(name)

    def pop_out(self, name: str):
        if name in self.in_window:
            if self.focus_name != name:
                self._focus_hidden = [w for w in self._focus_other if w.isVisible()]
                for w in self._focus_hidden:
                    w.hide()
                self.focus_name = name
                self._prev_state = self.main.windowState()
                if not self.main.isFullScreen():
                    self.main.showMaximized()
                self.changed.emit(name, True)
            return
        if name in self.out:
            win = self.out[name][0]
            win.raise_()
            win.activateWindow()
            return
        view, title = self.views[name]
        splitter = view.parentWidget()
        if not isinstance(splitter, QtWidgets.QSplitter):
            return
        sizes = splitter.sizes()
        placeholder = QtWidgets.QLabel(f"{title} is open in its own window.\n"
                                       "Close that window (or press Esc in it) to bring it back here.",
                                       objectName="PopPlaceholder", alignment=QtCore.Qt.AlignCenter)
        splitter.replaceWidget(splitter.indexOf(view), placeholder)
        splitter.setSizes(sizes)
        win = PopOutWindow(f"MSDF Display — {title}")
        win.setCentralWidget(view)
        view.show()
        win.closed.connect(lambda n=name: self.restore(n))
        self.out[name] = (win, placeholder, splitter)
        win.showMaximized()
        self.changed.emit(name, True)

    def restore(self, name: str):
        if name in self.in_window:
            if self.focus_name == name:
                for w in self._focus_hidden:
                    w.show()
                self._focus_hidden = []
                self.focus_name = None
                prev = getattr(self, "_prev_state", QtCore.Qt.WindowMaximized)
                if prev & QtCore.Qt.WindowMaximized:
                    self.main.showMaximized()
                elif not prev & QtCore.Qt.WindowFullScreen:
                    self.main.showNormal()              # back to the size it had before
                self.changed.emit(name, False)
            return
        entry = self.out.pop(name, None)
        if entry is None:
            return
        win, placeholder, splitter = entry
        view = win.takeCentralWidget()
        sizes = splitter.sizes()
        splitter.replaceWidget(splitter.indexOf(placeholder), view)
        splitter.setSizes(sizes)
        view.show()
        placeholder.deleteLater()
        win.deleteLater()
        self.changed.emit(name, False)

    def restore_all(self):
        for name in list(self.out):
            self.out[name][0].close()
        if self.focus_name is not None:
            self.restore(self.focus_name)
