"""Dark tactical-display theme."""

QSS = """
QWidget { background: #0b1220; color: #cbd5e1; font-family: "Segoe UI"; font-size: 9pt; }
QMainWindow, #Root { background: #070d18; }
QMenuBar { background: #070d18; color: #94a3b8; }
QMenuBar::item:selected { background: #1e293b; }
QMenu { background: #0f172a; border: 1px solid #334155; }
QMenu::item:selected { background: #1e3a5f; }
QStatusBar { background: #070d18; color: #94a3b8; }

#Header { background: #0a1426; border-bottom: 1px solid #1e3a5f; }
#AppTitle { color: #e2e8f0; font-size: 14pt; font-weight: 600; letter-spacing: 1px; background: transparent; }
#ModeButton { background: #111c30; border: 1px solid #28405f; padding: 5px 18px; color: #94a3b8;
              font-weight: 600; }
#ModeButton:checked { background: #0e4a6e; color: #e0f2fe; border-color: #38bdf8; }
#StatePill { padding: 4px 12px; border-radius: 10px; font-weight: 700; background: #1e293b; }
#StatePill[state="online"] { color: #4ade80; background: #0f2a1c; border: 1px solid #166534; }
#StatePill[state="paused"] { color: #fbbf24; background: #2a220f; border: 1px solid #854d0e; }
#StatePill[state="nodata"] { color: #f87171; background: #2a1111; border: 1px solid #7f1d1d; }

#NavPanel { background: #0a1322; border-right: 1px solid #1e293b; }
#NavButton { text-align: left; padding: 6px 10px; background: #0f1a2e; border: 1px solid #1e293b;
             color: #cbd5e1; }
#NavButton:checked { background: #0e3a58; border-color: #38bdf8; color: #f0f9ff; }
#NavButton:hover { border-color: #475569; }
#Sep { color: #1e293b; }

#ViewToolbar { background: #0d172a; border-bottom: 1px solid #1e293b; }
#ViewTitle { color: #7dd3fc; font-weight: 700; letter-spacing: 1px; background: transparent; }
#ViewInfo { color: #64748b; background: transparent; }
#PanelTitle { color: #7dd3fc; font-weight: 700; letter-spacing: 1px; padding: 4px 0; }
#SectionLabel { color: #64748b; font-size: 8pt; font-weight: 700; padding-top: 4px; }
#FieldName { color: #94a3b8; }
#FieldValue { color: #e2e8f0; font-family: Consolas; qproperty-alignment: 'AlignRight|AlignVCenter'; }
#FieldUnit { color: #64748b; }
#Hint { color: #64748b; font-size: 8pt; }
#Mono { font-family: Consolas; font-size: 8pt; color: #94a3b8; }
#MonoLive { font-family: Consolas; font-size: 8pt; color: #fbbf24; }
#Card { background: #0f1a2e; border: 1px solid #1e293b; border-radius: 4px; }
#Panel { background: #0a1322; }
#PPIFooter { background: #0a1322; color: #64748b; font-size: 8pt; padding: 4px 8px; }
#PlaybackBar { background: #0d172a; border-top: 1px solid #1e293b; }
#TimeLabel { font-family: Consolas; color: #e2e8f0; padding-left: 8px; }
#PlayButton { min-width: 80px; font-weight: 700; color: #e0f2fe; }
#RdpBar { background: #081120; border-bottom: 1px solid #1e293b; }
#RdpTitle { color: #7dd3fc; font-weight: 700; letter-spacing: 1px; background: transparent; }
#RdpCaption { color: #64748b; font-size: 8pt; background: transparent; }
#RdpValue { color: #e2e8f0; font-family: Consolas; font-size: 8pt; background: transparent; }
#RdpValue[state="ok"] { color: #4ade80; }
#RdpValue[state="error"] { color: #f87171; }
#RdpValue[state="warn"] { color: #fbbf24; }
#PopPlaceholder { color: #64748b; background: #070d18; font-size: 10pt; }
#ErrorBanner { background: #3b0d0d; color: #fecaca; border-bottom: 1px solid #7f1d1d; padding: 6px 14px; font-family: Consolas; font-size: 8pt; }
#Legend { background: #0a1322; border-top: 1px solid #1e293b; }
#LegendHead { color: #64748b; font-size: 7pt; font-weight: 700; letter-spacing: 1px; background: transparent; }
#LegendText { color: #cbd5e1; font-size: 8pt; background: transparent; }

QToolButton, QPushButton { background: #111c30; border: 1px solid #28405f; padding: 3px 6px;
                           border-radius: 3px; }
QToolButton:hover, QPushButton:hover { border-color: #38bdf8; }
QToolButton:checked { background: #0e4a6e; border-color: #38bdf8; color: #f0f9ff; }
QComboBox, QDoubleSpinBox { background: #111c30; border: 1px solid #28405f; padding: 2px 6px; }
QCheckBox { spacing: 8px; padding: 2px 0; }
QCheckBox::indicator { width: 14px; height: 14px; }
QSlider::groove:horizontal { height: 4px; background: #1e293b; }
QSlider::handle:horizontal { background: #38bdf8; width: 12px; margin: -5px 0; border-radius: 6px; }
QSlider::sub-page:horizontal { background: #0e7490; }
QSplitter::handle { background: #1e293b; }
QTableView { background: #0a1220; alternate-background-color: #0e182a; gridline-color: #1e293b;
             selection-background-color: #1e3a5f; selection-color: #f0f9ff;
             font-family: Consolas; font-size: 8pt; }
QHeaderView::section { background: #111c30; color: #94a3b8; border: none;
                       border-right: 1px solid #1e293b; padding: 3px 6px; font-weight: 600; }
QScrollArea { background: transparent; }
QPlainTextEdit { background: #050b14; }
QTabBar::tab { background: #111c30; padding: 4px 12px; }
QTabBar::tab:selected { background: #0e3a58; }
QToolTip { background: #0b1220; color: #e2e8f0; border: 1px solid #334155; padding: 4px; }
"""
