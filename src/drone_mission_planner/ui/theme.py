from __future__ import annotations

APP_STYLESHEET = """
QWidget {
    color: #dfe8f5;
    background: #111722;
    font-family: "Segoe UI", "Noto Sans";
    font-size: 10pt;
}
QMainWindow, QDialog { background: #0b1018; }
QMenuBar { background: #111722; border-bottom: 1px solid #273246; padding: 3px; }
QMenuBar::item { padding: 6px 10px; border-radius: 5px; }
QMenuBar::item:selected { background: #263248; }
QMenu { background: #17202e; border: 1px solid #34425a; padding: 6px; }
QMenu::item { padding: 7px 26px 7px 12px; border-radius: 4px; }
QMenu::item:selected { background: #2b65d9; }
QToolBar {
    background: #121a27;
    border: none;
    border-bottom: 1px solid #273246;
    spacing: 6px;
    padding: 7px 10px;
}
QToolButton { border: 1px solid transparent; border-radius: 7px; padding: 7px 9px; }
QToolButton:hover { background: #202c40; border-color: #344866; }
QToolButton:checked { background: #1e4f9f; border-color: #4d8df7; color: white; }
QDockWidget { color: #e9f1ff; titlebar-close-icon: none; titlebar-normal-icon: none; }
QDockWidget::title { background: #151e2c; padding: 9px 12px; border-bottom: 1px solid #2a3549; }
QTreeWidget, QTextEdit, QPlainTextEdit, QTableWidget, QListWidget {
    background: #101722;
    border: none;
    alternate-background-color: #141e2c;
    selection-background-color: #1e4f9f;
    outline: 0;
}
QTreeWidget::item { height: 29px; padding-left: 3px; }
QTreeWidget::item:hover { background: #1b2738; }
QHeaderView::section { background: #172132; border: none; border-bottom: 1px solid #34425a; padding: 7px; }
QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox {
    background: #0d1420;
    border: 1px solid #34425a;
    border-radius: 6px;
    padding: 6px 8px;
    min-height: 18px;
}
QLineEdit:focus, QSpinBox:focus, QDoubleSpinBox:focus, QComboBox:focus { border-color: #4d8df7; }
QPushButton { background: #245cc4; border: none; border-radius: 7px; padding: 7px 13px; font-weight: 600; }
QPushButton:hover { background: #3270e4; }
QPushButton:disabled { background: #2a3240; color: #748197; }
QTabWidget::pane { border: 1px solid #273246; border-radius: 6px; top: -1px; }
QTabBar::tab { background: #121a27; padding: 8px 14px; border-bottom: 2px solid transparent; }
QTabBar::tab:selected { color: #75a7ff; border-bottom-color: #4d8df7; }
QStatusBar { background: #0d1420; border-top: 1px solid #273246; color: #93a2b8; }
QScrollBar:vertical { background: #101722; width: 10px; margin: 0; }
QScrollBar::handle:vertical { background: #35445b; min-height: 28px; border-radius: 5px; }
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
QSplitter::handle { background: #273246; }
QLabel#SectionLabel { color: #7faaff; font-size: 9pt; font-weight: 700; text-transform: uppercase; }
QLabel#EmptyHint { color: #7c899d; padding: 24px; }
"""
