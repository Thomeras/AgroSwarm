"""Dark QSS stylesheet for Scout Swarm Center."""

DARK_QSS = """
* {
    font-size: 12px;
}

QMainWindow, QDialog {
    background-color: #0F172A;
}

QWidget {
    background-color: #0F172A;
    color: #E2E8F0;
}

/* ── Groups ─────────────────────────────────────────────────── */
QGroupBox {
    border: 1px solid #1E293B;
    border-radius: 6px;
    margin-top: 14px;
    padding: 16px 8px 8px 8px;
    font-weight: bold;
    font-size: 11px;
    color: #64748B;
}
QGroupBox::title {
    subcontrol-origin: margin;
    subcontrol-position: top left;
    left: 10px;
    padding: 0 6px;
    background-color: #0F172A;
    color: #64748B;
}

/* ── Buttons ─────────────────────────────────────────────────── */
QPushButton {
    background-color: #1E293B;
    color: #E2E8F0;
    border: 1px solid #334155;
    border-radius: 5px;
    padding: 6px 10px;
    min-height: 30px;
    min-width: 0;
    font-size: 12px;
}
QPushButton:hover {
    background-color: #334155;
    border-color: #475569;
}
QPushButton:pressed {
    background-color: #0F172A;
    border-color: #1D4ED8;
}
QPushButton:disabled {
    color: #334155;
    border-color: #1E293B;
    background-color: #0F172A;
}
QPushButton:flat {
    border: none;
    background-color: transparent;
}
QPushButton:flat:hover {
    background-color: #1E293B;
}

/* ── Tables ─────────────────────────────────────────────────── */
QTableWidget {
    background-color: #020617;
    alternate-background-color: #0F172A;
    border: 1px solid #1E293B;
    gridline-color: #1E293B;
    selection-background-color: #1D4ED8;
    selection-color: #F8FAFC;
    outline: none;
}
QTableWidget::item {
    padding: 4px 6px;
    border: none;
}
QHeaderView {
    background-color: #0F172A;
}
QHeaderView::section {
    background-color: #0F172A;
    color: #64748B;
    padding: 5px 6px;
    border: none;
    border-bottom: 1px solid #1E293B;
    border-right: 1px solid #1E293B;
    font-weight: bold;
    font-size: 11px;
}
QHeaderView::section:last {
    border-right: none;
}

/* ── Log / text areas ────────────────────────────────────────── */
QPlainTextEdit, QTextEdit {
    background-color: #020617;
    border: 1px solid #1E293B;
    border-radius: 4px;
    color: #94A3B8;
    font-family: "Fira Code", "Consolas", "DejaVu Sans Mono", monospace;
    font-size: 11px;
    selection-background-color: #334155;
}

/* ── Combos + spins ─────────────────────────────────────────── */
QComboBox {
    background-color: #1E293B;
    border: 1px solid #334155;
    border-radius: 5px;
    padding: 4px 8px;
    min-height: 28px;
    min-width: 0;
    color: #E2E8F0;
}
QComboBox:hover {
    border-color: #475569;
}
QComboBox::drop-down {
    border: none;
    padding-right: 6px;
}
QComboBox::down-arrow {
    image: none;
    width: 0;
}
QComboBox QAbstractItemView {
    background-color: #1E293B;
    border: 1px solid #334155;
    selection-background-color: #334155;
    color: #E2E8F0;
    outline: none;
}

QDoubleSpinBox, QSpinBox {
    background-color: #1E293B;
    border: 1px solid #334155;
    border-radius: 5px;
    padding: 4px 8px;
    min-height: 28px;
    min-width: 56px;
    color: #E2E8F0;
}
QDoubleSpinBox:hover, QSpinBox:hover {
    border-color: #475569;
}
QDoubleSpinBox::up-button, QDoubleSpinBox::down-button,
QSpinBox::up-button, QSpinBox::down-button {
    background-color: #334155;
    border: none;
    width: 16px;
}

/* ── Progress bar ────────────────────────────────────────────── */
QProgressBar {
    background-color: #1E293B;
    border: 1px solid #334155;
    border-radius: 4px;
    text-align: center;
    color: #E2E8F0;
    min-height: 22px;
    font-size: 12px;
}
QProgressBar::chunk {
    background-color: #22C55E;
    border-radius: 3px;
}

/* ── Checkboxes ──────────────────────────────────────────────── */
QCheckBox {
    spacing: 8px;
    color: #CBD5E1;
}
QCheckBox::indicator {
    width: 16px;
    height: 16px;
    border: 1px solid #334155;
    border-radius: 3px;
    background-color: #1E293B;
}
QCheckBox::indicator:checked {
    background-color: #22C55E;
    border-color: #16A34A;
}
QCheckBox::indicator:hover {
    border-color: #475569;
}

/* ── Tabs ────────────────────────────────────────────────────── */
QTabWidget::pane {
    border: 1px solid #1E293B;
    background-color: #0F172A;
}
QTabBar::tab {
    background-color: #020617;
    color: #64748B;
    padding: 8px 18px;
    border: 1px solid #1E293B;
    border-bottom: none;
    border-radius: 5px 5px 0 0;
    min-width: 80px;
}
QTabBar::tab:selected {
    background-color: #0F172A;
    color: #E2E8F0;
    border-color: #334155;
}
QTabBar::tab:hover:!selected {
    background-color: #1E293B;
    color: #CBD5E1;
}

/* ── Splitter ────────────────────────────────────────────────── */
QSplitter::handle {
    background-color: #1E293B;
}
QSplitter::handle:horizontal {
    width: 3px;
}
QSplitter::handle:vertical {
    height: 3px;
}
QSplitter::handle:hover {
    background-color: #334155;
}

/* ── Status bar ──────────────────────────────────────────────── */
QStatusBar {
    background-color: #020617;
    color: #475569;
    border-top: 1px solid #1E293B;
    font-size: 11px;
}

/* ── Menu ────────────────────────────────────────────────────── */
QMenuBar {
    background-color: #020617;
    color: #CBD5E1;
    border-bottom: 1px solid #1E293B;
}
QMenuBar::item:selected {
    background-color: #1E293B;
    border-radius: 3px;
}
QMenu {
    background-color: #1E293B;
    border: 1px solid #334155;
    color: #E2E8F0;
    padding: 4px;
}
QMenu::item {
    padding: 6px 20px;
    border-radius: 3px;
}
QMenu::item:selected {
    background-color: #334155;
}
QMenu::separator {
    height: 1px;
    background-color: #334155;
    margin: 4px 0;
}

/* ── Scrollbars ──────────────────────────────────────────────── */
QScrollBar:vertical {
    background-color: #020617;
    width: 10px;
    border: none;
}
QScrollBar::handle:vertical {
    background-color: #334155;
    border-radius: 5px;
    min-height: 24px;
}
QScrollBar::handle:vertical:hover {
    background-color: #475569;
}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
    height: 0;
    border: none;
}
QScrollBar:horizontal {
    background-color: #020617;
    height: 10px;
    border: none;
}
QScrollBar::handle:horizontal {
    background-color: #334155;
    border-radius: 5px;
    min-width: 24px;
}
QScrollBar::handle:horizontal:hover {
    background-color: #475569;
}
QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal {
    width: 0;
    border: none;
}

/* ── Labels ──────────────────────────────────────────────────── */
QLabel {
    color: #E2E8F0;
    background-color: transparent;
}

/* ── Tooltips ────────────────────────────────────────────────── */
QToolTip {
    background-color: #1E293B;
    color: #E2E8F0;
    border: 1px solid #334155;
    border-radius: 4px;
    padding: 4px 8px;
    font-size: 12px;
}
"""
