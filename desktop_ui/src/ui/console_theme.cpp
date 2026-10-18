#include "ui/console_theme.h"
#include <QStyle>
#include <QVariant>
#include <QWidget>

void setConsoleTone(QWidget *widget, const char *tone) {
  const QString value = QString::fromLatin1(tone);
  if (widget->property("consoleTone").toString() == value) return;
  widget->setProperty("consoleTone", value);
  widget->style()->unpolish(widget);
  widget->style()->polish(widget);
  widget->update();
}

QString consoleStyleSheet() {
  return QStringLiteral(R"QSS(
QWidget#consoleShell { background: #f3f5f9; }
QWidget#consoleHeader, QToolBar#consoleTools { background: #ffffff; border: none; border-bottom: 1px solid #dce3ec; }
QWidget#consoleHeader QLabel, QToolBar#consoleTools QLabel { color: #17253b; }
QLabel#consoleTitle { font-size: 18px; font-weight: 600; }
QLabel#consoleSource { color: #52627a; font-size: 11px; }
QLabel#consoleMode { background: #eaf1ff; color: #2563eb; border-radius: 5px; padding: 6px 10px; }
QWidget#consoleControls, QWidget#consoleMonitor, QWidget#consoleStatusStrip, QWidget#consoleLogPanel { background: #ffffff; color: #17253b; }
#consoleControls QLabel, #consoleMonitor QLabel, #consoleStatusStrip QLabel, #consoleLogPanel QLabel { color: #17253b; }
#consoleControls QLabel[muted="true"], #consoleMonitor QLabel[muted="true"], #consoleStatusStrip QLabel[muted="true"] { color: #52627a; }
#consoleControls QGroupBox, #consoleMonitor QGroupBox { border: 1px solid #dce3ec; border-radius: 7px; margin-top: 12px; padding: 12px 8px 8px; font-weight: 600; }
#consoleControls QGroupBox::title, #consoleMonitor QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0px 4px; color: #17253b; }
#consoleControls QScrollArea, #consoleMonitor QScrollArea { border: none; background: #ffffff; }
#consoleControls QScrollArea > QWidget > QWidget, #consoleMonitor QScrollArea > QWidget > QWidget { background: #ffffff; }
#consoleControls QTabWidget::pane, #consoleMonitor QTabWidget::pane { border: none; border-top: 1px solid #e4eaf2; }
#consoleControls QTabBar::tab, #consoleMonitor QTabBar::tab { background: #f3f5f9; color: #52627a; padding: 9px 13px; border: none; border-bottom: 2px solid transparent; }
#consoleControls QTabBar::tab:selected, #consoleMonitor QTabBar::tab:selected { background: #ffffff; color: #2563eb; border-bottom: 2px solid #2563eb; }
#consoleControls QTabBar::tab:hover, #consoleMonitor QTabBar::tab:hover { color: #2563eb; }
#consoleControls QLineEdit, #consoleControls QDoubleSpinBox, #consoleControls QComboBox,
#consoleMonitor QLineEdit, #consoleMonitor QDoubleSpinBox, #consoleMonitor QComboBox {
  color: #17253b; background: #f8fafc; border: 1px solid #dce3ec; border-radius: 5px; min-height: 27px; padding: 2px 6px; selection-background-color: #2563eb;
}
#consoleControls QLineEdit:focus, #consoleControls QDoubleSpinBox:focus, #consoleControls QComboBox:focus,
#consoleMonitor QLineEdit:focus, #consoleMonitor QDoubleSpinBox:focus { border-color: #2563eb; background: #ffffff; }
#consoleControls QLineEdit:read-only, #consoleControls QDoubleSpinBox:disabled, #consoleMonitor QDoubleSpinBox:disabled { color: #637188; background: #f3f5f9; }
#consoleHeader QPushButton, #consoleControls QPushButton, #consoleMonitor QPushButton {
  background: #ffffff; color: #17253b; border: 1px solid #cbd5e1; border-radius: 5px; min-height: 28px; padding: 3px 9px;
}
#consoleHeader QPushButton:hover, #consoleControls QPushButton:hover, #consoleMonitor QPushButton:hover { background: #eef4ff; border-color: #8aaaf0; }
#consoleHeader QPushButton:pressed, #consoleControls QPushButton:pressed, #consoleMonitor QPushButton:pressed { background: #dce8ff; }
#consoleControls QPushButton[consoleRole="primary"], #consoleMonitor QPushButton[consoleRole="primary"] { color: #ffffff; background: #2563eb; border-color: #2563eb; font-weight: 600; }
#consoleControls QPushButton[consoleRole="primary"]:hover, #consoleMonitor QPushButton[consoleRole="primary"]:hover { background: #1d4ed8; }
#consoleControls QPushButton[consoleRole="direction"], #consoleMonitor QPushButton[consoleRole="direction"] { min-height: 38px; padding: 4px 2px; }
#consoleHeader QPushButton:disabled, #consoleControls QPushButton:disabled, #consoleMonitor QPushButton:disabled { color: #7b8797; background: #f0f3f7; border-color: #dce3ec; }
#consoleHeader QPushButton:focus, #consoleControls QPushButton:focus, #consoleMonitor QPushButton:focus { border: 1px solid #2563eb; }
#consoleHeader QPushButton#emergencyStop { color: #ffffff; background: #c93434; border: 1px solid #b82929; font-weight: 600; min-width: 72px; min-height: 34px; }
#consoleHeader QPushButton#emergencyStop:hover { background: #b82929; }
#consoleHeader QPushButton#emergencyStop:disabled { color: #f6dcdc; background: #a76c6c; border-color: #a76c6c; }
#consoleHeader QCheckBox { color: #17253b; padding: 4px; }
#consoleControls QCheckBox, #consoleMonitor QCheckBox { spacing: 7px; padding: 3px 0px; }
#consoleControls QTableWidget { background: #ffffff; }
#consoleMonitor QTableWidget { background: #ffffff; alternate-background-color: #f7f9fc; border: 1px solid #dce3ec; gridline-color: #e7edf5; selection-background-color: #eaf1ff; selection-color: #17253b; }
#consoleMonitor QHeaderView::section { background: #f3f5f9; color: #52627a; padding: 6px; border: none; border-bottom: 1px solid #dce3ec; }
QToolBar#consoleTools { spacing: 4px; padding: 5px 9px; }
QToolBar#consoleTools QToolButton { color: #34435a; border: none; border-radius: 4px; padding: 6px 8px; }
QToolBar#consoleTools QToolButton:hover, QToolBar#consoleTools QToolButton:checked { color: #2563eb; background: #eaf1ff; }
QToolBar#consoleTools QToolButton:disabled { color: #7b8797; }
QToolBar#consoleTools QComboBox { min-height: 28px; color: #17253b; background: #f8fafc; border: 1px solid #dce3ec; border-radius: 5px; padding: 2px 7px; }
QWidget#consoleStatusStrip { border: 1px solid #dce3ec; border-radius: 7px; }
#consoleStatusStrip QLabel[metric="true"] { font-size: 23px; font-weight: 600; }
#consoleStatusStrip QLabel[consoleTone="good"], #consoleHeader QLabel[consoleTone="good"] { color: #15803d; }
#consoleStatusStrip QLabel[consoleTone="warning"], #consoleControls QLabel[consoleTone="warning"] { color: #b45309; }
#consoleStatusStrip QLabel[consoleTone="danger"] { color: #c93434; }
#consoleStatusStrip QLabel[consoleTone="muted"] { color: #637188; }
#consoleLogPanel { border: 1px solid #dce3ec; border-radius: 5px; }
#consoleLogPanel QToolButton, #consoleStatusStrip QToolButton, #consoleControls QToolButton, #consoleMonitor QToolButton { color: #2563eb; background: transparent; border: none; padding: 5px; }
#consoleLogPanel QPlainTextEdit { color: #34435a; background: #f8fafc; border: none; border-top: 1px solid #dce3ec; font-family: monospace; font-size: 11px; }
#consoleLogPanel QLabel[consoleTone="danger"] { color: #b82929; }
)QSS");
}

QString consoleDockStyleSheet() {
  return QStringLiteral(R"QSS(
#consoleDocks ads--CDockContainerWidget, #consoleDocks ads--CDockAreaWidget { background: #f3f5f9; }
#consoleDocks ads--CDockWidget { border: 1px solid #dce3ec; background: #ffffff; }
#consoleDocks ads--CDockWidgetTab, #consoleDocks ads--CDockWidgetTab[activeTab="true"] { background: #ffffff; border: none; padding: 5px 8px; }
#consoleDocks ads--CDockWidgetTab QLabel { color: #52627a; }
#consoleDocks ads--CDockSplitter::handle { background: #f3f5f9; }
#consoleDocks ads--CDockAreaTitleBar { background: #ffffff; border-bottom: 1px solid #e4eaf2; }
)QSS");
}
