#pragma once

#include <QFrame>
#include <QHeaderView>
#include <QLabel>
#include <QStyle>
#include <QTableWidget>

// Scoped to the patrol window; the main console and RViz keep their own theme.
namespace patrolui {
inline QFrame *card(QWidget *parent) {
  auto *frame = new QFrame(parent);
  frame->setProperty("patrolCard", true);
  return frame;
}

inline QLabel *heading(const QString &text, QWidget *parent) {
  auto *label = new QLabel(text, parent);
  label->setProperty("patrolHeading", true);
  return label;
}

inline QLabel *hint(const QString &text, QWidget *parent) {
  auto *label = new QLabel(text, parent);
  label->setProperty("patrolMuted", true);
  label->setWordWrap(true);
  return label;
}

inline void tone(QWidget *widget, const char *value) {
  if (widget->property("patrolTone").toString() == QLatin1String(value)) return;
  widget->setProperty("patrolTone", value);
  widget->style()->unpolish(widget);
  widget->style()->polish(widget);
  widget->update();
}

inline void configureTable(QTableWidget *view) {
  view->setAlternatingRowColors(true);
  view->setShowGrid(false);
  view->setSelectionBehavior(QAbstractItemView::SelectRows);
  view->setSelectionMode(QAbstractItemView::SingleSelection);
  view->setEditTriggers(QAbstractItemView::NoEditTriggers);
  view->setWordWrap(false);
  view->setTextElideMode(Qt::ElideRight);
  view->verticalHeader()->setVisible(false);
  view->verticalHeader()->setDefaultSectionSize(38);
  view->horizontalHeader()->setHighlightSections(false);
  view->horizontalHeader()->setDefaultAlignment(Qt::AlignLeft | Qt::AlignVCenter);
  view->horizontalHeader()->setMinimumSectionSize(56);
  view->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
  view->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);
}

inline QString styleSheet() {
  return QStringLiteral(R"QSS(
QWidget#patrolPage { background: #f3f6fb; color: #17253b; }
#patrolPage QWidget { font-size: 13px; color: #24344b; }
#patrolPage QFrame[patrolCard="true"] { background: #ffffff; border: 1px solid #dfe6ef; border-radius: 9px; }
#patrolPage QLabel { background: transparent; border: none; }
#patrolPage QLabel#patrolTitle { font-size: 22px; font-weight: 600; color: #17253b; }
#patrolPage QLabel[patrolHeading="true"] { font-size: 15px; font-weight: 600; color: #17253b; }
#patrolPage QLabel[patrolMuted="true"] { color: #748197; font-size: 12px; }
#patrolPage QLabel[patrolBadge="true"] { color: #52627a; background: #eef3f9; border-radius: 5px; padding: 5px 9px; font-size: 12px; }
#patrolPage QLabel[patrolTone="good"] { color: #167347; background: #ecf8f1; border-radius: 5px; padding: 6px 10px; }
#patrolPage QLabel[patrolTone="warning"] { color: #986016; background: #fff6e5; border-radius: 5px; padding: 6px 10px; }
#patrolPage QLabel[patrolTone="danger"] { color: #b93838; background: #fff0ef; border-radius: 5px; padding: 6px 10px; }
#patrolPage QLabel[patrolTone="info"] { color: #2563eb; background: #edf3ff; border-radius: 5px; padding: 6px 10px; }
#patrolPage QGroupBox { background: #ffffff; border: 1px solid #e3e9f1; border-radius: 7px; margin-top: 10px; padding: 13px 10px 10px; font-weight: 600; }
#patrolPage QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; color: #52627a; }
#patrolPage QLineEdit, #patrolPage QComboBox, #patrolPage QSpinBox, #patrolPage QDoubleSpinBox {
  background: #f8fafc; border: 1px solid #dce4ee; border-radius: 5px; min-height: 28px; padding: 2px 8px; selection-background-color: #2563eb;
}
#patrolPage QLineEdit:focus, #patrolPage QComboBox:focus, #patrolPage QSpinBox:focus, #patrolPage QDoubleSpinBox:focus { border-color: #5286ed; background: #ffffff; }
#patrolPage QLineEdit:disabled, #patrolPage QComboBox:disabled, #patrolPage QSpinBox:disabled, #patrolPage QDoubleSpinBox:disabled { color: #92a0b2; background: #f1f4f8; }
#patrolPage QComboBox::drop-down { border: none; width: 22px; }
#patrolPage QComboBox::down-arrow { image: url(:/console/chevron-down.svg); width: 12px; height: 12px; }
#patrolPage QSpinBox, #patrolPage QDoubleSpinBox { padding-right: 23px; }
#patrolPage QSpinBox::up-button, #patrolPage QDoubleSpinBox::up-button { subcontrol-origin: border; subcontrol-position: top right; width: 21px; height: 16px; border: none; }
#patrolPage QSpinBox::down-button, #patrolPage QDoubleSpinBox::down-button { subcontrol-origin: border; subcontrol-position: bottom right; width: 21px; height: 16px; border: none; }
#patrolPage QSpinBox::up-arrow, #patrolPage QDoubleSpinBox::up-arrow { image: url(:/console/chevron-up.svg); width: 12px; height: 12px; }
#patrolPage QSpinBox::down-arrow, #patrolPage QDoubleSpinBox::down-arrow { image: url(:/console/chevron-down.svg); width: 12px; height: 12px; }
#patrolPage QPushButton { background: #ffffff; color: #33455e; border: 1px solid #d4deea; border-radius: 5px; min-height: 28px; padding: 3px 12px; }
#patrolPage QPushButton:hover { color: #2563eb; background: #f0f5ff; border-color: #a7c1f5; }
#patrolPage QPushButton:pressed, #patrolPage QPushButton:checked { color: #2563eb; background: #e9f1ff; border-color: #a7c1f5; }
#patrolPage QPushButton:focus { border-color: #5286ed; }
#patrolPage QPushButton[primary="true"] { color: #ffffff; background: #2563eb; border-color: #2563eb; font-weight: 600; }
#patrolPage QPushButton[primary="true"]:hover { background: #1d4ed8; border-color: #1d4ed8; }
#patrolPage QPushButton[patrolDanger="true"] { color: #bc4343; border-color: #edcece; }
#patrolPage QPushButton[patrolDanger="true"]:hover { background: #fff1f1; border-color: #db9e9e; }
#patrolPage QPushButton:disabled { color: #9ba7b8; background: #f3f5f8; border-color: #e4e9f0; }
#patrolPage QTabWidget::pane { border: none; }
#patrolPage QTabBar::tab { background: transparent; color: #6a7890; border: none; border-bottom: 3px solid transparent; padding: 11px 20px; margin-right: 8px; }
#patrolPage QTabBar::tab:selected { color: #2563eb; border-bottom-color: #2563eb; font-weight: 600; }
#patrolPage QTabBar::tab:hover { color: #2563eb; background: #eaf1fc; }
#patrolPage QTabWidget#roadControlTabs QTabBar::tab { padding: 9px 9px; margin: 0; font-size: 12px; }
#patrolPage QTableWidget { border: 1px solid #e5eaf2; border-radius: 5px; background: #ffffff; alternate-background-color: #f8fafc; selection-background-color: #eaf1ff; selection-color: #1d4ed8; }
#patrolPage QTableWidget::item { padding: 0 8px; border-bottom: 1px solid #eff3f8; }
#patrolPage QTableWidget::item:selected { color: #1d4ed8; background: #eaf1ff; }
#patrolPage QHeaderView::section { background: #f3f6fa; color: #6a7890; padding: 9px 8px; border: none; border-bottom: 1px solid #e3eaf3; font-size: 12px; }
#patrolPage QScrollArea { border: none; background: transparent; }
#patrolPage QScrollArea > QWidget > QWidget { background: transparent; }
#patrolPage QScrollBar:vertical { background: #f3f6fa; width: 8px; margin: 0; }
#patrolPage QScrollBar::handle:vertical { background: #cad5e3; border-radius: 4px; min-height: 24px; }
#patrolPage QScrollBar::add-line:vertical, #patrolPage QScrollBar::sub-line:vertical { height: 0; }
#patrolPage QScrollBar::add-page:vertical, #patrolPage QScrollBar::sub-page:vertical { background: none; }
#patrolPage QSplitter::handle { background: transparent; width: 10px; }
#patrolPage QGraphicsView { background: #f7f9fc; border: 1px solid #e3eaf3; border-radius: 6px; }
#patrolPage QCheckBox { spacing: 7px; }
)QSS");
}
}  // namespace patrolui
