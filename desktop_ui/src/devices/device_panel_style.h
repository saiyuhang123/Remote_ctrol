#pragma once
#include <QFrame>
#include <QLabel>
#include <QStyle>

namespace deviceui {
inline QFrame *card(QWidget *parent) {
  auto *frame = new QFrame(parent);
  frame->setProperty("deviceCard", true);
  return frame;
}
inline QLabel *label(const QString &text, QWidget *parent, bool muted = false) {
  auto *result = new QLabel(text, parent);
  result->setTextFormat(Qt::PlainText);
  result->setWordWrap(true);
  result->setProperty("deviceMuted", muted);
  return result;
}
inline void tone(QWidget *widget, const char *value) {
  if (widget->property("deviceTone").toString() == QLatin1String(value)) return;
  widget->setProperty("deviceTone", value);
  widget->style()->unpolish(widget);
  widget->style()->polish(widget);
  widget->update();
}
inline QString styleSheet() {
  return QStringLiteral(R"QSS(
QWidget#devicePanel { background:#f3f6fb; color:#24344b; }
#devicePanel QWidget { font-size:13px; color:#24344b; }
#devicePanel QFrame[deviceCard="true"] { background:#ffffff; border:1px solid #dfe6ef; border-radius:9px; }
#devicePanel QLabel { background:transparent; border:none; }
#devicePanel QLabel#deviceTitle { font-size:22px; font-weight:600; color:#17253b; }
#devicePanel QLabel[deviceHeading="true"] { font-size:15px; font-weight:600; }
#devicePanel QLabel[deviceMuted="true"] { color:#748197; font-size:12px; }
#devicePanel QLabel[deviceMetric="true"] { font-size:19px; font-weight:600; }
#devicePanel QLabel#gasValue { font-size:42px; font-weight:600; }
#devicePanel QLabel#ptzActivity { font-size:25px; font-weight:600; }
#devicePanel QLabel[deviceTone="good"] { color:#167347; }
#devicePanel QLabel[deviceTone="warning"] { color:#986016; }
#devicePanel QLabel[deviceTone="danger"] { color:#b93838; }
#devicePanel QLabel[deviceTone="info"] { color:#2563eb; }
#devicePanel QLabel[deviceTone="muted"] { color:#748197; }
#devicePanel QLabel[deviceBadge="true"] { background:#eef3f9; border-radius:5px; padding:5px 9px; font-size:12px; }
#devicePanel QLabel[deviceBadge="true"][deviceTone="good"] { background:#ecf8f1; }
#devicePanel QLabel[deviceBadge="true"][deviceTone="warning"] { background:#fff6e5; }
#devicePanel QLabel[deviceBadge="true"][deviceTone="danger"] { background:#fff0ef; }
#devicePanel QLabel[deviceBadge="true"][deviceTone="info"] { background:#edf3ff; }
#devicePanel QLabel#ptzReadiness { background:#eef3f9; border-radius:6px; padding:8px; }
#devicePanel QLabel#ptzReadiness[deviceTone="good"] { background:#ecf8f1; }
#devicePanel QLabel#ptzReadiness[deviceTone="warning"] { background:#fff6e5; }
#devicePanel QTabWidget::pane { border:none; }
#devicePanel QTabBar::tab { background:transparent; color:#6a7890; border:none; border-bottom:3px solid transparent; padding:10px 18px; margin-right:8px; }
#devicePanel QTabBar::tab:selected { color:#2563eb; border-bottom-color:#2563eb; font-weight:600; }
#devicePanel QTabBar::tab:hover { color:#2563eb; background:#eaf1fc; }
#devicePanel QLineEdit, #devicePanel QSpinBox, #devicePanel QDoubleSpinBox { background:#f8fafc; border:1px solid #dce4ee; border-radius:5px; min-height:28px; padding:2px 8px; selection-background-color:#2563eb; }
#devicePanel QLineEdit:focus, #devicePanel QSpinBox:focus, #devicePanel QDoubleSpinBox:focus { border-color:#5286ed; background:#ffffff; }
#devicePanel QLineEdit:disabled, #devicePanel QSpinBox:disabled, #devicePanel QDoubleSpinBox:disabled { color:#92a0b2; background:#f1f4f8; }
#devicePanel QSpinBox, #devicePanel QDoubleSpinBox { padding-right:23px; }
#devicePanel QSpinBox::up-button, #devicePanel QDoubleSpinBox::up-button { subcontrol-origin:border; subcontrol-position:top right; width:21px; height:16px; border:none; }
#devicePanel QSpinBox::down-button, #devicePanel QDoubleSpinBox::down-button { subcontrol-origin:border; subcontrol-position:bottom right; width:21px; height:16px; border:none; }
#devicePanel QSpinBox::up-arrow, #devicePanel QDoubleSpinBox::up-arrow { image:url(:/console/chevron-up.svg); width:12px; height:12px; }
#devicePanel QSpinBox::down-arrow, #devicePanel QDoubleSpinBox::down-arrow { image:url(:/console/chevron-down.svg); width:12px; height:12px; }
#devicePanel QPushButton { background:#ffffff; color:#33455e; border:1px solid #d4deea; border-radius:5px; min-height:28px; padding:4px 12px; }
#devicePanel QPushButton:hover { color:#2563eb; background:#f0f5ff; border-color:#a7c1f5; }
#devicePanel QPushButton:pressed, #devicePanel QPushButton:checked { color:#2563eb; background:#e9f1ff; border-color:#a7c1f5; }
#devicePanel QPushButton:focus { border-color:#5286ed; }
#devicePanel QPushButton[devicePrimary="true"] { color:#ffffff; background:#2563eb; border-color:#2563eb; font-weight:600; }
#devicePanel QPushButton[devicePrimary="true"]:hover { background:#1d4ed8; }
#devicePanel QPushButton[deviceStop="true"] { color:#bc4343; border-color:#edcece; }
#devicePanel QPushButton[deviceStop="true"]:hover { background:#fff1f1; border-color:#db9e9e; }
#devicePanel QPushButton:disabled { color:#9ba7b8; background:#f3f5f8; border-color:#e4e9f0; }
#devicePanel QScrollArea { border:none; background:transparent; }
#devicePanel QScrollArea > QWidget > QWidget { background:transparent; }
#devicePanel QScrollBar:vertical { background:#f3f6fa; width:8px; margin:0; }
#devicePanel QScrollBar::handle:vertical { background:#cad5e3; border-radius:4px; min-height:24px; }
#devicePanel QScrollBar::add-line:vertical, #devicePanel QScrollBar::sub-line:vertical { height:0; }
#devicePanel QScrollBar::add-page:vertical, #devicePanel QScrollBar::sub-page:vertical { background:none; }
#devicePanel QCheckBox { spacing:7px; }
)QSS");
}
}  // namespace deviceui
