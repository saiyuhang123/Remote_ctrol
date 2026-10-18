#pragma once
#include <QString>

class QWidget;

// Selectors are scoped to the console's named panels, never the whole app.
QString consoleStyleSheet();
QString consoleDockStyleSheet();
void setConsoleTone(QWidget *widget, const char *tone);
