#include "ui/mainwindow.h"
#include "runtime/config.h"
#include <QApplication>
#include <QCommandLineParser>
#include <QDir>
#include <QFileInfo>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>
#include <csignal>
#include <iostream>

namespace { volatile std::sig_atomic_t interrupted = 0; void handleSignal(int) { interrupted = 1; } }
int main(int argc, char **argv) {
  QApplication app(argc, argv);
  QCoreApplication::setOrganizationName("RemoteCtrol"); QCoreApplication::setApplicationName("robot_desktop");
  QCommandLineParser parser; parser.setApplicationDescription("ROS2 Qt / RViz2 desktop; connects without issuing commands by default"); parser.addHelpOption();
  parser.addOptions({{{"c", "config"}, "Desktop JSON configuration", "path"}, {"resource-dir", "Directory containing config/rviz", "path"}, {"project-root", "Remote_ctrol project directory", "path"}, {"offline", "Isolated ROS domain, no network controls or process launches"}, {"smoke-test", "Offline GUI smoke test of all profiles"}, {"exit-after-ms", "Exit automatically after N milliseconds", "ms"}, {"screenshot", "Save a screenshot before exit", "path"}});
  parser.process(app);
  const bool offline = parser.isSet("offline") || parser.isSet("smoke-test");
  if (offline) { qputenv("ROS_DOMAIN_ID", "218"); qputenv("ROS_LOCALHOST_ONLY", "1"); }
  const auto installed = QDir(QCoreApplication::applicationDirPath()).absoluteFilePath("../share/robot_desktop");
  const QString resources = parser.isSet("resource-dir") ? parser.value("resource-dir") : QFileInfo::exists(installed + "/config/desktop.json") ? installed : QString(DESKTOP_UI_SOURCE_DIR);
  const QString project = parser.isSet("project-root") ? parser.value("project-root") : !qEnvironmentVariable("ROBOT_PROJECT_ROOT").isEmpty() ? qEnvironmentVariable("ROBOT_PROJECT_ROOT") : QDir(resources).absoluteFilePath("..");
  int result = 1;
  try {
    const auto config = DesktopConfig::load(parser.isSet("config") ? parser.value("config") : resources + "/config/desktop.json", resources, project);
    rclcpp::init(0, nullptr, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    std::signal(SIGINT, handleSignal); std::signal(SIGTERM, handleSignal);
    {
      MainWindow window(config, offline); window.show();
      QTimer signalTimer; signalTimer.setInterval(100); QObject::connect(&signalTimer, &QTimer::timeout, &app, [&app] { if (interrupted) app.quit(); }); signalTimer.start();
      if (parser.isSet("smoke-test")) {
        for (int i = 0; i < desktopProfiles().size(); ++i) QTimer::singleShot(700 * (i + 1), &window, [&window, i] { window.setProfileIndex(i); });
      }
      const int duration = parser.isSet("exit-after-ms") ? parser.value("exit-after-ms").toInt() : parser.isSet("smoke-test") ? 6500 : 0;
      if (duration > 0) QTimer::singleShot(duration, &window, [&] {
        if (parser.isSet("screenshot")) {
          const auto path = QFileInfo(parser.value("screenshot")).absoluteFilePath();
          if (!window.saveScreenshot(path)) {
            std::cerr << "Screenshot failed: " << path.toStdString() << ". Check that the directory exists and is writable.\n";
            app.exit(2);
            return;
          }
          std::cout << "Screenshot saved: " << path.toStdString() << '\n';
        }
        app.quit();
      });
      result = app.exec();
    }
    rclcpp::shutdown();
  } catch (const std::exception &e) { std::cerr << "Desktop startup failed: " << e.what() << '\n'; if (rclcpp::ok()) rclcpp::shutdown(); }
  return result;
}
