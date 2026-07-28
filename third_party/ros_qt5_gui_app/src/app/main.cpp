/*
 * @Author: chengyangkj chengyangkj@qq.com
 * @Date: 2023-09-28 14:56:04
 * @LastEditors: chengyangkj chengyangkj@qq.com
 * @LastEditTime: 2023-10-05 11:39:01
 * @FilePath: /ROS2_Qt5_Gui_App/src/app/main.cpp
 */
#ifndef SDL_MAIN_HANDLED
#define SDL_MAIN_HANDLED
#endif

#include <QApplication>
#include <QLabel>
#include <QMovie>
#include <QPixmap>
#include <QSplashScreen>
#include <QThread>
#include <QTimer>
#include <csignal>
#include <iostream>
#include <QCoreApplication>
#include <QDir>
#include "config/config_manager.h"
#include "logger/logger.h"
#include "mainwindow.h"
#include "ui/theme/ui_theme_manager.h"


static volatile std::sig_atomic_t g_exit_requested = 0;

void signalHandler(int signal) {
  if (signal == SIGINT || signal == SIGTERM) {
    g_exit_requested = 1;
  }
}

int main(int argc, char *argv[]) {
  QApplication a(argc, argv);
  Config::ConfigManager::Instance();
  const QString application_root = QCoreApplication::applicationDirPath();
  const QString configured_theme_root = QString::fromStdString(
      GET_CONFIG_VALUE("UiThemeRoot", ""));
  const QString theme_root = configured_theme_root.isEmpty()
                                 ? QDir(application_root).filePath("resources/themes")
                                 : configured_theme_root;
  const QString theme_id = QString::fromStdString(
      GET_CONFIG_VALUE("UiThemeId", "agt-light"));
  const QString density = QString::fromStdString(
      GET_CONFIG_VALUE("UiDensity", "comfortable"));
  UiThemeManager theme_manager;
  QString theme_error;
  if (!theme_manager.Apply(&a, theme_root, theme_id, density, &theme_error)) {
    LOG_ERROR("failed to load UI theme " << theme_id.toStdString() << ": "
                                         << theme_error.toStdString());
    if (theme_id != QStringLiteral("agt-light"))
      theme_manager.Apply(&a, theme_root, QStringLiteral("agt-light"), density,
                          &theme_error);
  }
  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);

  // Process termination requests on Qt's main thread.
  QTimer signal_timer;
  QObject::connect(&signal_timer, &QTimer::timeout, [&a]() {
    if (g_exit_requested) {
      a.quit();
    }
  });
  signal_timer.start(100);

  MainWindow main_window;
  main_window.show();
  LOG_INFO("ros_qt5_gui_app init!");
  return a.exec();
}
