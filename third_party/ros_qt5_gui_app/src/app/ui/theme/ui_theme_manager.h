#pragma once

#include <QString>

class QApplication;

struct ThemeTokens {
  QString background;
  QString surface;
  QString text;
  QString muted_text;
  QString border;
  QString accent;
  QString success;
  QString warning;
  QString danger;
};

struct ThemeManifest {
  QString id;
  QString name;
  ThemeTokens tokens;
};

class UiThemeManager {
 public:
  bool Apply(QApplication *application, const QString &theme_root,
             const QString &theme_id, const QString &density,
             QString *error = nullptr) const;

 private:
  bool LoadManifest(const QString &path, ThemeManifest *manifest,
                    QString *error) const;
};
