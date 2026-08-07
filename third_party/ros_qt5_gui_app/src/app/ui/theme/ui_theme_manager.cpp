#include "ui/theme/ui_theme_manager.h"

#include <QApplication>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMap>

bool UiThemeManager::LoadManifest(const QString &path, ThemeManifest *manifest,
                                  QString *error) const {
  QFile file(path);
  if (!file.open(QIODevice::ReadOnly)) {
    if (error) *error = QStringLiteral("cannot open theme manifest: ") + path;
    return false;
  }
  QJsonParseError parse_error;
  const auto document = QJsonDocument::fromJson(file.readAll(), &parse_error);
  if (parse_error.error != QJsonParseError::NoError || !document.isObject()) {
    if (error) *error = QStringLiteral("invalid theme manifest: ") + parse_error.errorString();
    return false;
  }
  const auto root = document.object();
  const auto tokens = root.value(QStringLiteral("tokens")).toObject();
  manifest->id = root.value(QStringLiteral("id")).toString();
  manifest->name = root.value(QStringLiteral("name")).toString();
  manifest->tokens = {
      tokens.value(QStringLiteral("background")).toString(),
      tokens.value(QStringLiteral("surface")).toString(),
      tokens.value(QStringLiteral("text")).toString(),
      tokens.value(QStringLiteral("mutedText")).toString(),
      tokens.value(QStringLiteral("border")).toString(),
      tokens.value(QStringLiteral("accent")).toString(),
      tokens.value(QStringLiteral("success")).toString(),
      tokens.value(QStringLiteral("warning")).toString(),
      tokens.value(QStringLiteral("danger")).toString(),
  };
  const auto &value = manifest->tokens;
  if (manifest->id.isEmpty() || value.background.isEmpty() ||
      value.surface.isEmpty() || value.text.isEmpty() ||
      value.muted_text.isEmpty() || value.border.isEmpty() ||
      value.accent.isEmpty() || value.success.isEmpty() ||
      value.warning.isEmpty() || value.danger.isEmpty()) {
    if (error) *error = QStringLiteral("theme manifest is missing required tokens");
    return false;
  }
  return true;
}

bool UiThemeManager::Apply(QApplication *application, const QString &theme_root,
                           const QString &theme_id, const QString &density,
                           QString *error) const {
  const QString directory = theme_root + QLatin1Char('/') + theme_id;
  ThemeManifest manifest;
  if (!LoadManifest(directory + QStringLiteral("/theme.json"), &manifest, error))
    return false;
  if (manifest.id != theme_id) {
    if (error) *error = QStringLiteral("theme manifest id does not match directory");
    return false;
  }
  QFile stylesheet_file(directory + QStringLiteral("/theme.qss"));
  if (!stylesheet_file.open(QIODevice::ReadOnly)) {
    if (error) *error = QStringLiteral("cannot open theme stylesheet");
    return false;
  }
  QString stylesheet = QString::fromUtf8(stylesheet_file.readAll());
  const QMap<QString, QString> replacements = {
      {QStringLiteral("@background"), manifest.tokens.background},
      {QStringLiteral("@surface"), manifest.tokens.surface},
      {QStringLiteral("@text"), manifest.tokens.text},
      {QStringLiteral("@mutedText"), manifest.tokens.muted_text},
      {QStringLiteral("@border"), manifest.tokens.border},
      {QStringLiteral("@accent"), manifest.tokens.accent},
      {QStringLiteral("@success"), manifest.tokens.success},
      {QStringLiteral("@warning"), manifest.tokens.warning},
      {QStringLiteral("@danger"), manifest.tokens.danger},
  };
  for (auto it = replacements.cbegin(); it != replacements.cend(); ++it)
    stylesheet.replace(it.key(), it.value());
  const QString spacing = density == QStringLiteral("compact")
                              ? QStringLiteral("4px")
                              : QStringLiteral("8px");
  stylesheet.replace(QStringLiteral("@controlPadding"), spacing);
  application->setProperty("agtThemeId", theme_id);
  application->setProperty("agtUiDensity", density);
  application->setStyleSheet(stylesheet);
  return true;
}
