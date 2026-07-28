#pragma once

#include <QString>

class UiLayoutManager {
 public:
  explicit UiLayoutManager(const QString &requested_layout);
  QString layoutId() const { return layout_id_; }
  bool usesControlCenter() const { return layout_id_ == QStringLiteral("control-center-v1"); }

 private:
  QString layout_id_;
};
