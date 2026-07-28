#pragma once

#include <QString>
#include <QVector>

#include "ui/ui_capability_policy.h"

struct UiPageDefinition {
  QString id;
  QString label;
};

class UiPageRegistry {
 public:
  explicit UiPageRegistry(const UiCapabilityPolicy &capabilities);
  const QVector<UiPageDefinition> &pages() const { return pages_; }

 private:
  QVector<UiPageDefinition> pages_;
};
