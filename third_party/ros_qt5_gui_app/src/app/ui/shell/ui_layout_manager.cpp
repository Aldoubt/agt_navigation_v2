#include "ui/shell/ui_layout_manager.h"

UiLayoutManager::UiLayoutManager(const QString &requested_layout) {
  layout_id_ = requested_layout == QStringLiteral("legacy")
                   ? QStringLiteral("legacy")
                   : QStringLiteral("control-center-v1");
}
