#include "ui/ui_page_registry.h"

#include "ui_language.h"

UiPageRegistry::UiPageRegistry(const UiCapabilityPolicy &capabilities) {
  const auto add = [this, &capabilities](const char *id, const QString &label) {
    if (capabilities.pageEnabled(id)) pages_.push_back({QString::fromLatin1(id), label});
  };
  add("overview", UiLanguage::Text("总览", "Overview"));
  add("platform", UiLanguage::Text("机器人平台", "Platform"));
  add("mapping", UiLanguage::Text("建图", "Mapping"));
  add("teach_tuning", UiLanguage::Text("示教与调参", "Teach & tuning"));
  add("navigation_mission", UiLanguage::Text("导航与任务", "Navigation & missions"));
  add("map_task", UiLanguage::Text("地图与任务资产", "Map & task assets"));
  add("diagnostics", UiLanguage::Text("诊断", "Diagnostics"));
}
