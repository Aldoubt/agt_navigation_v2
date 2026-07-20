#pragma once

#include <QString>

#include "config/config_manager.h"

namespace UiLanguage {

inline bool IsChinese() {
  return GET_CONFIG_VALUE("UiLanguage", "zh_CN") != "en_US";
}

inline QString Text(const char *chinese, const char *english) {
  return QString::fromUtf8(IsChinese() ? chinese : english);
}

}  // namespace UiLanguage
