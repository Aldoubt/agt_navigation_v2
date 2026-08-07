#include "ui/view_models/business_operations_view_model.h"

#include <QMetaObject>

#include "core/framework/framework.h"
#include "msg/msg_info.h"
#include "ui_language.h"

MappingViewModel::MappingViewModel(bool enabled, QObject *parent)
    : QObject(parent), enabled_(enabled) {
  subscription_id_ = SUBSCRIBE(
      MSG_ID_BUSINESS_MAPPING_STATUS,
      [this](const basic::BusinessMappingStatus &status) {
        QMetaObject::invokeMethod(
            this,
            [this, status]() {
              if (!status.session_id.empty() || status_.session_id.empty())
                status_ = status;
              else {
                const auto session_id = status_.session_id;
                const auto map_id = status_.map_id;
                status_ = status;
                status_.session_id = session_id;
                status_.map_id = map_id;
              }
              emit statusChanged(status_);
            },
            Qt::QueuedConnection);
      });
}

MappingViewModel::~MappingViewModel() {
  if (subscription_id_ != 0)
    UNSUBSCRIBE(MSG_ID_BUSINESS_MAPPING_STATUS, subscription_id_);
}

void MappingViewModel::publish(basic::MappingCommand::Type type) {
  if (!enabled_) {
    emit requestRejected(UiLanguage::Text(
        "当前 profile 禁止管理建图会话。",
        "Mapping session control is disabled by the active profile."));
    return;
  }
  basic::MappingCommand command;
  command.type = type;
  command.session_id = status_.session_id;
  command.map_id = status_.map_id;
  PUBLISH(MSG_ID_MAPPING_COMMAND, command);
}

void MappingViewModel::refresh() {
  publish(basic::MappingCommand::Type::kStatus);
}

void MappingViewModel::start(const QString &map_id) {
  if (!enabled_) {
    publish(basic::MappingCommand::Type::kStart);
    return;
  }
  if (map_id.trimmed().isEmpty()) {
    emit requestRejected(
        UiLanguage::Text("地图 ID 不能为空。", "Map ID is required."));
    return;
  }
  basic::MappingCommand command;
  command.type = basic::MappingCommand::Type::kStart;
  command.map_id = map_id.trimmed().toStdString();
  PUBLISH(MSG_ID_MAPPING_COMMAND, command);
}

void MappingViewModel::finalize() {
  publish(basic::MappingCommand::Type::kFinalize);
}

void MappingViewModel::commit(bool activate) {
  if (!enabled_) {
    publish(basic::MappingCommand::Type::kCommit);
    return;
  }
  basic::MappingCommand command;
  command.type = basic::MappingCommand::Type::kCommit;
  command.session_id = status_.session_id;
  command.map_id = status_.map_id;
  command.activate_after_commit = activate;
  PUBLISH(MSG_ID_MAPPING_COMMAND, command);
}

void MappingViewModel::discard() {
  publish(basic::MappingCommand::Type::kDiscard);
}

RelocalizationViewModel::RelocalizationViewModel(bool enabled, QObject *parent)
    : QObject(parent), enabled_(enabled) {
  subscription_id_ = SUBSCRIBE(
      MSG_ID_BUSINESS_RELOCALIZATION_STATUS,
      [this](const basic::BusinessRelocalizationStatus &status) {
        QMetaObject::invokeMethod(
            this, [this, status]() { emit statusChanged(status); },
            Qt::QueuedConnection);
      });
}

RelocalizationViewModel::~RelocalizationViewModel() {
  if (subscription_id_ != 0)
    UNSUBSCRIBE(MSG_ID_BUSINESS_RELOCALIZATION_STATUS, subscription_id_);
}

void RelocalizationViewModel::startAutoSearch(int max_candidates,
                                               double timeout_s) {
  if (!enabled_) {
    emit requestRejected(UiLanguage::Text(
        "当前 profile 禁止重定位。",
        "Relocalization is disabled by the active profile."));
    return;
  }
  if (max_candidates <= 0 || timeout_s <= 0.0) {
    emit requestRejected(UiLanguage::Text(
        "候选数量和超时必须为正数。",
        "Candidate count and timeout must be positive."));
    return;
  }
  basic::RelocalizationCommand command;
  command.max_candidates = static_cast<std::uint32_t>(max_candidates);
  command.timeout_s = timeout_s;
  PUBLISH(MSG_ID_RELOCALIZATION_COMMAND, command);
}

AssetViewModel::AssetViewModel(bool map_enabled, bool bag_enabled,
                               QObject *parent)
    : QObject(parent),
      map_enabled_(map_enabled),
      bag_enabled_(bag_enabled) {
  map_subscription_id_ = SUBSCRIBE(
      MSG_ID_BUSINESS_MAP_CATALOG,
      [this](const basic::BusinessMapCatalog &catalog) {
        QMetaObject::invokeMethod(
            this, [this, catalog]() { emit mapsChanged(catalog); },
            Qt::QueuedConnection);
      });
  bag_subscription_id_ = SUBSCRIBE(
      MSG_ID_BUSINESS_BAG_CATALOG,
      [this](const basic::BusinessBagCatalog &catalog) {
        QMetaObject::invokeMethod(
            this, [this, catalog]() { emit bagsChanged(catalog); },
            Qt::QueuedConnection);
      });
}

AssetViewModel::~AssetViewModel() {
  if (map_subscription_id_ != 0)
    UNSUBSCRIBE(MSG_ID_BUSINESS_MAP_CATALOG, map_subscription_id_);
  if (bag_subscription_id_ != 0)
    UNSUBSCRIBE(MSG_ID_BUSINESS_BAG_CATALOG, bag_subscription_id_);
}

void AssetViewModel::refreshMaps(bool include_deleted) {
  if (!map_enabled_) {
    emit requestRejected(UiLanguage::Text(
        "当前 profile 禁止访问地图管理服务。",
        "Map manager access is disabled by the active profile."));
    return;
  }
  basic::MapCommand command;
  command.include_deleted = include_deleted;
  PUBLISH(MSG_ID_MAP_COMMAND, command);
}

void AssetViewModel::manageMap(int operation, const QString &map_version_id,
                               bool confirm_destructive) {
  if (!map_enabled_) {
    refreshMaps();
    return;
  }
  if (map_version_id.trimmed().isEmpty()) {
    emit requestRejected(UiLanguage::Text(
        "请先选择地图版本。", "Select a map version first."));
    return;
  }
  basic::MapCommand command;
  command.type = static_cast<basic::MapCommand::Type>(operation);
  command.map_version_id = map_version_id.trimmed().toStdString();
  command.confirm_destructive = confirm_destructive;
  PUBLISH(MSG_ID_MAP_COMMAND, command);
}

void AssetViewModel::refreshBags() {
  if (!bag_enabled_) {
    emit requestRejected(UiLanguage::Text(
        "当前 profile 禁止访问 Bag 管理服务。",
        "Bag manager access is disabled by the active profile."));
    return;
  }
  PUBLISH(MSG_ID_BAG_COMMAND, basic::BagCommand{});
}

void AssetViewModel::manageBag(int operation, const QString &bag_id,
                               const QString &experiment_id,
                               const QString &title,
                               const QString &profile_id,
                               double playback_rate) {
  if (!bag_enabled_) {
    refreshBags();
    return;
  }
  basic::BagCommand command;
  command.type = static_cast<basic::BagCommand::Type>(operation);
  command.bag_id = bag_id.trimmed().toStdString();
  command.experiment_id = experiment_id.trimmed().toStdString();
  command.experiment_title = title.trimmed().toStdString();
  command.profile_id = profile_id.trimmed().toStdString();
  command.playback_rate = playback_rate;
  PUBLISH(MSG_ID_BAG_COMMAND, command);
}
