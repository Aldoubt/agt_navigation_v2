#pragma once

#include <QObject>
#include <QMetaType>
#include <QString>

#include "msg/business_state.h"

Q_DECLARE_METATYPE(basic::BusinessMappingStatus)
Q_DECLARE_METATYPE(basic::BusinessRelocalizationStatus)
Q_DECLARE_METATYPE(basic::BusinessMapCatalog)
Q_DECLARE_METATYPE(basic::BusinessBagCatalog)

class MappingViewModel : public QObject {
  Q_OBJECT

 public:
  explicit MappingViewModel(bool enabled, QObject *parent = nullptr);
  ~MappingViewModel() override;

 public slots:
  void refresh();
  void start(const QString &map_id);
  void finalize();
  void commit(bool activate);
  void discard();

 signals:
  void statusChanged(const basic::BusinessMappingStatus &status);
  void requestRejected(const QString &message);

 private:
  void publish(basic::MappingCommand::Type type);
  bool enabled_{false};
  basic::BusinessMappingStatus status_;
  std::size_t subscription_id_{0};
};

class RelocalizationViewModel : public QObject {
  Q_OBJECT

 public:
  explicit RelocalizationViewModel(bool enabled, QObject *parent = nullptr);
  ~RelocalizationViewModel() override;

 public slots:
  void startAutoSearch(int max_candidates, double timeout_s);

 signals:
  void statusChanged(const basic::BusinessRelocalizationStatus &status);
  void requestRejected(const QString &message);

 private:
  bool enabled_{false};
  std::size_t subscription_id_{0};
};

class AssetViewModel : public QObject {
  Q_OBJECT

 public:
  AssetViewModel(bool map_enabled, bool bag_enabled,
                 QObject *parent = nullptr);
  ~AssetViewModel() override;
  bool mapEnabled() const { return map_enabled_; }
  bool bagEnabled() const { return bag_enabled_; }

 public slots:
  void refreshMaps(bool include_deleted = false);
  void manageMap(int operation, const QString &map_version_id,
                 bool confirm_destructive = false);
  void refreshBags();
  void manageBag(int operation, const QString &bag_id,
                 const QString &experiment_id, const QString &title,
                 const QString &profile_id, double playback_rate = 1.0);

 signals:
  void mapsChanged(const basic::BusinessMapCatalog &catalog);
  void bagsChanged(const basic::BusinessBagCatalog &catalog);
  void requestRejected(const QString &message);

 private:
  bool map_enabled_{false};
  bool bag_enabled_{false};
  std::size_t map_subscription_id_{0};
  std::size_t bag_subscription_id_{0};
};
