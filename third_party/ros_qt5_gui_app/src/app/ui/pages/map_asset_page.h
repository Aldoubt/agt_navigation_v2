#pragma once

#include <QWidget>

#include "msg/business_state.h"

class QLabel;
class QTableWidget;
class AssetViewModel;

class MapAssetPage : public QWidget {
  Q_OBJECT

 public:
  explicit MapAssetPage(AssetViewModel *view_model,
                        QWidget *parent = nullptr);

 private slots:
  void updateMaps(const basic::BusinessMapCatalog &catalog);

 private:
  QString selectedVersion() const;
  void runOperation(basic::MapCommand::Type type, bool destructive = false);
  AssetViewModel *view_model_{nullptr};
  QTableWidget *table_{nullptr};
  QLabel *message_{nullptr};
};
