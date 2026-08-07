#pragma once

#include <QWidget>

#include "msg/business_state.h"

class QLabel;
class QLineEdit;
class QTableWidget;
class AssetViewModel;

class ExperimentPage : public QWidget {
  Q_OBJECT

 public:
  explicit ExperimentPage(AssetViewModel *view_model,
                          QWidget *parent = nullptr);

 private slots:
  void updateBags(const basic::BusinessBagCatalog &catalog);

 private:
  AssetViewModel *view_model_{nullptr};
  QLineEdit *experiment_id_{nullptr};
  QLineEdit *title_{nullptr};
  QLineEdit *bag_id_{nullptr};
  QLineEdit *profile_id_{nullptr};
  QTableWidget *table_{nullptr};
  QLabel *message_{nullptr};
};
