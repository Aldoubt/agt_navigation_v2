#pragma once

#include <QWidget>

#include "msg/business_state.h"

class QLabel;
class QLineEdit;
class QProgressBar;
class QPushButton;
class MappingViewModel;

class MappingPage : public QWidget {
  Q_OBJECT

 public:
  explicit MappingPage(MappingViewModel *view_model,
                       QWidget *parent = nullptr);

 private slots:
  void updateStatus(const basic::BusinessMappingStatus &status);

 private:
  MappingViewModel *view_model_{nullptr};
  QLineEdit *map_id_{nullptr};
  QLabel *session_{nullptr};
  QLabel *state_{nullptr};
  QLabel *message_{nullptr};
  QProgressBar *progress_{nullptr};
  QPushButton *start_{nullptr};
  QPushButton *finalize_{nullptr};
  QPushButton *commit_{nullptr};
  QPushButton *discard_{nullptr};
};
