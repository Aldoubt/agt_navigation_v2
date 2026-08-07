#pragma once

#include <QWidget>

#include "msg/business_state.h"

class QLabel;
class QLineEdit;
class QPushButton;
class QProgressBar;
class MissionViewModel;
class RelocalizationViewModel;

class NavigationMissionPage : public QWidget {
  Q_OBJECT

 public:
  NavigationMissionPage(MissionViewModel *view_model,
                                 RelocalizationViewModel *relocalization,
                                 QWidget *parent = nullptr);

 private slots:
  void updateStatus(const basic::BusinessMissionStatus &status);
  void updateRelocalization(
      const basic::BusinessRelocalizationStatus &status);

 private:
  MissionViewModel *view_model_{nullptr};
  QLineEdit *mission_id_{nullptr};
  QLineEdit *mission_version_{nullptr};
  QLineEdit *content_hash_{nullptr};
  QLabel *state_{nullptr};
  QLabel *message_{nullptr};
  QProgressBar *progress_{nullptr};
  QPushButton *execute_{nullptr};
  QPushButton *pause_{nullptr};
  QPushButton *resume_{nullptr};
  QPushButton *cancel_{nullptr};
  QLabel *relocalization_status_{nullptr};
};
