#include <QDebug>
#include <QHeaderView>
#include <QPainter>
#include <QStandardItemModel>
#include <QTableView>
#include "config/task_chain.h"
#include "map/topology_map.h"
using namespace basic;
class NavGoalTableView : public QTableView {
  Q_OBJECT
 public:
  explicit NavGoalTableView(QWidget *_parent_widget = nullptr);
  ~NavGoalTableView() override;

 private:
  QStandardItemModel *table_model_;
  TopologyMap topologyMap_;
  bool is_task_chain_running_{false};
  TaskChain task_chain_;
  int active_row_{-1};
 public slots:
  void UpdateTopologyMap(const TopologyMap &_topology_map);
  void AddItem();
  void UpdateSelectPoint(const TopologyMap::PointInfo &);
  void StartTaskChain(bool is_loop);
  void StopTaskChain();
  void UpdateTaskExecutionStatus(const TaskExecutionStatus &status);
  bool LoadTaskChain(const std::string &name);
  bool SaveTaskChain(const std::string &name);
 signals:
  void signalSendNavGoal(const RobotPose &pose);
  void signalExecuteTaskChain(const TaskExecutionRequest &request);
  void signalCancelTaskChain();
  void signalTaskFinish();

 private:
  void CreateRow(const QString &point_name = QString());
  void RefreshPointChoices();
  int RowForWidget(const QWidget *widget) const;
  int ActiveRow() const;
  void onItemChanged(QStandardItem *item);
};
