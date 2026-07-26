#pragma once

#include <QPointF>
#include <vector>

#include "virtual_display.h"

namespace Display {

class DisplayRouteAnnotations : public VirtualDisplay {
 private:
  struct SceneAnnotation {
    QPointF position;
    double angle_degrees{0};
    std::string kind;
  };

  OccupancyMap map_data_;
  RouteAnnotations annotations_;
  std::vector<SceneAnnotation> scene_annotations_;
  bool initial_fit_done_{false};

  void rebuildSceneAnnotations();
  void drawDirection(QPainter *painter, const SceneAnnotation &annotation);
  void drawEvent(QPainter *painter, const SceneAnnotation &annotation);

 public:
  DisplayRouteAnnotations(const std::string &display_type, int z_value,
                          const std::string &parent_name);
  ~DisplayRouteAnnotations() override = default;
  void paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
             QWidget *widget = nullptr) override;
};

}  // namespace Display
