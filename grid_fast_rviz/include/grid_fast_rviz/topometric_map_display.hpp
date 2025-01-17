#ifndef GRID_FAST_CUSTOM_DISPLAY_HPP_
#define GRID_FAST_CUSTOM_DISPLAY_HPP_

#include <grid_fast_msgs/msg/detail/topo_polygon__struct.hpp>
#include <grid_fast_msgs/msg/detail/topometric_map__struct.hpp>
#include <memory>

#include <grid_fast_msgs/msg/topometric_map.hpp>
#include <nav_msgs/msg/detail/map_meta_data__struct.hpp>
#include <qobjectdefs.h>

#include "rviz_common/message_filter_display.hpp"

#include "rviz_rendering/objects/billboard_line.hpp"

#include "rviz_default_plugins/visibility_control.hpp"

namespace Ogre {
class ManualObject;
}

namespace rviz_common {
namespace properties {
class ColorProperty;
class FloatProperty;
} // namespace properties
} // namespace rviz_common

namespace grid_fast_rviz_plugin {

class TopometricMapDisplay : public rviz_common::MessageFilterDisplay<
                                 grid_fast_msgs::msg::TopometricMap> {
  Q_OBJECT

private Q_SLOTS:
  void update_display();

public:
  TopometricMapDisplay();
  ~TopometricMapDisplay() override;

  void onInitialize() override;
  void reset() override;

protected:
  unsigned int obj_count = 0;
  unsigned int lin_count = 0;
  grid_fast_msgs::msg::TopometricMap topo_map;
  void processMessage(
      grid_fast_msgs::msg::TopometricMap::ConstSharedPtr msg) override;
  Ogre::MaterialPtr get_material(unsigned int mat_index);
  Ogre::ManualObject *get_manual_object(unsigned int obj_index);
  rviz_rendering::BillboardLine *get_billboard_line(unsigned int lin_index);
  void hide_manual_objects();
  void hide_billboard_lines();

  void creat_polygon_object(grid_fast_msgs::msg::TopoPolygon polygon,
                            nav_msgs::msg::MapMetaData info);
  void creat_opening_objects(grid_fast_msgs::msg::TopoPolygon polygon,
                             nav_msgs::msg::MapMetaData info);
  void creat_billboard_lines(grid_fast_msgs::msg::TopoPolygon polygon,
                             nav_msgs::msg::MapMetaData info);
  std::vector<Ogre::ManualObject *> manual_objects_;
  std::vector<rviz_rendering::BillboardLine *> billboard_lines_;
  std::vector<Ogre::MaterialPtr> materials_;
  rviz_common::properties::BoolProperty *show_polygons_property_,
      *show_openings_property_, *show_opening_arrow_property_,
      *show_paths_property_;
  rviz_common::properties::ColorProperty *color_property_intersecion_,
      *color_property_pathway_, *color_property_dead_end_,
      *color_property_pathway_unexplored_, *color_property_opening_,
      *color_property_path_;
  rviz_common::properties::FloatProperty *global_alpha_property_,
      *polygon_alpha_property_, *opening_alpha_property_, *path_alpha_property_,
      *opening_arrow_width_property_, *opening_width_property_,
      *path_width_property_;
};

} // namespace grid_fast_rviz_plugin

#endif
