
#include "grid_fast_rviz/topometric_map_display.hpp"

#include <OgreBillboardSet.h>
#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTechnique.h>
#include <algorithm>
#include <cstddef>
#include <grid_fast_msgs/msg/detail/point2_d_int__struct.hpp>
#include <grid_fast_msgs/msg/detail/point2_d_int_list__struct.hpp>
#include <grid_fast_msgs/msg/detail/topo_polygon__struct.hpp>
#include <grid_fast_msgs/msg/detail/topometric_map__struct.hpp>
#include <nav_msgs/msg/detail/map_meta_data__struct.hpp>
#include <qobjectdefs.h>
#include <rviz_common/message_filter_display.hpp>

#include "rviz_common/display_context.hpp"
#include "rviz_common/frame_manager_iface.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/parse_color.hpp"
#include "rviz_common/validate_floats.hpp"
#include "rviz_rendering/material_manager.hpp"

#include <earcut.hpp>
#include <vector>

namespace grid_fast_rviz_plugin {

struct point {
  float x, y;
};

TopometricMapDisplay::TopometricMapDisplay() {
  global_alpha_property_ = new rviz_common::properties::FloatProperty(
      "Global Alpha", 1.0f,
      "Amount of transparency to apply to the topometric map.", this,
      SLOT(update_display()), this);
  global_alpha_property_->setMin(0);
  global_alpha_property_->setMax(1);
  show_polygons_property_ = new rviz_common::properties::BoolProperty(
      "Regions", true, "Toggle to show or hide topometric regions.", this,
      SLOT(update_display()), this);
  polygon_alpha_property_ = new rviz_common::properties::FloatProperty(
      "Region Alpha", 1.0f, "Amount of transparency to apply to the polygon.",
      show_polygons_property_, SLOT(update_display()), this);
  polygon_alpha_property_->setMin(0);
  polygon_alpha_property_->setMax(1);
  color_property_intersecion_ = new rviz_common::properties::ColorProperty(
      "Intersection Color", QColor(185, 155, 145),
      "Color to draw intersections.", show_polygons_property_,
      SLOT(update_display()), this);
  color_property_pathway_ = new rviz_common::properties::ColorProperty(
      "Pathway Color", QColor(130, 200, 130), "Color to draw pathways.",
      show_polygons_property_, SLOT(update_display()), this);
  color_property_dead_end_ = new rviz_common::properties::ColorProperty(
      "Dead End Color", QColor(210, 220, 240), "Color to draw dead ends.",
      show_polygons_property_, SLOT(update_display()), this);
  color_property_pathway_unexplored_ =
      new rviz_common::properties::ColorProperty(
          "Unexplored Color", QColor(240, 175, 220),
          "Color to draw pathways to unexplored areas.",
          show_polygons_property_, SLOT(update_display()), this);

  show_openings_property_ = new rviz_common::properties::BoolProperty(
      "Openings", true, "Toggle to show or hide openings.", this,
      SLOT(update_display()), this);
  opening_alpha_property_ = new rviz_common::properties::FloatProperty(
      "Opening Alpha", 1.0f, "Amount of transparency to apply to openings.",
      show_openings_property_, SLOT(update_display()), this);
  opening_alpha_property_->setMin(0);
  opening_alpha_property_->setMax(1);
  color_property_opening_ = new rviz_common::properties::ColorProperty(
      "Opening Color", QColor(205, 90, 0), "Color to draw openings.",
      show_openings_property_, SLOT(update_display()), this);
  opening_width_property_ = new rviz_common::properties::FloatProperty(
      "Width", 0.05f, "Width of openings.", show_openings_property_,
      SLOT(update_display()), this);
  opening_width_property_->setMin(0);
  show_opening_arrow_property_ = new rviz_common::properties::BoolProperty(
      "Show Arrow", true, "Toggle to show or hide opening arrow.",
      show_openings_property_, SLOT(update_display()), this);
  opening_arrow_width_property_ = new rviz_common::properties::FloatProperty(
      "Arrow Width", 0.05f, "Width of opening arrow.", show_openings_property_,
      SLOT(update_display()), this);
  opening_arrow_width_property_->setMin(0);

  show_paths_property_ = new rviz_common::properties::BoolProperty(
      "Paths", true, "Toggle to show or hide paths.", this,
      SLOT(update_display()), this);
  path_alpha_property_ = new rviz_common::properties::FloatProperty(
      "Opening Alpha", 1.0f, "Amount of transparency to apply to openings.",
      show_paths_property_, SLOT(update_display()), this);
  path_alpha_property_->setMin(0);
  path_alpha_property_->setMax(1);
  color_property_path_ = new rviz_common::properties::ColorProperty(
      "Path Color", QColor(185, 50, 45), "Color to draw global robot paths.",
      show_paths_property_, SLOT(update_display()), this);
  path_width_property_ = new rviz_common::properties::FloatProperty(
      "Width", 0.1f, "Width of opening arrow.", show_paths_property_,
      SLOT(update_display()), this);
  path_width_property_->setMin(0);
}

TopometricMapDisplay::~TopometricMapDisplay() {
  if (initialized()) {
    for (auto manual_object : manual_objects_)
      scene_manager_->destroyManualObject(manual_object);
    manual_objects_.clear();

    for (auto billbord_line : billboard_lines_)
      delete billbord_line;
    billboard_lines_.clear();
  }
}

void TopometricMapDisplay::onInitialize() { MFDClass::onInitialize(); }

void TopometricMapDisplay::reset() {
  MFDClass::reset();
  topo_map = grid_fast_msgs::msg::TopometricMap();
  for (auto manual_object : manual_objects_)
    manual_object->clear();
  for (auto billbord_line : billboard_lines_)
    billbord_line->clear();
}

point map_to_world(grid_fast_msgs::msg::Point2DInt position,
                   nav_msgs::msg::MapMetaData info) {
  float rez = info.resolution;
  return {(float(position.x)) * rez + float(info.origin.position.x) + rez / 2,
          (float(position.y)) * rez + float(info.origin.position.y) + rez / 2};
}

Ogre::MaterialPtr TopometricMapDisplay::get_material(unsigned int mat_index) {
  // allocate materials
  static uint32_t count = materials_.size();
  if (mat_index >= materials_.size()) {
    for (size_t i = materials_.size(); mat_index >= i; i++) {
      std::stringstream ss;
      ss << "PolygonArrayMaterial" << count++;
      Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create(
          ss.str(), "rviz_rendering");
      material->setReceiveShadows(false);
      material->getTechnique(0)->setLightingEnabled(false);
      material->getTechnique(0)->setAmbient(0.5, 0.5, 0.5);

      materials_.push_back(material);
    }
  }
  return materials_[mat_index];
}

void TopometricMapDisplay::hide_manual_objects() {
  for (auto manual_object : manual_objects_)
    manual_object->setVisible(false);
}

void TopometricMapDisplay::hide_billboard_lines() {
  for (auto billbord_line : billboard_lines_)
    billbord_line->clear();
}

Ogre::ManualObject *
TopometricMapDisplay::get_manual_object(unsigned int obj_index) {
  if (obj_index >= manual_objects_.size()) {
    Ogre::ManualObject *manual_object = scene_manager_->createManualObject();
    manual_object->setDynamic(true);
    scene_node_->attachObject(manual_object);
    manual_objects_.push_back(manual_object);
  }
  manual_objects_[obj_index]->setVisible(true);
  manual_objects_[obj_index]->clear();
  return manual_objects_[obj_index];
}

rviz_rendering::BillboardLine *
TopometricMapDisplay::get_billboard_line(unsigned int lin_index) {
  if (lin_index >= billboard_lines_.size()) {
    auto billboard_line =
        new rviz_rendering::BillboardLine(scene_manager_, scene_node_);
    billboard_lines_.push_back(billboard_line);
  }
  return billboard_lines_[lin_index];
}

void TopometricMapDisplay::processMessage(
    grid_fast_msgs::msg::TopometricMap::ConstSharedPtr msg) {
  topo_map = *msg;
  update_display();
}

void TopometricMapDisplay::update_display() {
  unsigned int num_poly = topo_map.polygons.size();
  if (num_poly == 0)
    return;

  rclcpp::Time msg_time(topo_map.header.stamp, RCL_ROS_TIME);
  if (!updateFrame(topo_map.header.frame_id, msg_time)) {
    setMissingTransformToFixedFrame(topo_map.header.frame_id);
    return;
  }
  setTransformOk();

  obj_count = 0;
  lin_count = 0;
  hide_manual_objects();
  hide_billboard_lines();

  if (show_polygons_property_->getBool()) {
    for (auto polygon : topo_map.polygons) {
      creat_polygon_object(polygon, topo_map.info);
    }
  }
  if (show_openings_property_->getBool()) {
    for (auto polygon : topo_map.polygons) {
      if (polygon.type / 10 !=
          0) // dont draw openings for intersections as this
             // will result in the same openings drawn twice
        creat_opening_objects(polygon, topo_map.info);
    }
  }

  if (show_paths_property_->getBool()) {
    for (auto polygon : topo_map.polygons) {
      creat_billboard_lines(polygon, topo_map.info);
    }
  }
}

void TopometricMapDisplay::creat_polygon_object(
    grid_fast_msgs::msg::TopoPolygon polygon, nav_msgs::msg::MapMetaData info) {
  Ogre::ColourValue color;
  if (polygon.type / 10 == 0) {
    color = rviz_common::properties::qtToOgre(
        color_property_intersecion_->getColor());
  } else if (polygon.type / 10 == 1) {
    color =
        rviz_common::properties::qtToOgre(color_property_pathway_->getColor());
  } else if (polygon.type / 10 == 2) {
    color =
        rviz_common::properties::qtToOgre(color_property_dead_end_->getColor());
  } else {
    color = rviz_common::properties::qtToOgre(
        color_property_pathway_unexplored_->getColor());
  }
  color.a =
      global_alpha_property_->getFloat() * polygon_alpha_property_->getFloat();
  auto material = get_material(obj_count);
  auto polygon_object = get_manual_object(obj_count);
  obj_count += 1;

  rviz_rendering::MaterialManager::enableAlphaBlending(material, color.a);

  size_t num_points = polygon.polygon_points.list.size();
  if (num_points == 0)
    return;
  std::vector<std::array<double, 2>> polygon2d;
  for (unsigned int i = 0; i < num_points; ++i) {
    point msg_point = map_to_world(polygon.polygon_points.list[i], info);

    polygon2d.push_back({msg_point.x, msg_point.y});
  }

  std::vector<std::vector<std::array<double, 2>>> polygon_data = {polygon2d};
  auto indices = mapbox::earcut<unsigned int>(polygon_data);

  polygon_object->begin(material->getName(),
                        Ogre::RenderOperation::OT_TRIANGLE_LIST,
                        "rviz_rendering");
  float z_height = info.origin.position.z;
  // Add positions
  for (const auto &p : polygon2d) {
    polygon_object->position(p[0], p[1], z_height);
    polygon_object->colour(color);
  }
  // Add indices from earcut
  for (auto idx : indices) {
    polygon_object->index(idx);
  }

  polygon_object->end();
}

void creat_opening(point start, point end, std::vector<point> &points,
                   std::vector<unsigned int> &indices, bool generate_arrow,
                   float width, float arrwo_width) {
  // convert the line of an opening to a rectangular polygon

  point side = {end.x - start.x, end.y - start.y};

  float l = sqrt(abs(side.x * side.x + side.y * side.y));
  // if (l < 0.01)
  //   break;
  point nSide = {side.x / l, side.y / l};
  point nNorm = {-nSide.y * width / 2, nSide.x * width / 2};
  points.resize(generate_arrow ? 7 : 4);
  points[0].x = start.x + nNorm.x;
  points[0].y = start.y + nNorm.y;

  points[1].x = start.x - nNorm.x;
  points[1].y = start.y - nNorm.y;

  points[2].x = end.x - nNorm.x;
  points[2].y = end.y - nNorm.y;

  points[3].x = end.x + nNorm.x;
  points[3].y = end.y + nNorm.y;

  if (generate_arrow) {
    points[4].x = start.x + side.x / 2 - nSide.x * arrwo_width + nNorm.x;
    points[4].y = start.y + side.y / 2 - nSide.y * arrwo_width + nNorm.y;

    points[5].x = start.x + side.x / 2 + 4 * nNorm.x;
    points[5].y = start.y + side.y / 2 + 4 * nNorm.y;

    points[6].x = start.x + side.x / 2 + nSide.x * arrwo_width + nNorm.x;
    points[6].y = start.y + side.y / 2 + nSide.y * arrwo_width + nNorm.y;
  }

  indices.resize(generate_arrow ? 9 : 6);
  indices[0] = 0;
  indices[1] = 1;
  indices[2] = 2;
  indices[3] = 0;
  indices[4] = 2;
  indices[5] = 3;

  if (generate_arrow) {
    indices[6] = 6;
    indices[7] = 5;
    indices[8] = 4;
  }
}

void TopometricMapDisplay::creat_opening_objects(
    grid_fast_msgs::msg::TopoPolygon polygon, nav_msgs::msg::MapMetaData info) {

  float z_height = info.origin.position.z + 0.005;
  for (auto opening : polygon.openings) {
    point start = map_to_world(opening.start, info);
    point end = map_to_world(opening.end, info);

    std::vector<point> poly_points;
    std::vector<unsigned int> indices;
    creat_opening(start, end, poly_points, indices,
                  show_opening_arrow_property_->getBool(),
                  opening_width_property_->getFloat(),
                  opening_arrow_width_property_->getFloat());

    Ogre::ColourValue color =
        rviz_common::properties::qtToOgre(color_property_opening_->getColor());
    color.a = global_alpha_property_->getFloat() *
              opening_alpha_property_->getFloat();
    auto material = get_material(obj_count);
    auto opening_object = get_manual_object(obj_count);
    obj_count += 1;

    rviz_rendering::MaterialManager::enableAlphaBlending(material, color.a);

    opening_object->begin(material->getName(),
                          Ogre::RenderOperation::OT_TRIANGLE_LIST,
                          "rviz_rendering");
    // Add positions
    for (const auto &p : poly_points) {
      opening_object->position(p.x, p.y, z_height);
      opening_object->colour(color);
    }
    for (auto idx : indices) {
      opening_object->index(idx);
    }

    opening_object->end();
  }
}

void TopometricMapDisplay::creat_billboard_lines(
    grid_fast_msgs::msg::TopoPolygon polygon, nav_msgs::msg::MapMetaData info) {
  float z_height = info.origin.position.z + 0.01;
  for (auto path : polygon.connected_paths) {
    auto billboard_line = get_billboard_line(lin_count);
    lin_count += 1;

    auto color = color_property_path_->getOgreColor();
    color.a =
        global_alpha_property_->getFloat() * path_alpha_property_->getFloat();

    billboard_line->setNumLines(1);
    billboard_line->setMaxPointsPerLine(
        static_cast<uint32_t>(path.list.size()));
    billboard_line->setLineWidth(path_width_property_->getFloat());

    for (auto p : path.list) {
      Ogre::Vector3 xpos(p.x, p.y, z_height);
      billboard_line->addPoint(xpos, color);
    }
  }
}

} // namespace grid_fast_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(grid_fast_rviz_plugin::TopometricMapDisplay,
                       rviz_common::Display)
