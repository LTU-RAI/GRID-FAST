#include "GapHandler.hh"
#include "MapHandler.hh"
#include "MapTransform.hh"
#include "OpeningHandler.hh"
#include "PolygonHandler.hh"
#include "Time.hpp"
#include "Utility.hh"
#include "datatypes.hh"
#include <algorithm>
#include <complex>
#include <grid_fast_msgs/msg/detail/point2_d_int__struct.hpp>
#include <grid_fast_msgs/msg/detail/point2_d_int_list__struct.hpp>
#include <memory>
#include <new>
#include <string>

// ROS2 includes
#include <grid_fast_msgs/msg/opening_list.hpp>
#include <grid_fast_msgs/msg/point2_d_list.hpp>
#include <grid_fast_msgs/msg/topometric_map.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class TopometricMapping : public rclcpp::Node {
private:
  // Subscribers and Publishers
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subOccupancyMap;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pubTopoMap;
  rclcpp::Publisher<grid_fast_msgs::msg::TopometricMap>::SharedPtr
      pubTopometricMap;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      pubPolyDebug;

  // Service
  // rclcpp::Service<grid_fast::srv::CustomOpening>::SharedPtr service;

  // Messages
  nav_msgs::msg::OccupancyGrid topoMapMsg;
  nav_msgs::msg::OccupancyGrid topoMapMsgD;

  // Pointers to handlers
  MapHandler *map, *mapDebug;
  MapTransform *transform;
  GapHandler *gaps;
  OpeningHandler *openingList;
  PolygonHandler *polygonList;

  // Other variables
  int oldNodCount = 0;
  std::vector<std::vector<double>> timeVector;
  Time tFromS;

  // Parameters
  int minGroupSize;
  int cfilterSize;
  int objectFilterMaxStep;
  int numberOfDir;
  bool forceUpdate;
  int minFrontier;
  bool optimizIntersections;
  double dw;
  int minimumDesendingSteps;
  double minDistToCenter;
  double maxPenalty;
  int polygonRez;
  bool polygonMerging;
  double polygonMergingDist;
  int voronoiRez;
  std::string mapFrame;
  bool show_removed_openings = false;

public:
  TopometricMapping() : Node("grid_fast") {
    // Declare and get parameters
    this->declare_parameter<std::string>("map_frame", "map");
    mapFrame = this->get_parameter("map_frame").as_string();

    this->declare_parameter<int>("min_gap_size", 3);
    minGroupSize = this->get_parameter("min_gap_size").as_int();

    this->declare_parameter<int>("unknown_cells_filter", 1);
    cfilterSize = this->get_parameter("unknown_cells_filter").as_int();

    this->declare_parameter<int>("object_filter_max_steps", 10);
    objectFilterMaxStep =
        this->get_parameter("object_filter_max_steps").as_int();

    this->declare_parameter<int>("number_of_scanning_direction", 4);
    numberOfDir = this->get_parameter("number_of_scanning_direction").as_int();

    this->declare_parameter<bool>("force_map_transform_update", false);
    forceUpdate = this->get_parameter("force_map_transform_update").as_bool();

    this->declare_parameter<int>("min_frontier_size", -1);
    minFrontier = this->get_parameter("min_frontier_size").as_int();

    this->declare_parameter<bool>("optimize_intersections", true);
    optimizIntersections =
        this->get_parameter("optimize_intersections").as_bool();

    this->declare_parameter<int>("minimum_desending_steps", 5.0);
    minimumDesendingSteps =
        this->get_parameter("minimum_desending_steps").as_int();

    this->declare_parameter<double>("dw", 1.0);
    dw = this->get_parameter("dw").as_double();

    this->declare_parameter<double>("opt_min_distans_to_center", 0.0);
    minDistToCenter =
        this->get_parameter("opt_min_distans_to_center").as_double();

    this->declare_parameter<double>("opt_max_penalty", 1e6);
    maxPenalty = this->get_parameter("opt_max_penalty").as_double();

    this->declare_parameter<int>("polygon_downsampling", 1);
    polygonRez = this->get_parameter("polygon_downsampling").as_int();

    this->declare_parameter<bool>("merge_polygons", false);
    polygonMerging = this->get_parameter("merge_polygons").as_bool();

    this->declare_parameter<double>("polygon_merge_dist", 0.0);
    polygonMergingDist = this->get_parameter("polygon_merge_dist").as_double();

    this->declare_parameter<int>("voronoi_downsampling", 4);
    voronoiRez = this->get_parameter("voronoi_downsampling").as_int();

    this->declare_parameter<bool>("show_removed_openings", false);
    show_removed_openings =
        this->get_parameter("show_removed_openings").as_bool();

    minGroupSize = std::max(minGroupSize, 2);
    cfilterSize = std::max(cfilterSize, 0);
    objectFilterMaxStep = std::max(objectFilterMaxStep, 0);
    numberOfDir = std::max(numberOfDir, 2);
    if (minFrontier < 0) {
      minFrontier = minGroupSize;
    }
    polygonRez = std::max(polygonRez, 1);
    polygonMergingDist = std::max(polygonMergingDist, 0.0);
    voronoiRez = std::max(voronoiRez, 4);

    // Setup subscribers and publishers
    subOccupancyMap = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 1,
        std::bind(&TopometricMapping::updateMap, this, std::placeholders::_1));

    pubTopoMap = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/grid_fast/map_filtered", 5);
    pubPolyDebug = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/grid_fast/polyDebug", 5);
    pubTopometricMap =
        this->create_publisher<grid_fast_msgs::msg::TopometricMap>(
            "/grid_fast/topometric_map", 5);

    // service = this->create_service<grid_fast::srv::CustomOpening>(
    //     "creat_opening",
    //     std::bind(&TopometricMapping::creatCustomOpenings, this,
    //     std::placeholders::_1, std::placeholders::_2));

    loadMemory();
  }

  void loadMemory() {
    timeVector.resize(4);
    map = new MapHandler;
    mapDebug = new MapHandler;
    transform = new MapTransform(numberOfDir);
    gaps = new GapHandler(numberOfDir, cfilterSize, minGroupSize);
    openingList =
        new OpeningHandler(numberOfDir, minGroupSize, minFrontier,
                           objectFilterMaxStep, show_removed_openings);
    polygonList = new PolygonHandler(minGroupSize, polygonRez, voronoiRez,
                                     minimumDesendingSteps, dw, minDistToCenter,
                                     maxPenalty, polygonMergingDist,
                                     optimizIntersections, polygonMerging);
  }

  grid_fast_msgs::msg::Point2DList ConvertPointListToMsg(robotPath *path,
                                                         MapHandler *map) {
    grid_fast_msgs::msg::Point2DList msg;
    msg.list.resize(path->size());
    double rez = map->getMapResolution();

    for (size_t i = 0; i < path->size(); i++) {
      point p = {
          static_cast<double>(path->at(i).x) * rez + map->getMapOffsetX(),
          static_cast<double>(path->at(i).y) * rez + map->getMapOffsetY()};
      msg.list[i].x = p.x;
      msg.list[i].y = p.y;
    }
    return msg;
  }

  /*bool creatCustomOpenings(*/
  /*    const std::shared_ptr<grid_fast::srv::CustomOpening::Request> req,*/
  /*    std::shared_ptr<grid_fast::srv::CustomOpening::Response> res) {*/
  /*  int label = -1;*/
  /*  switch (req->operation) {*/
  /*  case -1:*/
  /*    openingList->removeCustomOpening(req->id);*/
  /*    res->id = req->id;*/
  /*    return true;*/
  /*  case 1:*/
  /*    label = 6;*/
  /*    break;*/
  /*  case 2:*/
  /*    label = 8;*/
  /*    break;*/
  /*  default:*/
  /*    break;*/
  /*  }*/
  /*  int newId = openingList->maxCustomOpeningId() + 1;*/
  /*  opening newOp;*/
  /*  newOp.start.x = req->iOp.start.x;*/
  /*  newOp.start.y = req->iOp.start.y;*/
  /*  newOp.end.x = req->iOp.end.x;*/
  /*  newOp.end.y = req->iOp.end.y;*/
  /*  newOp.label = req->iOp.label;*/
  /*  openingList->addCustomOpening(newOp, newId);*/
  /**/
  /*  res->id = newId;*/
  /*  return true;*/
  /*}*/
  template <typename... Args>
  void log(const std::string &format, Args &&...args) {
    RCLCPP_INFO(this->get_logger(), format.c_str(),
                std::forward<Args>(args)...);
  }

  void updateMap(const nav_msgs::msg::OccupancyGrid::SharedPtr mapMsg) {
    std::vector<int> data;
    std::vector<int> bData;
    data.resize(mapMsg->data.size());
    bData.resize(mapMsg->data.size());
    for (size_t index = 0; index < mapMsg->data.size(); index++) {
      data[index] = mapMsg->data[index];
      bData[index] = -1;
    }
    map->updateMap(
        data, mapMsg->info.width, mapMsg->info.height, mapMsg->info.resolution,
        mapMsg->info.origin.position.x, mapMsg->info.origin.position.y,
        mapMsg->info.origin.position.z, mapMsg->info.origin.position.z);

    mapDebug->updateMap(
        bData, mapMsg->info.width, mapMsg->info.height, mapMsg->info.resolution,
        mapMsg->info.origin.position.x, mapMsg->info.origin.position.y,
        mapMsg->info.origin.position.z, mapMsg->info.origin.position.z);

    Time T;

    gaps->clear();
    openingList->clear();
    polygonList->clear();
    transform->updateTransform(map, forceUpdate);
    timeVector[0].insert(timeVector[0].begin(), T.get_since());

    gaps->analysis(map, transform);

    openingList->updateDetections(map, transform, gaps);

    openingList->update(map);

    polygonList->updateIntersections(openingList, map);
    timeVector[1].insert(timeVector[1].begin(), T.get_since());

    polygonList->optimize(openingList, map);

    polygonList->generatePolygonArea(openingList);
    timeVector[2].insert(timeVector[2].begin(), T.get_since());

    polygonList->generateRobotPath(openingList, map, mapDebug);

    timeVector[3].insert(timeVector[3].begin(), T.get_since());
    std::vector<double> Td;

    Td.resize(timeVector.size());
    for (size_t i1 = 0; i1 < timeVector.size(); i1++) {
      Td[i1] = 0;
      if (timeVector[i1].size() > 10)
        timeVector[i1].pop_back();
      for (size_t i2 = 0; i2 < timeVector[i1].size(); i2++) {
        Td[i1] += timeVector[i1][i2];
      }
      Td[i1] = timeVector[i1].back();
    }

    RCLCPP_INFO(this->get_logger(), "Time: %f, %f, %f, %f", Td[0], Td[1], Td[2],
                Td[3]);
    pubMap();
  }

  grid_fast_msgs::msg::Point2DIntList
  get_polygon_point_list(vector<point_int> list) {
    if (list.size() < 1)
      return grid_fast_msgs::msg::Point2DIntList();
    point_int first, last;
    grid_fast_msgs::msg::Point2DIntList poly_points;
    first = list[0];
    last = first;
    grid_fast_msgs::msg::Point2DInt p_to_add;
    p_to_add.x = first.x;
    p_to_add.y = first.y;
    poly_points.list.push_back(p_to_add);
    for (int i = 1; i < list.size(); i++) {
      point_int p = list[i];
      if (p == first)
        break;
      if (p == last)
        continue;
      last = p;
      p_to_add.x = p.x;
      p_to_add.y = p.y;
      poly_points.list.push_back(p_to_add);
    }
    return poly_points;
  }

  void pubMap() {
    // Initialize OpeningList message
    grid_fast_msgs::msg::OpeningList openingListMsg;

    // Set up topoMapMsg
    topoMapMsg.info.width = map->getMapSizeX();
    topoMapMsg.info.height = map->getMapSizeY();
    topoMapMsg.info.resolution = map->getMapResolution();
    topoMapMsg.info.origin.position.x = map->getMapOffsetX();
    topoMapMsg.info.origin.position.y = map->getMapOffsetY();
    topoMapMsg.info.origin.position.z =
        map->getMapHight() + 0.05; // For visualization
    topoMapMsg.data.resize(topoMapMsg.info.width * topoMapMsg.info.height);

    topoMapMsg.header.stamp = this->get_clock()->now();
    topoMapMsg.header.frame_id = mapFrame;

    // Populate topoMapMsg data
    for (int y = 0; y < map->getMapSizeY(); y++) {
      for (int x = 0; x < map->getMapSizeX(); x++) {
        int index = x + y * map->getMapSizeX();
        topoMapMsg.data[index] = map->getMap(x, y);
      }
    }
    pubTopoMap->publish(topoMapMsg);

    float resolution = map->getMapResolution();
    int mapOriginX = -map->getMapOffsetX() / resolution;
    int mapOriginY = -map->getMapOffsetY() / resolution;

    // Prepare polygon debug markers
    visualization_msgs::msg::MarkerArray polyDebug;
    polyDebug.markers.resize(1 + polygonList->size());
    polyDebug.markers[0].header.frame_id = mapFrame;
    polyDebug.markers[0].header.stamp = this->get_clock()->now();
    polyDebug.markers[0].action = visualization_msgs::msg::Marker::DELETEALL;

    for (size_t i = 1; i <= polygonList->size(); i++) {
      polygon *p = polygonList->get(i - 1);
      polyDebug.markers[i].header.frame_id = mapFrame;
      polyDebug.markers[i].header.stamp = this->get_clock()->now();
      polyDebug.markers[i].ns = "polyDebug";
      polyDebug.markers[i].id = i;
      polyDebug.markers[i].type =
          visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      polyDebug.markers[i].action = visualization_msgs::msg::Marker::ADD;

      polyDebug.markers[i].pose.position.x =
          (p->center.x - mapOriginX) * resolution;
      polyDebug.markers[i].pose.position.y =
          (p->center.y - mapOriginY) * resolution;
      polyDebug.markers[i].pose.position.z = map->getMapHight() + 0.05;
      polyDebug.markers[i].pose.orientation.w = 1.0;
      polyDebug.markers[i].scale.z = 0.5; // Text height
      polyDebug.markers[i].color.a = 1.0;
      polyDebug.markers[i].color.r = 0.0;
      polyDebug.markers[i].color.g = 0.0;
      polyDebug.markers[i].color.b = 0.0;
      polyDebug.markers[i].lifetime = rclcpp::Duration(0, 0);
      polyDebug.markers[i].text = std::to_string(i - 1);
    }
    pubPolyDebug->publish(polyDebug);

    polygonList->updateIndex();

    // Prepare topometric map message
    grid_fast_msgs::msg::TopometricMap topometricMapMsg;
    topometricMapMsg.header = topoMapMsg.header;
    topometricMapMsg.info = topoMapMsg.info;
    topoMapMsg.info.origin.position.z += 0.05;
    topometricMapMsg.polygon_type.resize(topoMapMsg.data.size(), -1);
    topometricMapMsg.polygon_id.resize(topoMapMsg.data.size(), -1);
    topometricMapMsg.polygons.resize(polygonList->size());

    for (size_t i = 0; i < polygonList->size(); i++) {
      // Fill polygon area
      std::vector<point_int> filledPoints =
          fillPoly(polygonList->get(i)->polygon_points);
      for (size_t j = 0; j < filledPoints.size(); j++) {
        int index = filledPoints[j].x + filledPoints[j].y * map->getMapSizeX();
        topometricMapMsg.polygon_type[index] = polygonList->get(i)->label;
        topometricMapMsg.polygon_id[index] = static_cast<int>(i);
      }

      // Prepare polygon message
      topometricMapMsg.polygons[i].id = static_cast<int>(i);
      topometricMapMsg.polygons[i].center.x = polygonList->get(i)->center.x;
      topometricMapMsg.polygons[i].center.y = polygonList->get(i)->center.y;
      topometricMapMsg.polygons[i].type = polygonList->get(i)->label;
      topometricMapMsg.polygons[i].polygon_points =
          get_polygon_point_list(polygonList->get(i)->polygon_points_desplay);

      // Connected paths
      size_t size = polygonList->get(i)->pathList.size();
      topometricMapMsg.polygons[i].connected_paths.resize(size);
      for (size_t n = 0; n < size; n++) {
        topometricMapMsg.polygons[i].connected_paths[n] =
            ConvertPointListToMsg(&polygonList->get(i)->pathList[n], map);
      }

      // Connected polygons
      size_t connSize = polygonList->get(i)->connectedpolygons.size();
      topometricMapMsg.polygons[i].connected_polygons.resize(connSize);
      for (size_t cIndex = 0; cIndex < connSize; cIndex++) {
        if (polygonList->get(i)->connectedpolygons[cIndex] == nullptr) {
          topometricMapMsg.polygons[i].connected_polygons[cIndex] = -1;
          continue;
        }
        topometricMapMsg.polygons[i].connected_polygons[cIndex] =
            polygonList->get(i)->connectedpolygons[cIndex]->index;
      }

      // Openings
      size_t openingsSize = polygonList->get(i)->openings.size();
      topometricMapMsg.polygons[i].openings.resize(openingsSize);
      for (size_t oIndex = 0; oIndex < openingsSize; oIndex++) {
        auto op = polygonList->get(i)->openings[oIndex];
        topometricMapMsg.polygons[i].openings[oIndex].start.x = op->start().x;
        topometricMapMsg.polygons[i].openings[oIndex].start.y = op->start().y;
        topometricMapMsg.polygons[i].openings[oIndex].end.x = op->end().x;
        topometricMapMsg.polygons[i].openings[oIndex].end.y = op->end().y;
        topometricMapMsg.polygons[i].openings[oIndex].label = op->label;
      }
    }
    pubTopometricMap->publish(topometricMapMsg);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<TopometricMapping>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
