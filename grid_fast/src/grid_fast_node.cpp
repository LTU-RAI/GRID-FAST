#include "GapHandler.hh"
#include "MapHandler.hh"
#include "MapTransform.hh"
#include "OpeningHandler.hh"
#include "PolygonHandler.hh"
#include "Time.hpp"
#include "Utility.hh"
#include <algorithm>
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

// #include "grid_fast_msgs/srv/custom_opening.hpp"
// #include "jsk_recognition_msgs/msg/polygon_array.hpp"
class TopometricMapping : public rclcpp::Node {
private:
  // Subscribers and Publishers
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subOccupancyMap;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pubTopoMap;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pubdMap;
  rclcpp::Publisher<grid_fast_msgs::msg::OpeningList>::SharedPtr pubOpeningList;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pubMapDebug;
  // rclcpp::Publisher<jsk_recognition_msgs::msg::PolygonArray>::SharedPtr
  // pubTopoPolyDebug;
  // rclcpp::Publisher<jsk_recognition_msgs::msg::PolygonArray>::SharedPtr
  // pubTopoPoly;
  rclcpp::Publisher<grid_fast_msgs::msg::TopometricMap>::SharedPtr
      pubTopometricMap;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      pubRobotPath;
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

    this->declare_parameter<int>("voronoi_downsampling", 1);
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
    voronoiRez = std::max(voronoiRez, 1);

    // Setup subscribers and publishers
    subOccupancyMap = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 1,
        std::bind(&TopometricMapping::updateMap, this, std::placeholders::_1));

    pubTopoMap = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/grid_fast/map_filtered", 5);
    pubdMap = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/grid_fast/map_d", 5);
    pubOpeningList = this->create_publisher<grid_fast_msgs::msg::OpeningList>(
        "/grid_fast/opening_list_int", 5);
    pubMapDebug = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/grid_fast/map_debug", 5);
    // pubTopoPolyDebug =
    // this->create_publisher<jsk_recognition_msgs::msg::PolygonArray>("/grid_fast/openings",
    // 5);
    pubRobotPath = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/grid_fast/robot_path", 5);
    pubPolyDebug = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/grid_fast/polyDebug", 5);
    // pubTopoPoly =
    // this->create_publisher<jsk_recognition_msgs::msg::PolygonArray>("/grid_fast/regions",
    // 5);
    pubTopometricMap =
        this->create_publisher<grid_fast_msgs::msg::TopometricMap>(
            "/grid_fast/map", 5);

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

    // Prepare opening list for publishing
    std::vector<opening> pubOpList;
    for (int i = 0; i < openingList->size(); i++) {
      opening op = openingList->get(i)->getOpening();
      pubOpList.push_back(op);
    }
    pubOpList.insert(pubOpList.end(), openingList->openingDebug.begin(),
                     openingList->openingDebug.end());

    // Prepare debug map data
    std::vector<std::vector<int>> dMap(
        mapDebug->getMapSizeX(), std::vector<int>(mapDebug->getMapSizeY()));
    for (int x = 0; x < mapDebug->getMapSizeX(); x++) {
      for (int y = 0; y < mapDebug->getMapSizeY(); y++) {
        dMap[x][y] = mapDebug->getMap(x, y);
      }
    }

    // Prepare PolygonArray message for openings
    /*jsk_recognition_msgs::msg::PolygonArray pubPolyArrayOld;*/
    /*pubPolyArrayOld.header.frame_id = mapFrame;*/
    /*pubPolyArrayOld.header.stamp = this->get_clock()->now();*/
    /*pubPolyArrayOld.polygons.resize(pubOpList.size());*/
    /*pubPolyArrayOld.labels.resize(pubOpList.size());*/
    /*pubPolyArrayOld.likelihood.resize(pubOpList.size());*/
    /**/
    float resolution = map->getMapResolution();
    int mapOriginX = -map->getMapOffsetX() / resolution;
    int mapOriginY = -map->getMapOffsetY() / resolution;
    /**/
    /*for (size_t i = 0; i < pubOpList.size(); i++) {*/
    /*  double zHeight = map->getMapOffsetZ() + 0.11;*/
    /*  opening op = pubOpList[i];*/
    /*  for (size_t j = 0; j < op.occupiedPoints.size(); j++) {*/
    /*    dMap[op.occupiedPoints[j].x][op.occupiedPoints[j].y] = 10;*/
    /*  }*/
    /*  dMap[op.start.x][op.start.y] = 100;*/
    /*  dMap[op.end.x][op.end.y] = 50;*/
    /**/
    /*  point side = {static_cast<double>(op.end.x - op.start.x) * resolution,*/
    /*                static_cast<double>(op.end.y - op.start.y) *
     * resolution};*/
    /**/
    /*  double l = sqrt(std::abs(side.x * side.x + side.y * side.y));*/
    /*  if (l < 0.01)*/
    /*    continue;*/
    /*  point nSide = {side.x / (16 * l), side.y / (16 * l)};*/
    /*  point nNorm = {-nSide.y, nSide.x};*/
    /**/
    /*  geometry_msgs::msg::PolygonStamped p;*/
    /*  p.header.frame_id = mapFrame;*/
    /*  p.header.stamp = this->get_clock()->now();*/
    /*  p.polygon.points.resize(7);*/
    /**/
    /*  p.polygon.points[0].x = (op.start.x - mapOriginX) * resolution;*/
    /*  p.polygon.points[0].y = (op.start.y - mapOriginY) * resolution;*/
    /*  p.polygon.points[0].z = zHeight;*/
    /**/
    /*  p.polygon.points[1].x = (op.start.x - mapOriginX) * resolution + side.x
     * / 2 - nSide.x;*/
    /*  p.polygon.points[1].y = (op.start.y - mapOriginY) * resolution + side.y
     * / 2 - nSide.y;*/
    /*  p.polygon.points[1].z = zHeight;*/
    /**/
    /*  p.polygon.points[2].x = (op.start.x - mapOriginX) * resolution + side.x
     * / 2 + 2 * nNorm.x;*/
    /*  p.polygon.points[2].y = (op.start.y - mapOriginY) * resolution + side.y
     * / 2 + 2 * nNorm.y;*/
    /*  p.polygon.points[2].z = zHeight;*/
    /**/
    /*  p.polygon.points[3].x = (op.start.x - mapOriginX) * resolution + side.x
     * / 2 + nSide.x;*/
    /*  p.polygon.points[3].y = (op.start.y - mapOriginY) * resolution + side.y
     * / 2 + nSide.y;*/
    /*  p.polygon.points[3].z = zHeight;*/
    /**/
    /*  p.polygon.points[4].x = (op.end.x - mapOriginX) * resolution;*/
    /*  p.polygon.points[4].y = (op.end.y - mapOriginY) * resolution;*/
    /*  p.polygon.points[4].z = zHeight;*/
    /**/
    /*  p.polygon.points[5].x = (op.end.x - mapOriginX) * resolution -
     * nNorm.x;*/
    /*  p.polygon.points[5].y = (op.end.y - mapOriginY) * resolution -
     * nNorm.y;*/
    /*  p.polygon.points[5].z = zHeight;*/
    /**/
    /*  p.polygon.points[6].x = (op.start.x - mapOriginX) * resolution -
     * nNorm.x;*/
    /*  p.polygon.points[6].y = (op.start.y - mapOriginY) * resolution -
     * nNorm.y;*/
    /*  p.polygon.points[6].z = zHeight;*/
    /**/
    /*  pubPolyArrayOld.polygons[i] = p;*/
    /*  pubPolyArrayOld.labels[i] = op.label;*/
    /*  pubPolyArrayOld.likelihood[i] = 1.0;*/
    /*}*/

    // Update topoMapMsg for debug map
    for (int y = 0; y < map->getMapSizeY(); y++) {
      for (int x = 0; x < map->getMapSizeX(); x++) {
        int index = x + y * map->getMapSizeX();
        topoMapMsg.data[index] = dMap[x][y];
      }
    }
    pubdMap->publish(topoMapMsg);
    /*pubTopoPolyDebug->publish(pubPolyArrayOld);*/

    // Prepare PolygonArray message for regions
    /*jsk_recognition_msgs::msg::PolygonArray pubPolyArray;*/
    /*pubPolyArray.header.frame_id = mapFrame;*/
    /*pubPolyArray.header.stamp = this->get_clock()->now();*/
    /*pubPolyArray.polygons.resize(polygonList->size());*/
    /*pubPolyArray.labels.resize(polygonList->size());*/
    /*pubPolyArray.likelihood.resize(polygonList->size());*/
    /**/
    /*int c = 0;*/
    /*for (size_t i = 0; i < polygonList->size(); i++) {*/
    /*  polygon p = *polygonList->get(i);*/
    /*  pubPolyArray.polygons[i] = get_polygon(p, resolution,
     * map->getMapOffsetX(),*/
    /*                                         map->getMapOffsetY(),
     * map->getMapOffsetZ() + 0.1);*/
    /*  pubPolyArray.labels[i] = p.label;*/
    /*  pubPolyArray.likelihood[i] = 1.0;*/
    /*  if (p.label == 30 || p.label == 41 || p.label == 76 || p.label == 52)*/
    /*    c++;*/
    /*}*/
    /**/
    /*if (c != oldNodCount) {*/
    /*  oldNodCount = c;*/
    /*  RCLCPP_INFO(this->get_logger(), "Node count: %d", c);*/
    /*}*/
    /**/
    /*pubTopoPoly->publish(pubPolyArray);*/

    // Prepare robot paths
    std::vector<robotPath> robotPathList;
    for (size_t i = 0; i < polygonList->size(); i++) {
      polygon *p = polygonList->get(i);
      for (size_t pathI = 0; pathI < p->pathList.size(); pathI++) {
        robotPathList.push_back(p->pathList[pathI]);
      }
    }

    visualization_msgs::msg::MarkerArray msgRobotPath;
    msgRobotPath.markers.resize(1 + robotPathList.size());

    // Delete all previous markers
    msgRobotPath.markers[0].header.frame_id = mapFrame;
    msgRobotPath.markers[0].header.stamp = this->get_clock()->now();
    msgRobotPath.markers[0].action = visualization_msgs::msg::Marker::DELETEALL;

    for (size_t i = 1; i <= robotPathList.size(); i++) {
      msgRobotPath.markers[i].header.frame_id = mapFrame;
      msgRobotPath.markers[i].header.stamp = this->get_clock()->now();
      msgRobotPath.markers[i].ns = "robotPathList";
      msgRobotPath.markers[i].id = i;
      msgRobotPath.markers[i].type =
          visualization_msgs::msg::Marker::LINE_STRIP;
      msgRobotPath.markers[i].action = visualization_msgs::msg::Marker::ADD;

      msgRobotPath.markers[i].pose.position.x = 0;
      msgRobotPath.markers[i].pose.position.y = 0;
      msgRobotPath.markers[i].pose.position.z = map->getMapHight() + 0.05;
      msgRobotPath.markers[i].pose.orientation.w = 1.0;
      msgRobotPath.markers[i].scale.x = 0.1;
      msgRobotPath.markers[i].scale.y = 0.1;
      msgRobotPath.markers[i].scale.z = 0.1;
      msgRobotPath.markers[i].color.a = 1.0;
      msgRobotPath.markers[i].color.r = 1.0;
      msgRobotPath.markers[i].color.g = 0.0;
      msgRobotPath.markers[i].color.b = 0.0;
      msgRobotPath.markers[i].lifetime = rclcpp::Duration(0, 0);

      size_t pathIndex = i - 1;
      msgRobotPath.markers[i].points.resize(robotPathList[pathIndex].size());
      for (size_t m = 0; m < robotPathList[pathIndex].size(); m++) {
        msgRobotPath.markers[i].points[m].x =
            (robotPathList[pathIndex][m].x - mapOriginX) * resolution;
        msgRobotPath.markers[i].points[m].y =
            (robotPathList[pathIndex][m].y - mapOriginY) * resolution;
        msgRobotPath.markers[i].points[m].z = 0;
      }
    }
    pubRobotPath->publish(msgRobotPath);

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

    pubMapDebug->publish(topoMapMsg);
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
