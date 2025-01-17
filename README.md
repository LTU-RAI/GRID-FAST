# GRID-FAST

##### **Notice:** This ROS package is still under development and currently has some stability issues. We are working on improving both the implementation and the documentation.

This ROS package generates topometric maps by detecting intersections in 2D grid-based maps. The method identifies and classifies intersections, pathways, dead ends, and pathways leading to unexplored regions on the map, as well as the openings between them.

## Paper and Citation

This ROS package is based on the following article:

**GRID-FAST: A Grid-based Intersection Detection for Fast Semantic Topometric Mapping** [[JIRS](https://link.springer.com/article/10.1007/s10846-024-02180-6)] [[ArXiv](https://arxiv.org/abs/2406.11635)]

```
@Article{Fredriksson2024a,
  author     = {Fredriksson, Scott and Saradagi, Akshit and Nikolakopoulos, George},
  journal    = {Journal of Intelligent \& Robotic Systems},
  title      = {GRID-FAST: A Grid-based Intersection Detection for Fast Semantic Topometric Mapping},
  year       = {2024},
  issn       = {1573-0409},
  month      = oct,
  number     = {4},
  pages      = {154},
  volume     = {110},
  url        = {https://doi.org/10.1007/s10846-024-02180-6},
}

```

If you use this ROS package in a scientific publication, please cite the paper.

## Installation

1. Navigate to src folder in workspace `cd ~/catkin_ws/src`
2. Clone the GitHub  to ros workspace: `git clone https://github.com/LTU-RAI/GRID-FAST.git`
3. `cd ..`
4. Build project: `catkin build grid_fast`

## ROS Launch Files

`run.launch`: Use this launch file to launch the `grid_fast_node`.

## ROS Nodes

### `grid_fast_node`

#### Ros Parameters 

`map_frame` (default: `map`): Name of frame used for the output topometric map.

`number_of_scanning_directions` (default: `4`): The number of scanning directions the method uses to detect intersections. A higher value results in more accurate detections but increases computation time. Recommended values range from `3` to `8`.

`min_gap_size` (default: `3`): The minimum width, in map cells, that the robot can pass through. The smallest allowable value for this parameter is `2`.

`unknown_cells_filter` (default: `1`): The maximum width of a 'hole' (group of unknown cells) in free space that the method will remove. Holes wider than this value will not be removed.

`object_filter_max_steps` (default: `10`): Specifies the maximum allowable size (measured in cell circumference) of an object that this method will remove. Objects larger than this value will not be removed.

`force_map_transform_update` (default: `false`): This parameter controls the transform updates in the method. By default, the method caches the rotated maps and only updates the transform when the map size changes. Setting this parameter to `true` forces the method to update the transform every time a new map is generated. It is recommended to keep this parameter set to `false` in all cases except when benchmarking the method.

`min_frontier_size` (default: `-1`): Specifies the minimum number of cells for a frontier to be detected. Setting this to `-1` will make it automatically use the value of `min_gap_size`.

`optimize_intersections` (default: `true`): If set to `true`, this option improves the placement of intersections and removes unnecessary ones.

`opt_min_distance_to_center` (default: `0.0`): This parameter specifies the minimum distance, in cell lengths, from the center of the intersection at which the optimization will attempt to place openings. Note that this method does not guarantee that the opening will be placed any closer than the specified distance

`polygon_downsampling` (default: `1`): This parameter controls the downsampling of points in the polygons used for visualization. The number of points is reduced by a factor of `1/X`, where `X` is the value of this parameter. A value of `1` means no downsampling, while higher values result in greater downsampling (fewer points).

`voronoi_downsampling` (default: `4`): This parameter controls the reduction of points in the output paths of the skeleton map. The number of points is reduced by a factor of `1/X`, where `X` is the value of this parameter. A value of `1` means no downsampling, while higher values result in greater downsampling (fewer points).

#### Subscribed Topics 

`map` (type: `nav_msgs/OccupancyGrid`): Input 2D map. 

#### Published Topics

`grid_fast/topometric_map` (type: `grid_fast/topometricMap`): The output topometric map, which includes all regions, openings, paths, and connections within the topometric map.

`grid_fast/map_filtered` (type: `nav_msgs/OccupancyGrid`): A filtered version of the input 2D map used to generate the topometric map. The filter removes small objects, narrow openings too small for the robot, and holes of unknown space.

`grid_fast/regions` (type: `jsk_recognition_msgs/PolygonArray`): Polygons representing the regions in the topometric map. This topic is used for visualization in Rviz.

`grid_fast/regions` (type: `jsk_recognition_msgs/PolygonArray`): Polygons representing the regions in the topometric map. This topic is used for visualization in Rviz.

`grid_fast/robot_path` (type: `visualization_msgs/MarkerArray`): Markers representing the skeleton map generated by the method. This topic is used for visualization in RViz.

