# Topology Mapping

## Installation

1. Install jsk_visualization package`sudo apt install ros-noetic-jsk-visualization`
2. Clone the GitHub  to ros workspace: `git clone https://github.com/LTU-RAI/topology_mapping.git`
3. Build project: `catkin build`

## ROS Launch Files

`topology_mapping.launch`: Use this launch file to launch Topology Mapping. Launches the `topology_gap_analysis_node` and`topology_mapping_node` nodes.

## ROS Nodes

### `topology_gap_analysis_node`

A node that performs gap analysis on an occupancy grid and filters the map to remove areas that can't be occupied by the robot. Outputs a filtered map and detects openings of intersections in grid coordinates.  

#### Subscribed Topics

`occupancy_map_global`: Input occupancy map.

#### Published Topics

`topology_map_filterd`: Filtered version of `occupancy_map_global`.

`opening_list_int`: List of detected openings in grid coordinates. 

### `topology_mapping_node`

A node that uses the output of the `topology_gap_analysis_node` node to detect intersections. Outputs polygons of intersections, rooms, pathways,  pathways leading to an unexplored map region, and openings out of intersections and rooms. 

#### Subscribed Topics

`topology_map_filterd`

`opening_list_int`

#### Published Topics

`topology_poly`: List of polygons containing: intersections, rooms, pathways and pathways leading to an unexplored map region.

`topology_poly_opening`: List of polygons containing openings leading out of intersections and rooms.
