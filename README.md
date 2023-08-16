# GRID-FAST

## Installation

1. Install jsk_visualization package`sudo apt install ros-noetic-jsk-visualization`
2. Navigate to src folder in workspace `cd ~/ros_ws/src`
3. Clone the GitHub  to ros workspace: `git clone https://github.com/LTU-RAI/GRID-FAST.git`
4. `cd ..`
5. Build project: `catkin build grid_fast`

## ROS Launch Files

`run.launch`: Use this launch file to launch the`grid_fast_node`.

## ROS Nodes

### `grid_fast_node`

A node that performs gap analysis on an occupancy grid and filters the map to remove areas that can't be occupied by the robot. Outputs a filtered map and detects openings of intersections in grid coordinates.  

#### Subscribed Topics

`map`: Input occupancy map.

#### Published Topics

`topology_map_filterd`: Filtered version of `occupancy_map_global`.

`opening_list_int`: List of detected openings in grid coordinates. 

`topology_poly`: List of polygons containing: intersections, rooms, pathways and pathways leading to an unexplored map region.

`topology_poly_opening`: List of polygons containing openings leading out of intersections and rooms.
