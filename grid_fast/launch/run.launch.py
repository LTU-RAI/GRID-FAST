import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the directory of the grid_fast package
    grid_fast_share_dir = get_package_share_directory('grid_fast')

    # Path to the RViz configuration file
    rviz_config_file = os.path.join(grid_fast_share_dir, 'launch', 'include', 'grid_fast.rviz')

    # Node parameters
    node_params = {
        'map_frame': 'map',
        'polygon_downsampling': 1,
        'voronoi_downsampling': 4,
        'number_of_scanning_direction': 4,
        'min_gap_size': 3,
        'unknown_cells_filter': 1,
        'object_filter_max_steps': 10,
        'force_map_transform_update': False,
        'min_frontier_size': -1,
        'optimize_intersections': True,
        'opt_min_distans_to_center': 0.0,
    }

    # grid_fast_node
    grid_fast_node = Node(
        package='grid_fast',
        executable='grid_fast_node',
        name='grid_fast',
        output='screen',
        parameters=[node_params],
        respawn=True,
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        grid_fast_node,
        rviz_node,
    ])

