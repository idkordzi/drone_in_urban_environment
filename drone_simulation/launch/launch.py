import os

from ament_index_python.packages import get_package_share_directory

from ros_gz_bridge.actions import RosGzBridge

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    
  drone_navigation_ros_pkg_path = get_package_share_directory('drone_navigation_ros')
  drone_simulation_ros_pkg_path = get_package_share_directory('drone_simulation')
  ros_gz_sim_pkg_path           = get_package_share_directory('ros_gz_sim')
  
  nav_config_dir = os.path.join(drone_navigation_ros_pkg_path, 'config')
  sim_config_dir = os.path.join(drone_simulation_ros_pkg_path, 'config')

  world_file_path = os.path.join(drone_simulation_ros_pkg_path, 'worlds', 'world_test.sdf')

  gzserver_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(ros_gz_sim_pkg_path, 'launch', 'gz_sim.launch.py')
    ),
    launch_arguments={'gz_args': ['-s ', world_file_path], 'on_exit_shutdown': 'true'}.items()
  )
  gzclient_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(ros_gz_sim_pkg_path, 'launch', 'gz_sim.launch.py')
    ),
    launch_arguments={'gz_args': '-g '}.items()
  )
  
  ros_gz_bridge = RosGzBridge(
    bridge_name='ros_gz_bridge',
    config_file=os.path.join(sim_config_dir, 'config_gazebo_bridge.yaml'),
  )
  
  node_vision = Node(
    package='drone_navigation_ros',
    namespace='',
    executable='drone_vision_node',
    name='drone_vision_node',
    parameters=[os.path.join(nav_config_dir, 'config_drone_vision.yaml')]
  )
  
  node_planner = Node(
    package='drone_navigation_ros',
    namespace='',
    executable='drone_planner_node',
    name='drone_planner_node',
    parameters=[os.path.join(nav_config_dir, 'config_drone_planner.yaml')]
  )
  
  node_flight_controller = Node(
    package='drone_navigation_ros',
    namespace='',
    executable='drone_flight_controller_node',
    name='drone_flight_controller_node',
    parameters=[os.path.join(nav_config_dir, 'config_flight_controller.yaml')]
  )
  
  # node_drone_test = Node(
  #   package='drone_navigation_ros',
  #   namespace='',
  #   executable='drone_test_node',
  #   name='drone_test_node'
  # )

  ld = LaunchDescription()

  ld.add_action(gzserver_cmd)
  ld.add_action(gzclient_cmd)
  ld.add_action(ros_gz_bridge)
  ld.add_action(node_vision)
  ld.add_action(node_planner)
  ld.add_action(node_flight_controller)

  return ld