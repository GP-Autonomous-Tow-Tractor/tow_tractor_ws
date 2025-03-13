import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition

def generate_launch_description():


    ############################### GAZEBO-ROS CUMMUNICATION  ###############################     
    #########################################################################################

    ############# GZ-ROS-LAUNCH INITIAL SETUP #############
    pkg_project_bringup = os.path.join(get_package_share_directory("tow_tractor_bringup"))
    pkg_project_gazebo = os.path.join(get_package_share_directory("tow_tractor_gazebo"))
    pkg_project_description = os.path.join(get_package_share_directory("tow_tractor_description"))

    gz_sim_resource_path = f"{pkg_project_bringup}:{pkg_project_gazebo}:{pkg_project_description}"
    set_gz_sim_resource_path = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',gz_sim_resource_path)


    ################## GZ-ROS-BRIDGE NODE ##################

    node_ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_bringup, 'config', 'ros_gz_bridge_config.yaml'),
            # 'qos_overrides./tf_static.publisher.durability': 'transient_local',
            # "qos_overrides./scan.reliability": "best_effort",
            # "qos_overrides./scan.durability": "transient_local"
        }],
        output='screen'
    )

    ##################  VISUALIZE IN RVIZ ##################
    node_rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', os.path.join(pkg_project_bringup, 'config', 'stupid_robot.rviz')],
       condition=IfCondition(LaunchConfiguration('rviz'))
    )

    declare_rviz = DeclareLaunchArgument('rviz', default_value='true',
                        description='Open RViz.')
    #########################################################################################
    #########################################################################################




    ################################ Launching Gazebo World  ################################     
    #########################################################################################

    ############# GAZEBO SIM LAUNCH - WORLD #############
    launch_gazebo_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': [PathJoinSubstitution([pkg_project_gazebo, 'worlds/warehouse.sdf']),],
        }.items()
    )

    ############# GAZEBO SIM LAUNCH - MODEL #############
    node_spawn_urdf = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'stupid_robot',
            '-x', '9.34', '-y', '1.33', '-z', '0.1'
        ],
        output='screen'
    )
    #########################################################################################
    #########################################################################################




    ############################## Robot URDF Proccessing Unit ##############################
    #########################################################################################

    ############# ROBOT STATE PUBLISHER NODE #############
    # Path to URDF file
    robot_name = "stupid_robot"
    xacro_file = os.path.join(pkg_project_description,"urdf", robot_name, f'{robot_name}.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    # Create a robot_state_publisher node
    params=[
        {'use_sim_time': True},
        {'robot_description': robot_description_config.toxml()},
        ]
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=params,
    )
    #########################################################################################
    #########################################################################################




    return LaunchDescription([

        ################ PACKAGE SETUP ACTIONS ################
        set_gz_sim_resource_path,
        declare_rviz,

        ############# LAUNCH NODES AND LAUNCH FILES #############
        launch_gazebo_world,
        node_ros_gz_bridge,
        node_rviz,
        node_robot_state_publisher,
        node_spawn_urdf,
    ])
