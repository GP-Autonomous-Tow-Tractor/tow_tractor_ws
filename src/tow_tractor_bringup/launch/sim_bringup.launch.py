import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import Command, PythonExpression

def generate_launch_description():

    robot_name = "tow_tractor_v1"
    use_gazebo = 'true'                 # set to false to disable gazebo plugins
    declare_use_gazebo_arg = DeclareLaunchArgument(
            'use_gazebo',
            default_value=use_gazebo,
            description='Whether to include Gazebo plugins'
        )
    use_gazebo_arg = LaunchConfiguration("use_gazebo")

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
        }],
        output='screen'
    )

    ##################  VISUALIZE IN RVIZ ##################
    node_rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', os.path.join(pkg_project_bringup, 'config', 'tow_tractor.rviz')],
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
            '-name', robot_name,
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
    xacro_file = PathJoinSubstitution([
        pkg_project_description,
        "urdf",
        robot_name,
        f"{robot_name}.urdf.xacro"
    ])

    # Generate robot_description using proper substitution handling
    robot_description_content = Command([
            "xacro ", xacro_file,
            " use_gazebo:=", use_gazebo_arg,
        ])

    params = [{
            "use_sim_time": PythonExpression(["'", use_gazebo_arg, "' == 'true'"]),
            "robot_description": robot_description_content,
        }]

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=params,
    )

    node_model_info_publisher = Node(
        package='tow_tractor_description',
        executable='model_info_publisher',
        output='screen',
        parameters=[os.path.join(pkg_project_description, 'config', f'{robot_name}.yaml')],
    )
    #########################################################################################
    #########################################################################################




    ############################ REAR STEERING CONTROLLER LAUNCH ############################
    #########################################################################################
    node_rear_steering_controller = Node(
        package='tow_tractor_control',
        executable='rear_wheel_steering_controller',
        output='screen',
        parameters=[{
            'cmd_vel_topic': 'cmd_vel',
            'speed_cmd_topic': f'/{robot_name}/cmd_motor_speed',
            'steer_cmd_topic': f'/{robot_name}/cmd_motor_pos',
            'model_info_topic': f'/{robot_name}/info',
        }]
    )
    #########################################################################################
    #########################################################################################




    ########################### SLAM TOOLBOX - ONLINE ASYNC LAUNCH ##########################
    #########################################################################################
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            ])
        ]),
        launch_arguments={
            'params_file': PathJoinSubstitution([
                pkg_project_bringup,
                'config',
                'mapper_params_online_async.yaml'
            ]),
            'use_sim_time': use_gazebo,
        }.items()
    )
    #########################################################################################
    #########################################################################################




    return LaunchDescription([

        ################ PACKAGE SETUP ACTIONS ################
        set_gz_sim_resource_path,
        declare_rviz,
        declare_use_gazebo_arg,

        ############# GAZEBO NODES AND LAUNCH FILES #############
        launch_gazebo_world,
        node_ros_gz_bridge,
        node_spawn_urdf,

        ############# R0BOT NODES AND LAUNCH FILES #############
        node_rviz,
        node_robot_state_publisher,
        node_model_info_publisher,
        node_rear_steering_controller,

        ############# Autonomous System NODES AND LAUNCH FILES #############
        slam_toolbox_launch,
    ])
