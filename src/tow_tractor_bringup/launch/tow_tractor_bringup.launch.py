import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, PythonExpression

def generate_launch_description():

    robot_name = "tow_tractor_v2"
    robot_control = "diff_drive"        # ['rear_steer', 'diff_drive']
    use_gazebo = 'true'
    use_lidar = 'false'

    # /dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_34330313231351809001-if00
    # /dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_34330313231351B051E2-if00
    # /dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_854353331313517061C2-if00 -> uno
    port_arduino_1 = "/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_854353331313517061C2-if00"
    port_arduino_2 = "/dev/ttyACM1"
    baudrate = 115200

    pkg_project_bringup = os.path.join(get_package_share_directory("tow_tractor_bringup"))
    pkg_project_description = os.path.join(get_package_share_directory("tow_tractor_description"))
    pkg_project_gazebo = os.path.join(get_package_share_directory("tow_tractor_gazebo"))
    pkg_project_hardware = os.path.join(get_package_share_directory("tow_tractor_hardware"))
    pkg_project_slam = os.path.join(get_package_share_directory("tow_tractor_slam"))

    print(f"▶ robot_name    = {robot_name}")
    print(f"▶ robot_control = {robot_control}")
    print(f"▶ port_arduino_1 = {port_arduino_1}, baudrate = {baudrate}")
    print(f"▶ port_arduino_2 = {port_arduino_2}, baudrate = {baudrate}")

    ###################################### GAZEBO MODE ######################################
    #########################################################################################

    ############# GZ-ROS-LAUNCH INITIAL SETUP #############

    gz_sim_resource_path = f"{pkg_project_bringup}:{pkg_project_gazebo}:{pkg_project_description}"
    set_gz_sim_resource_path = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',gz_sim_resource_path)

    declare_use_gazebo_arg = DeclareLaunchArgument(
            'use_gazebo',
            default_value=use_gazebo,
            description='Whether to include Gazebo plugins'
        )
    use_gazebo_arg = LaunchConfiguration("use_gazebo")

    ################## GZ-ROS-BRIDGE NODE ##################

    node_ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_bringup, 'config', f'{robot_name}_ros_gz_bridge_config.yaml'),
        }],
        condition=IfCondition(use_gazebo_arg),
        output='screen'
    )

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
        }.items(),
        condition=IfCondition(use_gazebo_arg),
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
        output='screen',
        condition=IfCondition(use_gazebo_arg),
    )
    #########################################################################################
    #########################################################################################




    ##################################### HARDWARE MODE #####################################
    #########################################################################################
    declare_use_lidar_arg = DeclareLaunchArgument(
        'lidar',
        default_value=use_lidar,
        description='Whether to include the LDLidar node'
    )
    use_lidar_arg = LaunchConfiguration("lidar")

    ldlidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('ldlidar_node'),
                'launch',
                'ldlidar_with_mgr.launch.py'
            ])
        ]),
        condition=IfCondition(
            PythonExpression([
                "'", use_gazebo_arg, "'", " == 'false' and ", "'", use_lidar_arg, "'", " == 'true'"
            ])
        ),
    )
    node_sensor_receiver = Node(
        package='tow_tractor_hardware',
        executable='sensor_receiver_node',
        name='sensor_receiver_node',
        output='screen',
        parameters=[{
            'baudrate_imu': baudrate,
            'channel_imu': port_arduino_1,
            'baudrate_encoders': baudrate,
            'channel_encoders': port_arduino_1,
            'baudrate_actuator_feedback': baudrate,
            'channel_actuator_feedback': port_arduino_1,
            'imu_msg_id': 0x30,
            'encoders_msg_id': 0x20,
            'actuator_feedback_msg_id': 0x31,
            'timer_period': 0.002,
            'imu_topic': f'/{robot_name}/imu',
            'right_motor_feedback_topic': f'/{robot_name}/feedback_right_motor_raw',
            'left_motor_feedback_topic': f'/{robot_name}/feedback_left_motor_raw',
            'actuator_feedback_topic': f'/{robot_name}/actuator_status',
        }],
        condition=UnlessCondition(use_gazebo_arg),
    )

    node_actuators_sender = Node(
        package='tow_tractor_hardware',
        executable='actuators_sender_node',
        name='actuators_sender_node',
        output='screen',
        parameters=[{
            'baudrate_motors': baudrate,
            'channel_motors': port_arduino_1,
            'baudrate_actuator': baudrate,
            'channel_actuator': port_arduino_1,
            'motors_msg_id': 0x10,
            'actuator_msg_id': 0x11,
            'cmd_motor1_topic': f'/{robot_name}/cmd_motor1',
            'cmd_motor2_topic': f'/{robot_name}/cmd_motor2',
            'cmd_actuator_topic': f'/{robot_name}/cmd_actuator',
            'timer_period': 0.04,
        }],
        condition=UnlessCondition(use_gazebo_arg),
    )

    node_odometry_publisher = Node(
        package='tow_tractor_hardware',
        executable='odometry_publisher_node',
        name='odometry_publisher_node',
        output='screen',
        parameters=[{
            'right_motor_feedback_topic': f'/{robot_name}/feedback_right_motor_raw',
            'left_motor_feedback_topic': f'/{robot_name}/feedback_left_motor_raw',
            'odom_unfiltered_topic': f'/odom_unfiltered',
            'joint_states_topic': f'/joint_states',
            'model_info_topic': f'/{robot_name}/info',
        }],
        condition=UnlessCondition(use_gazebo_arg),
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

    robot_description_content = Command([
            "xacro ", xacro_file,
            " use_gazebo:=", use_gazebo_arg,
        ])

    params = [{
            "use_sim_time": use_gazebo_arg,
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

    ##################  VISUALIZE IN RVIZ ##################
    node_rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', os.path.join(pkg_project_bringup, 'config', 'tow_tractor.rviz')],
       condition=IfCondition(LaunchConfiguration('rviz')),
    )

    declare_rviz = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Open RViz.',
    )

    #########################################################################################
    #########################################################################################




    #################################### SLAM && NAVIGATION #################################
    #########################################################################################

    # sensor fusion for localization
    node_robot_localization = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[
            PathJoinSubstitution([pkg_project_slam, 'config', 'ekf.yaml']),
            {'use_sim_time': use_gazebo_arg},
            ]
    )

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
                pkg_project_slam,
                'config',
                'slam.yaml'
            ]),
            'use_sim_time': use_gazebo_arg,
        }.items()
    )
    #########################################################################################
    #########################################################################################




    ##################################### CONTROL NODES #####################################
    #########################################################################################
    node_rear_steering_controller = Node(
        package='tow_tractor_control',
        executable='rear_wheel_steering_controller',
        output='screen',
        parameters=[{
            'cmd_vel_topic': '/cmd_vel',
            'model_info_topic': f'/{robot_name}/info',
            'speed_cmd_topic': f'/{robot_name}/cmd_motor_speed',
            'steer_cmd_topic': f'/{robot_name}/cmd_motor_pos',
        }],
        condition=IfCondition(PythonExpression(["'", robot_control, "' == 'rear_steer'"])),
    )

    node_diff_driver_controller = Node(
        package='tow_tractor_control',
        executable='diff_drive_controller',
        name='diff_drive_controller',
        parameters=[
            {'cmd_vel_topic': '/cmd_vel'},
            {'model_info_topic': f'/{robot_name}/info'},
            {'cmd_motor1_topic': f'/{robot_name}/cmd_motor1'}, # Right motor
            {'cmd_motor2_topic': f'/{robot_name}/cmd_motor2'}, # Left motor
        ],
        output='screen',
        condition=IfCondition(PythonExpression(["'", robot_control, "' == 'diff_drive'"])) and UnlessCondition(use_gazebo_arg),
        )
    #########################################################################################
    #########################################################################################




    ############# LDLidar node launch (only if not using gazebo and use_lidar is true) #############

    #########################################################################################
    #########################################################################################




    return LaunchDescription([

        ################ PACKAGE SETUP ACTIONS ################
        set_gz_sim_resource_path,
        declare_rviz,
        declare_use_gazebo_arg,
        declare_use_lidar_arg,

        ############# R0BOT NODES AND LAUNCH FILES #############
        node_robot_state_publisher,
        node_model_info_publisher,
        node_rviz,

        ############# GAZEBO NODES AND LAUNCH FILES #############
        launch_gazebo_world,
        node_ros_gz_bridge,
        node_spawn_urdf,

        ############# HARDWARE NODES AND LAUNCH FILES #############
        ldlidar_launch,
        node_sensor_receiver,
        node_actuators_sender,
        node_odometry_publisher,

        ############# Autonomous System NODES AND LAUNCH FILES #############
        node_robot_localization,
        slam_toolbox_launch,
        node_rear_steering_controller,
        node_diff_driver_controller,
    ])
