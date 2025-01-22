import os
from os import pathsep
from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    flexobot_description = get_package_share_directory("flexobot_description")
    flexobot_description_prefix = get_package_prefix("flexobot_description")
    gazebo_ros_dir = get_package_share_directory("gazebo_ros")

    model_arg = DeclareLaunchArgument(name="model", default_value=os.path.join(
                                        flexobot_description, "urdf", "flexobot.urdf"
                                        ),
                                      description="Absolute path to robot urdf file"
    )

    model_path = os.path.join(flexobot_description, "meshes")
    model_path += pathsep + os.path.join(flexobot_description_prefix, "share")

    env_var = SetEnvironmentVariable("GAZEBO_MODEL_PATH", model_path)

    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]),
                                       value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, "launch", "gzserver.launch.py")
        )
    )

    start_gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, "launch", "gzclient.launch.py")
        )
    )

    spawn_robot = Node(package="gazebo_ros", executable="spawn_entity.py",
                        arguments=["-entity", "flexobot",
                                   "-topic", "robot_description",
                                  ],
                        output="screen"
    )

    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("flexobot_controllers"),
            "launch",
            "controllers.launch.py"
        ),
    )

    # joystick_controller = IncludeLaunchDescription(
    #     os.path.join(
    #         get_package_share_directory("flexobot_controller"),
    #         "launch",
    #         "joystick_teleop.launch.py"
    #     ),
    # )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(flexobot_description, "rviz", "display.rviz")],
    )

    return LaunchDescription([
        env_var,
        model_arg,
        start_gazebo_server,
        start_gazebo_client,
        robot_state_publisher_node,
        spawn_robot,
        controller,
        # rviz_node,
        # joystick_controller
    ])