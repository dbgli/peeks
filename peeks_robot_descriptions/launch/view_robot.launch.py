from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # region 参数定义

    robot_name_argument = DeclareLaunchArgument(
        name="robot_name",
        default_value="robot_1",
        description="The name of robot.",
    )
    robot_name = LaunchConfiguration("robot_name")

    robot_type_argument = DeclareLaunchArgument(
        name="robot_type",
        default_value="abb/crb15000",
        description="Type/series of used robot.",
        choices=[
            "abb/crb15000",
        ],
    )
    robot_type = LaunchConfiguration("robot_type")

    tool_name_argument = DeclareLaunchArgument(
        name="tool_name",
        default_value="tool_1",
        description="The name of tool.",
    )
    tool_name = LaunchConfiguration("tool_name")

    tool_type_argument = DeclareLaunchArgument(
        name="tool_type",
        default_value="dummy",
        description="Type/series of used tool.",
        choices=[
            "dummy",
        ],
    )
    tool_type = LaunchConfiguration("tool_type")

    tf_prefix_argument = DeclareLaunchArgument(
        name="tf_prefix",
        default_value='""',
        description="Prefix of the joint names, useful for multi-robot setup. If changed than also joint names in the controllers' configuration have to be updated.",
    )
    tf_prefix = LaunchConfiguration("tf_prefix")

    robot_description_file_argument = DeclareLaunchArgument(
        name="robot_description_file",
        default_value=PathJoinSubstitution([FindPackageShare("peeks_robot_descriptions"), "urdf", "robot.urdf.xacro"]),
        description="URDF/XACRO description file (absolute path) with the robot.",
    )
    robot_description_file = LaunchConfiguration("robot_description_file")

    rviz_config_file_argument = DeclareLaunchArgument(
        name="rviz_config_file",
        default_value=PathJoinSubstitution([FindPackageShare("peeks_robot_descriptions"), "rviz", "view_robot.rviz"]),
        description="RViz config file (absolute path) to use when launching rviz.",
    )
    rviz_config_file = LaunchConfiguration("rviz_config_file")

    # endregion 参数定义

    # region 节点定义

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            robot_description_file,
            " ",
            "robot_name:=",
            robot_name,
            " ",
            "robot_type:=",
            robot_type,
            " ",
            "tool_name:=",
            tool_name,
            " ",
            "tool_type:=",
            tool_type,
            " ",
            "tf_prefix:=",
            tf_prefix,
        ]
    )
    robot_description = {"robot_description": ParameterValue(value=robot_description_content, value_type=str)}
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="both",
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="both",
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_file],
        output="both",
    )

    # endregion 节点定义

    arguments = [
        robot_name_argument,
        robot_type_argument,
        tool_name_argument,
        tool_type_argument,
        tf_prefix_argument,
        robot_description_file_argument,
        rviz_config_file_argument,
    ]

    nodes = [
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz2_node,
    ]

    return LaunchDescription(arguments + nodes)