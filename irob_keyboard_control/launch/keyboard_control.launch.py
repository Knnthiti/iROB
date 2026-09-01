from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    esp32_ip = LaunchConfiguration("esp32_ip")
    udp_port = LaunchConfiguration("udp_port")
    speed_rpm = LaunchConfiguration("speed_rpm")
    turn_rpm = LaunchConfiguration("turn_rpm")
    publish_rate_hz = LaunchConfiguration("publish_rate_hz")
    start_feedback = LaunchConfiguration("start_feedback")
    start_rqt_graph = LaunchConfiguration("start_rqt_graph")

    return LaunchDescription([
        DeclareLaunchArgument(
            "esp32_ip",
            default_value="192.168.1.100",
            description="ESP32 IP address shown in Arduino Serial Monitor.",
        ),
        DeclareLaunchArgument(
            "udp_port",
            default_value="6767",
            description="UDP port used by ESP32 and ROS2 bridge nodes.",
        ),
        DeclareLaunchArgument(
            "speed_rpm",
            default_value="100",
            description="Default translation speed in motor RPM.",
        ),
        DeclareLaunchArgument(
            "turn_rpm",
            default_value="100",
            description="Default rotation speed in motor RPM.",
        ),
        DeclareLaunchArgument(
            "publish_rate_hz",
            default_value="20.0",
            description="Keyboard command publish rate.",
        ),
        DeclareLaunchArgument(
            "start_feedback",
            default_value="true",
            description="Start the ESP32 feedback receiver node.",
        ),
        DeclareLaunchArgument(
            "start_rqt_graph",
            default_value="false",
            description="Open rqt_graph together with the control nodes.",
        ),
        Node(
            package="irob_ros",
            executable="iROB_ESP",
            name="iROB_feedback_bridge",
            output="screen",
            condition=IfCondition(start_feedback),
            arguments=["--port", udp_port],
        ),
        Node(
            package="irob_ros",
            executable="iROB_CMD",
            name="iROB_command_bridge",
            output="screen",
            arguments=["--esp32-ip", esp32_ip, "--port", udp_port],
        ),
        Node(
            package="irob_keyboard_control",
            executable="irob_keyboard",
            name="iROB_keyboard",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "speed_rpm": ParameterValue(speed_rpm, value_type=int),
                "turn_rpm": ParameterValue(turn_rpm, value_type=int),
                "publish_rate_hz": ParameterValue(publish_rate_hz, value_type=float),
            }],
        ),
        Node(
            package="rqt_graph",
            executable="rqt_graph",
            name="rqt_graph",
            output="screen",
            condition=IfCondition(start_rqt_graph),
        ),
    ])
