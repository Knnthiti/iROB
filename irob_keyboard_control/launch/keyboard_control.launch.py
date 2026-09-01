from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # LaunchConfiguration คือค่าที่ผู้ใช้ส่งเข้ามาจากคำสั่ง ros2 launch
    esp32_ip = LaunchConfiguration("esp32_ip")
    udp_port = LaunchConfiguration("udp_port")
    speed_rpm = LaunchConfiguration("speed_rpm")
    turn_rpm = LaunchConfiguration("turn_rpm")
    publish_rate_hz = LaunchConfiguration("publish_rate_hz")
    start_feedback = LaunchConfiguration("start_feedback")
    start_rqt_graph = LaunchConfiguration("start_rqt_graph")

    return LaunchDescription([
        # IP ของ ESP32 ดูได้จาก Serial Monitor หลังจาก ESP32 join WiFi สำเร็จ
        DeclareLaunchArgument(
            "esp32_ip",
            default_value="192.168.1.100",
            description="IP ของ ESP32 ที่เห็นจาก Arduino Serial Monitor",
        ),
        # UDP port ต้องตรงกับ IROB_UDP_PORT ใน iROB_ros.ino
        DeclareLaunchArgument(
            "udp_port",
            default_value="6767",
            description="UDP port ที่ ESP32 และ ROS2 bridge ใช้ร่วมกัน",
        ),
        # ความเร็วพื้นฐานสำหรับ W/S/A/D
        DeclareLaunchArgument(
            "speed_rpm",
            default_value="100",
            description="ความเร็วพื้นฐานของการเคลื่อนที่ หน่วย RPM",
        ),
        # ความเร็วพื้นฐานสำหรับ Q/E
        DeclareLaunchArgument(
            "turn_rpm",
            default_value="100",
            description="ความเร็วพื้นฐานของการหมุน หน่วย RPM",
        ),
        # ความถี่ publish /iROB_command ควรเร็วกว่าค่า timeout บน ESP32
        DeclareLaunchArgument(
            "publish_rate_hz",
            default_value="20.0",
            description="ความถี่ publish คำสั่ง keyboard",
        ),
        # เปิด/ปิด node รับ feedback จาก ESP32
        DeclareLaunchArgument(
            "start_feedback",
            default_value="true",
            description="เริ่ม node รับ feedback จาก ESP32",
        ),
        # เปิด rqt_graph พร้อมระบบควบคุม ถ้าต้องการดู graph ทันที
        DeclareLaunchArgument(
            "start_rqt_graph",
            default_value="false",
            description="เปิด rqt_graph พร้อม node ควบคุม",
        ),
        # iROB_ESP รับ feedback จาก ESP32 แล้ว publish /iROB_controller
        Node(
            package="irob_ros",
            executable="iROB_ESP",
            name="iROB_feedback_bridge",
            output="screen",
            condition=IfCondition(start_feedback),
            arguments=["--port", udp_port],
        ),
        # iROB_CMD subscribe /iROB_command แล้วส่งคำสั่งไป ESP32
        Node(
            package="irob_ros",
            executable="iROB_CMD",
            name="iROB_command_bridge",
            output="screen",
            arguments=["--esp32-ip", esp32_ip, "--port", udp_port],
        ),
        # iROB_keyboard อ่านปุ่ม keyboard แล้ว publish /iROB_command ต่อเนื่อง
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
        # rqt_graph ใช้ดู ROS2 node/topic เฉพาะตอน start_rqt_graph:=true
        Node(
            package="rqt_graph",
            executable="rqt_graph",
            name="rqt_graph",
            output="screen",
            condition=IfCondition(start_rqt_graph),
        ),
    ])
