from glob import glob
import os

from setuptools import setup


package_name = "irob_keyboard_control"

# setup.py นี้ทำให้ package แบบ ament_python ถูก build/install ด้วย colcon ได้
setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        # ลงทะเบียน package ให้ ROS2 หาเจอผ่าน ament index
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        # ติดตั้ง metadata และ README ไปที่ share ของ package
        ("share/" + package_name, ["package.xml", "README.md"]),
        # ติดตั้ง launch file เพื่อให้ ros2 launch เรียกใช้ได้
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        # ติดตั้งเอกสาร rqt_graph ไปพร้อม package
        (os.path.join("share", package_name, "docs"), glob("docs/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="iROB Team",
    maintainer_email="irob@example.com",
    description="Keyboard teleoperation package for sending iROB motor commands to ESP32.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        # สร้างคำสั่ง ros2 run irob_keyboard_control irob_keyboard
        "console_scripts": [
            "irob_keyboard = irob_keyboard_control.keyboard_control:main",
        ],
    },
)
