from glob import glob
import os

from setuptools import setup


package_name = "irob_keyboard_control"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
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
        "console_scripts": [
            "irob_keyboard = irob_keyboard_control.keyboard_control:main",
        ],
    },
)
