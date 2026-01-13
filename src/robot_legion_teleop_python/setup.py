from setuptools import setup

package_name = "robot_legion_teleop_python"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Vitruvian Systems LLC",
    maintainer_email="emilio@vitruvian.systems",
    description="Keyboard teleop, motor driver, and FPV web UI for Robot Legion robots.",
    license="CC-BY-NC-4.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # Teleop
            "legion_teleop_key = robot_legion_teleop_python.teleop_legion_key:main",

            # Motor driver
            "motor_driver_node = robot_legion_teleop_python.motor_driver_node:main",

            # Lock manager (NEW)
            "control_lock_manager = robot_legion_teleop_python.control_lock_manager:main",

            # FPV WebRTC UI (if you have fpv_web_server.py in this package)
            "fpv_web_server = robot_legion_teleop_python.fpv_web_server:main",

            # Gazebo reset utility
            "reset_gz = robot_legion_teleop_python.reset_gz:main",
        ],
    },
)
