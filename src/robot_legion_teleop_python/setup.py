from setuptools import setup

package_name = "robot_legion_teleop_python"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/robot_profiles.yaml"]),
        ("share/" + package_name + "/launch", ["launch/robot_bringup.launch.py", "launch/robot_minimal.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Vitruvian Systems",
    maintainer_email="n/a",
    description="Heterogeneous fleet teleop + robot-side adapters (proprietary).",
    license="LicenseRef-Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "teleop_legion_key = robot_legion_teleop_python.teleop_legion_key:main",
            "motor_driver_node = robot_legion_teleop_python.motor_driver_node:main",
            "motor_driver_node_OLD = robot_legion_teleop_python.motor_driver_node_OLD:main",
            "heartbeat_node = robot_legion_teleop_python.heartbeat_node:main",
            "unit_executor_action_server = robot_legion_teleop_python.unit_executor_action_server:main",
            # leave your existing ones if you still use them:
            "fpv_control_arbiter = robot_legion_teleop_python.fpv_control_arbiter:main",
            "fpv_camera_mux = robot_legion_teleop_python.fpv_camera_mux:main",
            "usb_camera_node = robot_legion_teleop_python.usb_camera_node:main",
            "legion_camera_node = robot_legion_teleop_python.legion_camera_node:main",
            "reset_gz = robot_legion_teleop_python.reset_gz:main",
        ],
    },
)
