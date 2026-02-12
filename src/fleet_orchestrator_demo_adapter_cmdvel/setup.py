# SPDX-License-Identifier: LicenseRef-Proprietary
from setuptools import setup

package_name = "fleet_orchestrator_demo_adapter_cmdvel"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "cmdvel_adapter = fleet_orchestrator_demo_adapter_cmdvel.cmdvel_adapter_node:main",
        ],
    },
)
