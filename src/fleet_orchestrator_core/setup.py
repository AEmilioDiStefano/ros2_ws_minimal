# SPDX-License-Identifier: LicenseRef-Proprietary
from setuptools import setup
from glob import glob
import os

package_name = "fleet_orchestrator_core"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "intent_gateway = fleet_orchestrator_core.intent_gateway_node:main",
            "intent_translator = fleet_orchestrator_core.intent_translator_node:main",
            "audit_logger = fleet_orchestrator_core.audit_logger_node:main",
            "unit_emulator = fleet_orchestrator_core.unit_emulator_node:main",
        ],
    },
)
