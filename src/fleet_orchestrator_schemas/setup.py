# SPDX-License-Identifier: LicenseRef-Proprietary
from setuptools import setup
from glob import glob
import os

package_name = "fleet_orchestrator_schemas"

setup(
    name=package_name,
    version="0.1.0",
    packages=[],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "schemas"), glob("schemas/*.json")),
        (os.path.join("share", package_name, "examples"), glob("examples/*.json")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
)
