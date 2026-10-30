import os
from glob import glob

from setuptools import setup

package_name = "adaptive_path_replanner"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="robotics",
    maintainer_email="maintainer@example.com",
    description="Adaptive Nav2 path replanning based on progress and path deviation.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "adaptive_path_replanner = adaptive_path_replanner.adaptive_path_replanner:main",
        ],
    },
)
