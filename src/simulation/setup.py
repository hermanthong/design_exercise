import os
from glob import glob

from setuptools import setup

package_name = "simulation"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Fabrica AI",
    maintainer_email="software@fabrica.ai",
    description="Lightweight grout-line-following simulation for the Fabrica take-home.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "simulation = simulation.sim_node:main",
        ],
    },
)
