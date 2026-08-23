from glob import glob
import os

from setuptools import find_packages, setup


package_name = "beam_estimator"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Jinho Kwon",
    maintainer_email="jinhokwon9910@users.noreply.github.com",
    description="ROS 2 wrapper for the Unity camera beam-direction estimator.",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "beam_estimator_node = beam_estimator.beam_estimator_node:main",
        ],
    },
)
