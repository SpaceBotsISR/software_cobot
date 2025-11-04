from setuptools import find_packages, setup
import os
from glob import glob

package_name = "cobot_gz"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
        (os.path.join("share", package_name, "meshes"), glob("meshes/*")),
        (os.path.join("share", package_name, "sdf"), glob("sdf/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="space_cobot",
    maintainer_email="itsnotsoftware@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "space_cobot_state_bridge = cobot_gz.state_bridge:main",
            "space_cobot_tf_broadcaster = cobot_gz.tf_broadcaster:main",
            "cmd_bridge = cobot_gz.cmd_bridge:main",
        ],
    },
)
