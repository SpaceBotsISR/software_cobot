from setuptools import find_packages, setup
import glob

package_name = "slam"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob.glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="space_cobot",
    maintainer_email="itsnotsoftware@gmail.com",
    description="EKF-SLAM with ArUco marker detection.",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ekf = slam.ekf:main",
            "gtsam = slam.gtsam:main",
        ],
    },
)
