from glob import glob
from setuptools import find_packages, setup


package_name = "snowplow_navigation"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Mingrxc",
    maintainer_email="2535854358@qq.com",
    description="Nav2 click-to-go navigation for snowplow_sim.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "ground_truth_tf = snowplow_navigation.ground_truth_tf:main",
            "navigation_goal = snowplow_navigation.navigation_goal:main",
        ],
    },
)
