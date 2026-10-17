from glob import glob

from setuptools import setup

package_name = "mr_racing"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "esp32_bridge = mr_racing.esp32_bridge:main",
            "race_node = mr_racing.race_node:main",
            "vision_node = mr_racing.vision_node:main",
            "teleop_check = mr_racing.teleop_check:main",
        ],
    },
)
