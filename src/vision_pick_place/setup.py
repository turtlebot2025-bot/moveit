from setuptools import find_packages, setup
import glob

package_name = "vision_pick_place"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
             ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch",
             glob.glob("launch/*.py")),
        ("share/" + package_name + "/config",
             glob.glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="cojetson",
    maintainer_email="cojetson@cojetson-desktop",
    description="Vision-guided pick and place using RealSense D435 eye-in-hand",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "green_ball_detector   = vision_pick_place.green_ball_detector:main",
            "camera_tf_broadcaster = vision_pick_place.camera_tf_broadcaster:main",
            "vision_pick_place     = vision_pick_place.vision_pick_place:main",
        ],
    },
)