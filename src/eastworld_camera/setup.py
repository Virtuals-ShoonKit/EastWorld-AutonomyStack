from setuptools import setup

package_name = "eastworld_camera"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", [
            "config/csi_streamer.yaml",
            "config/camera_tilt.yaml",
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="EastWorld's Janitor",
    maintainer_email="shoonkit@virtuals.io",
    description="Jetson CSI camera streamer with HW H.264/JPEG for Foxglove",
    license="MIT",
    entry_points={
        "console_scripts": [
            "csi_streamer = eastworld_camera.csi_streamer:main",
            "test_camera = eastworld_camera.test_camera:main",
            "camera_tilt = eastworld_camera.camera_tilt:main",
        ],
    },
)
