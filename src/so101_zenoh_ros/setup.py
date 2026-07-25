from setuptools import find_packages, setup


PACKAGE_NAME = "so101_zenoh_ros"


setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE_NAME}"]),
        (f"share/{PACKAGE_NAME}", ["package.xml"]),
        (f"share/{PACKAGE_NAME}/launch", ["launch/bridge.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Noah",
    maintainer_email="noah@example.com",
    description="ROS joint adapter for the SO-101 native Zenoh supervisor.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "so101_zenoh_bridge = so101_zenoh_ros.bridge_node:main",
        ],
    },
)
