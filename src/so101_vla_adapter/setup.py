from glob import glob
from setuptools import find_packages, setup


PACKAGE_NAME = "so101_vla_adapter"


setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE_NAME}"]),
        (f"share/{PACKAGE_NAME}", ["package.xml", "README.md"]),
        (f"share/{PACKAGE_NAME}/config", glob("config/*.yaml")),
        (f"share/{PACKAGE_NAME}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Noah",
    maintainer_email="noah@example.com",
    description="Thin, validated GetActionChunk adapter for SO-101 policy servers.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "get_action_chunk_adapter = so101_vla_adapter.adapter_node:main",
            "hold_policy_server = so101_vla_adapter.hold_policy_server:main",
        ],
    },
)
