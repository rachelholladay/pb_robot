"""Setup script.

Usage examples:

    pip install -e .
    pip install -e .[develop]
"""

from setuptools import find_packages, setup

path_to_myproject = "src/"

setup(
    name="pb_robot_spot",
    version="0.1.0",
    packages=find_packages(include=["src", "src/."]),
    include_package_data=True,
    install_requires=[
        "numpy",
        "pybullet",
        "recordclass",
        "networkx",
    ],
    include_package_data=True,
    )

