"""Setup script.

Usage examples:

    pip install -e .
    pip install -e .[develop]
"""

from setuptools import find_packages, setup

path_to_myproject = "src/"

setup(
    name="pbrspot",
    version="0.1.0",
    packages=find_packages(include=["pbrspot", "pbrspot.*"]),
    include_package_data=True,
    install_requires=[
        "numpy",
        "pybullet",
        "recordclass",
        "networkx",
        "scipy"
    ]
    )

