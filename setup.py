"""Setup script.

Usage examples:

    pip install -e .
    pip install -e .[develop]
"""

from setuptools import find_packages, setup

import ipdb; ipdb.set_trace()

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

