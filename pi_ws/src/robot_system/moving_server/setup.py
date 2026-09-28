#!/usr/bin/env python3

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=[
        "moving_server",
        "moving_server.controllers",
        "moving_server.executors",
    ],
    package_dir={"": "src"},
)

setup(**setup_args)
