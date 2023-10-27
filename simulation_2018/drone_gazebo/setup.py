#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

__author__ = "Anass Al-Wohoush"

d = generate_distutils_setup(packages=["odlc"], package_dir={"": "src"})

setup(**d)
