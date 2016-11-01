#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['command_executer'],
    package_dir={'': 'command_executer'}
)

setup(**d)