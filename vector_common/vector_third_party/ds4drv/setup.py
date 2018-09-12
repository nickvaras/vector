#!/usr/bin/env python
## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=["ds4drv",
              "ds4drv.actions",
              "ds4drv.backends",
              "ds4drv.packages"],
    package_dir={'': 'src'})

setup(**setup_args)
