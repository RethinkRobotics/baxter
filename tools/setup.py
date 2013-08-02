#!/usr/bin/env python
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup()
d['packages'] = ['baxter_tools','smoke_test']
d['package_dir'] = {'':'src'}

setup(**d)
