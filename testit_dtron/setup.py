#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup()
# d['packages'] = ['testit_dtron']
# d['package_dir'] = {'': 'testit_dtron'}

setup(**d)
