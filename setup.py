#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['interaction_modules.notification_modules', 'interaction_modules.reporting_modules', 'state_machine',
              'util_modules'],
    package_dir={'': 'src'}
)

setup(**d)
