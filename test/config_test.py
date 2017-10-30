#!/usr/bin/env python

import unittest
import rostest
import rospy
from util_modules import config_access


class ConfigTestClass(unittest.TestCase):
    def test_get_version(self):
        """
        Test to ensure the 'version' can be retrieved from the config.
        """
        version = config_access.get_config(config_access.KEY_VERSION)
        self.assertIsNotNone(version, 'Version is none.')
        self.assertNotEqual(version, '', 'Version is empty.')
        self.assertRegexpMatches(version, '^(\d+\.)+\d+$', 'Version not regexed right.')


if __name__ == '__main__':
    rospy.init_node('config_test')
    rostest.rosrun('leslie', 'config_test', ConfigTestClass)
