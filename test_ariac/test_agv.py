#!/usr/bin/env python

from __future__ import print_function

import sys
import time
import unittest

from test_example_node import ExampleNodeTester
from ariac_example import ariac_example
from std_msgs.msg import Float32
import rospy
import rostest

import geometry_msgs.msg
import tf2_ros
import tf2_geometry_msgs  # import support for transforming geometry_msgs stamped msgs


class AgvTester(ExampleNodeTester):

    def test(self):
        self.prepare_tester()

        # Starting the competition will cause parts from the order to be spawned on AGV1
        self._test_start_comp()
        time.sleep(0.5)

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        for i in range(4):
            self._test_agv_control()
            time.sleep(20.0)

        # Check the score
        self._test_comp_end()

    def _test_agv_pose():
        frame = 'kit_tray_1_frame'
        # Ensure that the transform is available.
        #try:
        trans = self.tfBuffer.lookup_transform('world', frame, rospy.Time(), rospy.Duration(1.0))
        #except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        #    print(e)

        print(trans)
        tol = 0.05
        expected_pose = geometry_msgs.msg.PoseStamped()
        expected_pose.header.frame_id = 'world'
        expected_pose.pose.position.x = 1.25
        expected_pose.pose.position.y = 0.15
        self.assert(abs(trans.pose.position.x - expected_pose.pose.position.x) < tol, 'AGV x position incorrect')
        self.assert(abs(trans.pose.position.y - expected_pose.pose.position.y) < tol, 'AGV y position incorrect')


if __name__ == '__main__':
    rospy.init_node('test_agv', anonymous=True)

    # Wait until /clock is being published; this can take an unpredictable
    # amount of time when we're downloading models.
    while rospy.Time.now().to_sec() == 0.0:
        print('Waiting for Gazebo to start...')
        time.sleep(1.0)
    # Take an extra nap, to allow plugins to be loaded
    time.sleep(10.0)
    print('OK, starting test.')

    rostest.run('test_ariac', 'test_agv', AgvTester, sys.argv)
