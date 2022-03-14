#!/usr/bin/env python2

import unittest
import numpy as np

import rospy
import rostest
from localization.motion_model import MotionModel
from __init__ import TEST_PARTICLES, TEST_MOTION_MODEL_ODOM, TEST_MOTION_MODEL_RESULTS

class TestMotionModel(unittest.TestCase):
    def setUp(self):
        self.motion_model = MotionModel()
        self.particles = np.array(TEST_PARTICLES)
        self.odom = np.array(TEST_MOTION_MODEL_ODOM)
        self.expected = np.array(TEST_MOTION_MODEL_RESULTS)
        print("setup") 
        self.tol = 1e-2
       # rospy.loginfo("AAAA")
    def tearDown(self):
        pass

    def test_evaluate_motion_model(self):
        print("here")
        rospy.loginfo("\n\nActual: {}".format(self.motion_model.evaluate(self.particles, self.odom)))
        rospy.loginfo("\n\nExpected: {}".format(self.expected))
        np.testing.assert_allclose(self.expected,
                self.motion_model.evaluate(self.particles, self.odom),
                rtol=self.tol)

if __name__ == "__main__":
    print("main")
    rospy.init_node("motion_model_test")
    rostest.rosrun("localization", 'test_motion_model', TestMotionModel)

