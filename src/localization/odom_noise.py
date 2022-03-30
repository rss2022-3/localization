#!/usr/bin/env python2
import rospy
import numpy as np
from nav_msgs.msg import Odometry

class OdomNoise:
    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic", "/odom")
        self.odom_sub  = rospy.Subscriber(self.odom_topic, Odometry,
                                          self.add_noise, # TODO: Fill this in
                                          queue_size=1)
        self.odom_noise_pub = rospy.Publisher("odom_noise", Odometry, queue_size = 1)


    def add_noise(self, odom_msg):
        mean = [0, 0, 0]
        covariance = [[.001, 0, 0], [0, 0.001, 0], [0, 0, np.deg2rad(0.5)**2]]
        noise = np.random.multivariate_normal(mean, covariance)
        
        noise_x = noise[0]
        noise_y = noise[1]
        noise_z = noise[2]

        odom_msg.twist.twist.linear.x += noise_x
        odom_msg.twist.twist.linear.y += noise_y
        odom_msg.twist.twist.angular.z += noise_z

        self.odom_noise_pub.publish(odom_msg)

if __name__ == "__main__":
    rospy.init_node("odom_noise")
    pf = OdomNoise()
    rospy.spin()