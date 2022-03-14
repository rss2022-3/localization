#!/usr/bin/env python2
python2
import rospy
from sensor_model import SensorModel
from motion_model import MotionModel
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from math import sin, cos

class ParticleFilter:
    def __init__(self):
        # Get parameters
        self.particle_filter_frame = \
                rospy.get_param("~particle_filter_frame")

        # Initialize publishers/subscribers
        #
        #  *Important Note #1:* It is critical for your particle
        #     filter to obtain the following topic names from the
        #     parameters for the autograder to work correctly. Note
        #     that while the Odometry message contains both a pose and
        #     a twist component, you will only be provided with the
        #     twist component, so you should rely only on that
        #     information, and *not* use the pose component.
        scan_topic = rospy.get_param("~scan_topic", "/scan")
        odom_topic = rospy.get_param("~odom_topic", "/odom")
        self.laser_sub = rospy.Subscriber(scan_topic, LaserScan,
                                          get_points, # TODO: Fill this in
                                          queue_size=1)
        self.odom_sub  = rospy.Subscriber(odom_topic, Odometry,
                                          get_odom, # TODO: Fill this in
                                          queue_size=1)
        self.pcd = None
        self.observations = None
        self.odom = None
        self.pos = None
        self.heading = None
        self.motion_noise_covariance = None
        self.sensor_noise_covariance = None
        #  *Important Note #2:* You must respond to pose
        #     initialization requests sent to the /initialpose
        #     topic. You can test that this works properly using the
        #     "Pose Estimate" feature in RViz, which publishes to
        #     /initialpose.
        self.pose_sub  = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped,
                                          init_pose, # TODO: Fill this in
                                          queue_size=1)

        #  *Important Note #3:* You must publish your pose estimate to
        #     the following topic. In particular, you must use the
        #     pose field of the Odometry message. You do not need to
        #     provide the twist part of the Odometry message. The
        #     odometry you publish here should be with respect to the
        #     "/map" frame.
        self.odom_pub  = rospy.Publisher("/pf/pose/odom", Odometry, queue_size = 1)
        
        # Initialize the models
        self.motion_model = MotionModel()
        self.sensor_model = SensorModel()

        #Pose initialization callback
        def init_pose(self, pose_msg):
          self.pos = pose_msg.pose.pose.position
          self.heading = pose_msg.pose.pose.orientation
          self.covar = pose_msg.pose.covariance
          
    
        def get_points(self, scan_msg):
          #Choose laser scan values to consider based on side of wall to follow.
          angles = np.array([scan_msg.angle_min + i*scan_msg.angle_increment for i in range(len(scan_msg.ranges))])
          ranges = np.array(scan_msg.ranges)
          pcd = np.vstack([ranges*np.cos(angles), ranges*np.sin(angles), angles])
          self.pcd = pcd
          
          self.observations = ranges
          

        def get_odm(self, data):
          twist_msg = data.twist
          x_dot = twist_msg.linear.x
          y_dot = twist_msg.linear.y
          th_dot = twist_msg.angular.z
           
          #Compute odometry matrix (input to MotionModel())
          odom = np.array([x_dot, y_dot, th_dot]) 
          self.odom = odom

        def MCL_update(self):
          particles = self.motion_model.evaluate(self.pcd, self.odom)

          weights = self.sensor_model.evaluate(particles, self.observations) 
          indices = np.random.choice(particles.shape[0], size=particles.shape[0], p=weights)
          new_particles = [particles[i] for i in indices]

          # Add a small amount of noise to blur the samples.
          mean = [0, 0, 0]
          covariance = [[.001, 0, 0], [0, 0.001, 0], [0, 0, np.deg2rad(1)**2]]

          for i, sample in enumerate(new_samples):
            new_samples[i] = sample + np.random.multivariate_normal(mean, covariance)

          eta = np.sum(weights)
          return np.linalg.inv(eta).dot(weights)

          

        # Implement the MCL algorithm
        # using the sensor model and the motion model
        #
        # Make sure you include some way to initialize
        # your particles, ideally with some sort
        # of interactive interface in rviz
        #
        # Publish a transformation frame between the map
        # and the particle_filter_frame.


if __name__ == "__main__":
    rospy.init_node("particle_filter")
    pf = ParticleFilter()
    rospy.spin()
