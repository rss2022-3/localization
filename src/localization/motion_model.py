import rospy
import numpy as np
from math import cos, sin, acos, asin
from std_msgs.msg import Float32
import random

class MotionModel:

    def __init__(self):
      
      '''
      ####################################
      # TODO
      # Do any precomputation for the motion
      # model here.
      #rospy.init('motion_model')
      #pub = rospy.Publisher("", ,queue_size=10)
      #sub = rospy.Subscriber()
      ####################################
      # TODO need to get covariance Matrix
      '''

    def evaluate(self, particles, odometry):
       """
        Update the particles to reflect probable
        future states given the odometry data.

        args:
            particles: An Nx3 matrix of the form:

                [x0 y0 theta0]
                [x1 y0 theta1]
                [    ...     ]

            odometry: A 3-vector [dx dy dtheta]

        returns:
            particles: An updated matrix of the
                same size
       """
       ans = []
       #rospy.loginfo(odometry.shape)
       dtheta = odometry[2]
       odom = np.matrix([[cos(dtheta), -1*sin(dtheta), odometry[0]],[sin(dtheta), cos(dtheta), odometry[1]],[0,0,1]])
       for particle in particles:
          E = random.random()
          theta = particle[2]
          #rospy.loginfo(particle.shape)
          y = particle[1]
          x = particle[0]
          pmatrix = np.matrix([[cos(theta), -1*sin(theta), x],[sin(theta), cos(theta), y],[0,0,1]])
          temp = pmatrix.dot(odom) 
          # add noise
          #rospy.loginfo(temp.shape)
          #theta = max(acos(temp[0,0]),asin(temp[1,0]))
          theta += acos(odom[0,0])


          
          ans.append([temp[0,2], temp[1,2], theta])
       #rospy.loginfo(ans)
       return np.array(ans)

       #daweeds way of optimizing this shit like hardcore
       #THIS IS ONLY IN 2D
       motion_noise_covariance = np.array([[0.1,                 0],
                                           [0,  np.deg2rad(10)**2]], dtype='float64')

       n = particles.shape[0]
       noise = np.random.multivariate_normal(np.array([0, 0]), motion_noise_covariance, size=n)
       x, y, theta = particles[:, 0], particles[:, 1], particles[:, 2]
       return np.stack([x + np.cos(theta) * (dx + noise[:, 0]),
                        y + np.sin(theta) * (dy + noise[:, 0]),
                        theta + dtheta + noise[:, 1]], axis=1)

        dx, dy, dheta = odometry
        odom = np.stack([   [np.cos(dtheta + noise[:, 1]), -np.sin(dtheta + noise[:, 1]), dx + noise[:,0]],
                            [np.sin(dtheta + noise[:, 1]),  np.cos(dtheta + noise[:, 1]), dy + noise[:,0]],
                            [0,0,1]])
        T_Pose = np.array()



