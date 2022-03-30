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

    def pose_matrix(self, state):
        x, y, th = state[0], state[1], state[2]
        mat =  np.array([[cos(th), -sin(th), 0,  x],
                         [sin(th),  cos(th), 0,  y], 
                         [0,              0, 1, th],
                         [0,              0, 0,  1]])
        return mat
	    
    def compute_final_state(self, init_state, d_x):
        pose_mat = self.pose_matrix(init_state)
	
        return pose_mat.dot(d_x)


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
        odometry = np.append(odometry, 1)
        for particle in particles:
            next_particle = self.compute_final_state(particle, odometry)    
            next_particle = next_particle[:-1]
            ans.append(next_particle)
       
        return np.array(ans)

    # def evaluate(self, particles, odometry):
    #     """
    #     Update the particles to reflect probable
    #     future states given the odometry data.

    #     args:
    #         particles: An Nx3 matrix of the form:

    #             [x0 y0 theta0]
    #             [x1 y0 theta1]
    #             [    ...     ]

    #         odometry: A 3-vector [dx dy dtheta]

    #     returns:
    #         particles: An updated matrix of the
    #             same size
     
    #     """
        # ans = []
        # #rospy.loginfo(odometry.shape)
        
        # for particle in particles:
        #     noisy_odometry = odometry 
        #     dtheta = noisy_odometry[2]
        #     odom = np.matrix([[cos(dtheta), -sin(dtheta), noisy_odometry[0]],[sin(dtheta), cos(dtheta), noisy_odometry[1]],[0,0,1]])

        #     E = random.random()
        #     theta = particle[2]
        #     #rospy.loginfo(particle.shape) 
        #     y = particle[1]
        #     x = particle[0]
        #     pmatrix = np.matrix([[cos(theta), -sin(theta), x],[sin(theta), cos(theta), y],[0,0,1]])
        #     temp = pmatrix.dot(odom)
        #     # add noise
        #     #rospy.loginfo(temp.shape)
        #     #theta = max(acos(temp[0,0]),asin(temp[1,0]))
        #     theta += acos(odom[0,0])


            
        #     ans.append([temp[0,2], temp[1,2], theta])
        # #rospy.loginfo(ans)
        # return np.array(ans)
        #daweeds way of optimizing this shit like hardcore
       
        #x, y, theta = particles[:, 0], particles[:, 1], particles[:, 2]

        ##need to transform dx, dy by dtheta
        #R = np.array([[np.cos(odometry[2]), -np.sin(odometry[2]), 0],
        #              [np.sin(odometry[2]),  np.cos(odometry[2]), 0],
        #              [0,0,1]])

        #dx, dy, dtheta = np.transpose(R).dot(np.array([[odometry[0]], [odometry[1]], [odometry[2]]]))[:, 0].tolist()

        #return np.stack([x + np.cos(theta) * dx - np.sin(theta)* dy,
        #                y + np.sin(theta) * dx + np.cos(theta) * dy,
        #                theta + dtheta], axis=1)

        #return np.stack([x + np.cos(theta) * odometry[0] - np.sin(theta) * odometry[1],
        #                y + np.sin(theta)  * odometry[0] + np.cos(theta) * odometry[1],
        #                theta + dtheta], axis=1)