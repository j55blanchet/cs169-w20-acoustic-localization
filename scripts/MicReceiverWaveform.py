#!/usr/bin/env python

# Team Acoustic
# CS 69-169 Winter 2020
#
# Acoustic DoA
# MicReceiver.py
# Mack Reiferson and Siddharth Agrawal
#
# Tests the concept
# 
# 

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math


#STATIC PARAMETERS
M1_X = 50
M1_Y = 50
M2_X = 48
M2_Y = 50
M3_X = 49
M3_Y = 51
SPEED_OF_SOUND = 343    #meters per second (truly 343 m/s)

VECTOR_MATCH_TOLERANCE = 0.1


class MicReceiverWaveform:
    def __init__(self):
        self.m1_x = M1_X
        self.m1_y = M1_Y
        self.m2_x = M2_X
        self.m2_y = M2_Y
        self.m3_x = M3_X
        self.m3_y = M3_Y
        self.speed_of_sound = SPEED_OF_SOUND

        print "\n--- Starting MicReceiverWaveform execution ---"

        self.L_vec_12 = self.findVector(self.m1_x, self.m1_y, self.m2_x, self.m2_y)
        self.L_vec_13 = self.findVector(self.m1_x, self.m1_y, self.m3_x, self.m3_y)

        self.L_mag_12 = self.findDistance(self.m1_x, self.m1_y, self.m2_x, self.m2_y)
        self.L_mag_13 = self.findDistance(self.m1_x, self.m1_y, self.m3_x, self.m3_y)

        #print "DEBUG: L_mag_12:", self.L_mag_12
        #print "DEBUG: L_mag_13:", self.L_mag_13

    def decrypt(self, delta_t12, delta_t13):
        
        theta_12 = np.arccos(self.speed_of_sound * delta_t12 / self.L_mag_12)
        theta_13 = np.arccos(self.speed_of_sound * delta_t13 / self.L_mag_13)

        # Handle special cases that the incoming vector is colinear with one of the microphone vectors
        if (theta_12 == 0):

            # If the time-delta is in the same direction as the vector between mics, return the vector between mics
            if delta_t12 > 0:
                vector = self.L_vec_12
            elif delta_t12 == 0:
                vector = [0,0] 
            else:
                vector = [-self.L_vec_12[0], -self.L_vec_12[1]]
            return self.unit_vector(vector)

        elif (theta_13 == 0):

            # If the time-delta is in the same direction as the vector between mics, return the vector between mics
            if delta_t13 > 0:
                vector = self.L_vec_13
            elif delta_t13 == 0:
                vector = [0,0] 
            else:
                vector = [-self.L_vec_13[0], -self.L_vec_13[1]]
            return self.unit_vector(vector)

        v12a, v12b = self.findPossibleVectors(theta_12, self.L_vec_12)
        v13a, v13b = self.findPossibleVectors(theta_13, self.L_vec_13)

        vector = self.findCommonVector(v12a, v12b, v13a, v13b)
        return vector

    def parseWaveform(self, m1Waveform, m2Waveform, m3Waveform, time_per_step):
        
        # Create a 2xn array of the timestep at which each local-maximum was seen, preceded by an index local to this array
        marray1 = self.createPeakArray(m1Waveform)
        marray2 = self.createPeakArray(m2Waveform)
        marray3 = self.createPeakArray(m3Waveform)

        # Compare the difference in arrival time of peaks, and take an average
        delta_t12 = self.takeArrayDifference(marray1, marray2, time_per_step)
        delta_t13 = self.takeArrayDifference(marray1, marray3, time_per_step)

        # Using the time differences in arrival obtained above, decrypt to get the direction-of-arrival vector, and return it
        vector = self.decrypt(delta_t12, delta_t13)
        return vector


    def createPeakArray(self, waveform):
        marray = np.zeros([1,2])
        count = 0
        twoprior = 0
        prior = 0
        current = 0
        for i in range(len(waveform)):
            if (i == 0):
                current = waveform[i]
            elif (i == 1):
                prior   = current
                current = waveform[i]
            else:
                twoprior = prior
                prior = current
                current = waveform[i]
            
            if ( twoprior < prior and prior > current):
                # Prior was a peak
                marray = np.concatenate((marray, np.array([[count,i-1]])), axis=0)
                count += 1

        marray = marray[1:].copy()
        return marray

    
    def takeArrayDifference(self, marrayA, marrayB, time_per_step):
        minLength = min(len(marrayA), len(marrayB))
        diffarray = np.empty(0)

        for i in range(minLength):
            difference = marrayB[i,1] - marrayA[i,1]
            diffarray = np.append(diffarray, difference)

        average = sum(diffarray)/len(diffarray)
        delta_tAB = average * time_per_step
        return delta_tAB
        


#self.rec_path_taken = np.concatenate((self.rec_path_taken, np.array([[self.odom.x + START_X,self.odom.y + START_Y]])), axis=0)


    def findPossibleVectors(self, theta, L_vec):
        
        # Calculate two possible vectors, based on theta and minus-theta
        vpossible_a = self.calculateRotatedNormalizedVector(theta, L_vec)
        vpossible_b = self.calculateRotatedNormalizedVector(-theta, L_vec)
        return vpossible_a, vpossible_b

    def calculateRotatedNormalizedVector(self, theta, L_vec):

        # Apply rotation matrix to turn the vector
        # Acknowledgement: Revised rotation matrix based on Wikipedia:
        # https://en.wikipedia.org/wiki/Rotation_matrix
        rotatedX = L_vec[0]*np.cos(theta) - L_vec[1]*np.sin(theta)
        rotatedY = L_vec[0]*np.sin(theta) + L_vec[1]*np.cos(theta)
        rotatedVector = [rotatedX, rotatedY]
        rotatedNormalizedVector = self.unit_vector(rotatedVector)
        return rotatedNormalizedVector

    def unit_vector(self, vector):
        return vector / np.linalg.norm(vector)

    def findCommonVector(self, v1a, v1b, v2a, v2b):
        #print "Vector v1a: ", v1a
        #print "Vector v1b: ", v1b
        #print "Vector v2a: ", v2a
        #print "Vector v2b: ", v2b
        tolerance = VECTOR_MATCH_TOLERANCE
        if (self.findDifference(v1a, v2a) < tolerance):
            result = v1a
        elif (self.findDifference(v1a, v2b) < tolerance):
            result = v1a
        elif (self.findDifference(v1b, v2a) < tolerance):
            result = v1b
        elif (self.findDifference(v1b, v2b) < tolerance):
            result = v1b
        else:
            result = None
        return result

    def findDifference(self, v1, v2):
        x_difference = v1[0]-v2[0]
        y_difference = v1[1]-v2[1]
        magnitude = math.sqrt(x_difference**2 + y_difference**2)
        return magnitude

    def findDistance(self, x1, y1, x2, y2):
        distance = math.sqrt((x2-x1)**2 + (y2-y1)**2)
        return distance

    def findVector(self, x1, y1, x2, y2):
        vector = [x2-x1, y2-y1]
        return vector





if __name__ == "__main__":
    rospy.init_node("MicReceiverWaveform") 
    m = MicReceiverWaveform()
