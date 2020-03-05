#!/usr/bin/env python

# Team Acoustic
# CS 69-169 Winter 2020
#
# Acoustic DoA
# MicReceiver.py
# Mack Reiferson and Siddharth Agrawal
#
# Module which simulates the 3 microphone array.
# This module can find TDOAs between three microphones given the microphones' received waveforms.
# Then, using these TDOAs, this module can calculate the estimated vectors from the microphone array to a speaker.

import rospy
import numpy as np
import math

# SET TOLERANCE FOR MATCHING
VECTOR_MATCH_TOLERANCE = 0.5

# MicReceiverWaveform Class
# This class can parse received waveforms to calculate TDOAs relative to the first microphone which receives the sound.
# Using these TDOAs, this class can then derice the DOAs of the sound (from a speaker to the microphone array).
class MicReceiverWaveform:
    def __init__(self, M1_X, M1_Y, M2_X, M2_Y, M3_X, M3_Y, SPEED_OF_SOUND):
        # Microphone Parameters
        self.m1_x = M1_X
        self.m1_y = M1_Y
        self.m2_x = M2_X
        self.m2_y = M2_Y
        self.m3_x = M3_X
        self.m3_y = M3_Y

        # Speed of sound
        self.speed_of_sound = SPEED_OF_SOUND

        print "\n--- Starting MicReceiverWaveform execution ---"

        # Calculate vectors from m1 to both m2 and m3
        self.L_vec_12 = self.findVector(self.m1_x, self.m1_y, self.m2_x, self.m2_y)
        self.L_vec_13 = self.findVector(self.m1_x, self.m1_y, self.m3_x, self.m3_y)
        
        # Calculate the magnitude of the vectors from m1 to both m2 and m3
        self.L_mag_12 = self.findDistance(self.m1_x, self.m1_y, self.m2_x, self.m2_y)
        self.L_mag_13 = self.findDistance(self.m1_x, self.m1_y, self.m3_x, self.m3_y)

    # The decrypt function takes two TDOAs relative to one microphone, and returns a vector to the speaker of origin.
    def decrypt(self, delta_t12, delta_t13):
        
        # Calculates angles of incidence of the sound vector with the vector between two given microphones
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

        # Given the thetas, calculate what possible vectors these angles may really be representing
        v12a, v12b = self.findPossibleVectors(theta_12, self.L_vec_12)
        v13a, v13b = self.findPossibleVectors(theta_13, self.L_vec_13)

        # Determine the correct vector and return
        vector = self.findCommonVector(v12a, v12b, v13a, v13b)
        return vector

    # The parseWaveform function takes three waveforms (one per mic) and the time per step (which represents how much time passes
    # between each value in the Waveform array), and returns the DOA vector
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

    # This function takes a waveform as input and returns a a 2xn array of the timestep at which each local-maximum was seen, 
    # preceded by an index local to this array
    def createPeakArray(self, waveform):
        
        # Initialize array and necessary variables
        marray = np.zeros([1,2])
        count = 0
        twoprior = 0
        prior = 0
        current = 0

        # Iterate over waveform
        for i in range(len(waveform)):

            if (i == 0):
                # Handle the first point in the waveform
                current = waveform[i]
            elif (i == 1):
                # Handle the second point in the waveform
                prior   = current
                current = waveform[i]
            else:
                # Handle the rest of the wave
                twoprior = prior
                prior = current
                current = waveform[i]
            
            # Find peaks and add them to the array, increasing
            if ( twoprior < prior and prior > current):
                # Prior was a peak
                marray = np.concatenate((marray, np.array([[count,i-1]])), axis=0)
                count += 1
        
        # Eliminate the first element and return
        marray = marray[1:].copy()
        return marray

    # This function calculates the TDOA between to microphones as an average over all peaks found in each waveform received    
    def takeArrayDifference(self, marrayA, marrayB, time_per_step):

        # Initialize necessary variables
        minLength = min(len(marrayA), len(marrayB))
        diffarray = np.empty(0)

        # Iterate over the arrays
        for i in range(minLength):

            # Calculate the difference between each array, and append into the diffarray for averaging later
            difference = marrayB[i,1] - marrayA[i,1]
            diffarray = np.append(diffarray, difference)

        # Take the average difference and multiply it by the unit definition, then return
        average = sum(diffarray)/len(diffarray)
        delta_tAB = average * time_per_step
        return delta_tAB
        
    # This function takes a theta and a vector and returns two potential DOAs
    def findPossibleVectors(self, theta, L_vec):
        
        # Calculate two possible vectors, based on theta and minus-theta
        vpossible_a = self.calculateRotatedNormalizedVector(theta, L_vec)
        vpossible_b = self.calculateRotatedNormalizedVector(-theta, L_vec)
        return vpossible_a, vpossible_b

    # This function takes a theta and vector and returns a vector rotated by theta and then normalized
    def calculateRotatedNormalizedVector(self, theta, L_vec):

        # Apply rotation matrix to turn the vector
        # Acknowledgement: Revised rotation matrix based on Wikipedia:
        # https://en.wikipedia.org/wiki/Rotation_matrix

        # Rotate the vector by theta
        rotatedX = L_vec[0]*np.cos(theta) - L_vec[1]*np.sin(theta)
        rotatedY = L_vec[0]*np.sin(theta) + L_vec[1]*np.cos(theta)
        rotatedVector = [rotatedX, rotatedY]

        # Normalize and return
        rotatedNormalizedVector = self.unit_vector(rotatedVector)
        return rotatedNormalizedVector

    # This function takes a vector and returns a normalized vector
    def unit_vector(self, vector):
        return vector / np.linalg.norm(vector)

    # This function takes in four potential vectors and finds two which match
    def findCommonVector(self, v1a, v1b, v2a, v2b):
        
        # Set the tolerance
        tolerance = VECTOR_MATCH_TOLERANCE

        # DEBUG STATEMENTS
        #print "v1a: ",v1a
        #print "v1b: ",v1b
        #print "v2a: ",v2a
        #print "v2b: ",v2b

        # Check each potential pair and return the correct vector
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

    # This function returns the difference in magnitude between two vectors
    def findDifference(self, v1, v2):
        x_difference = v1[0]-v2[0]
        y_difference = v1[1]-v2[1]
        magnitude = math.sqrt(x_difference**2 + y_difference**2)
        return magnitude

    # This function finds the distance between two points
    def findDistance(self, x1, y1, x2, y2):
        distance = math.sqrt((x2-x1)**2 + (y2-y1)**2)
        return distance

    # This function finds a vector from two points
    def findVector(self, x1, y1, x2, y2):
        vector = [x2-x1, y2-y1]
        return vector
