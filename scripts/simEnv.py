#!/usr/bin/env python

# Team Acoustic
# CS 69-169 Winter 2020
#
# Acoustic DoA
# simEnv.py
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
import MicReceiverWaveform as mrw


# SPEAKER PARAMETERS
SPEAKER_A_X = 75
SPEAKER_A_Y = 75
SPEAKER_B_X = 75
SPEAKER_B_Y = 25

# ROOM PARAMETERS
ROOM_WIDTH  = 100
ROOM_HEIGHT = 100

# MICROPHONE PARAMETERS
M1_X = 50
M1_Y = 50
M2_X = 48
M2_Y = 50
M3_X = 49
M3_Y = 51

# SOUND PARAMETERS
SPEED_OF_SOUND = 343        # meters per second (truly 343 m/s)
RATIO_TIME_STEP = 0.01      # User-defined ratio which expresses the number of steps of time there should be, relative to the amount of time it takes to travel between the nearest pair of microphones. This quantity fixes a time-step-per-point. Essentially a measure of granularity. Sets an upper-bound on frequency.
NUM_POINTS_PER_PEAK = 16    # User-defined proxy for frequency. Increasing this value will decrease frequency (curve will take NUM_POINTS_PER_PEAK timesteps to complete a wavelength).


# Simulation Environment Class
# This class provides a simulation environment through which to test angle-of-arrival of a triangularly-configured array of 3 microphones. It creates a room whose dimensions are specified by the Room Parameters, and places speakers and microphones
# as described by the Speaker Parameters and Microphone Parameters. For each speaker and each pair of microphones, it calculates the expected time-difference-of-arrival (TDOA) between those microphones of the speaker's sound.  
# It generates waveforms which are sent to the microphones as ROS messages, with the suitable delay built in to the waveform. It then receives the microphone array's estimation of the direction-of-arrival (DOA) of each speaker's sound, and
# compares to the ground truth which it knows. This information is displayed to the user. 
class simEnv:
    def __init__(self):

        # Pull the parameters into instance variables
        self.room_width = ROOM_WIDTH
        self.room_height = ROOM_HEIGHT
        self.speed_of_sound = SPEED_OF_SOUND
        self.sA_x = SPEAKER_A_X
        self.sA_y = SPEAKER_A_Y
        self.sA_name = "A"
        self.sA_num_points_per_peak = NUM_POINTS_PER_PEAK
        self.sB_x = SPEAKER_B_X
        self.sB_y = SPEAKER_B_Y
        
        self.m1_x = M1_X
        self.m1_y = M1_Y
        self.m2_x = M2_X
        self.m2_y = M2_Y
        self.m3_x = M3_X
        self.m3_y = M3_Y


        # Construct vectors from mic1 to other mics
        self.L_vec_12 = self.findVector(self.m1_x, self.m1_y, self.m2_x, self.m2_y)
        self.L_vec_13 = self.findVector(self.m1_x, self.m1_y, self.m3_x, self.m3_y)

        # Find distance between the mics
        self.L_mag_12 = self.findDistance(self.m1_x, self.m1_y, self.m2_x, self.m2_y)
        self.L_mag_13 = self.findDistance(self.m1_x, self.m1_y, self.m3_x, self.m3_y)

        print "\n--- Simulation Environment ---\n"

        print "\nVectors from Microphone 1 to other microphones"
        print "Vector from mic 1 to mic 2:", self.L_vec_12
        print "Distance                  :", self.L_mag_12
        print "Vector from mic 1 to mic 3:", self.L_vec_13
        print "Distance                  :", self.L_mag_13

        # STUFF BELOW - SHOULD WE PACKAGE INTO A FUNCTION

        self.RunSimulation(self.sA_name, self.sA_x, self.sA_y, self.sA_num_points_per_peak)

    def RunSimulation(self, s_name, sloc_x, sloc_y, num_points_per_peak):

        # Construct vector from the known location of the speaker to mic1 (mic1 is considered origin for the speaker array)
        S_vec = self.unit_vector(self.findVector(sloc_x, sloc_y, self.m1_x, self.m1_y))

        print "\n--- Running simulation for Speaker",s_name, "---\n"
        print "\nVectors from Speaker to Microphone 1"
        print "Vector: ", S_vec

        delta_t12 = self.CalculateDeltaT(self.L_vec_12, self.L_mag_12, S_vec)
        delta_t13 = self.CalculateDeltaT(self.L_vec_13, self.L_mag_13, S_vec)

        print "\n Given the above inputs, the simulator environment is generating the following expected time-difference-of-arrival (TDOA) of mic2 relative to mic1, and mic3 relative to mic1"
        print "TDOA at mic2 relative to mic1:", delta_t12
        print "TDOA at mic3 relative to mic1:", delta_t13
    
        # Call MicReceiver to decrypt the TDOAs
        return_vector = mrw.MicReceiverWaveform().decrypt(delta_t12, delta_t13)
        print "\nBy directly passing the TDOAs to the MicReceiverWaveform's decrypt function, the following result was obtained"
        print "Arrival vector: ",return_vector
        print "Error vector:   ",self.findVector(return_vector[0], return_vector[1], S_vec[0], S_vec[1])

        # Generate the time-shifted waveforms
        m1Waveform, m2Waveform, m3Waveform, time_per_step = self.GenerateWaveformMaster(s_name, NUM_POINTS_PER_PEAK, self.speed_of_sound, delta_t12, delta_t13, self.L_mag_12, self.L_mag_13, RATIO_TIME_STEP)

        # Call MicReceiverWaveform to decrypt the waveforms
        return_vector_from_waveforms = mrw.MicReceiverWaveform().parseWaveform(m1Waveform, m2Waveform, m3Waveform, time_per_step)
        if return_vector_from_waveforms is None:
            print "Failed to produce definitive answer."
        else:
            print "\nBy passing the waveforms to the MicReceiverWaveform's parseWaveform function, the following result was obtained"
            print "Arrival vector: ",return_vector_from_waveforms
            print "Error vector:   ",self.findVector(return_vector_from_waveforms[0], return_vector_from_waveforms[1], S_vec[0], S_vec[1])

 


    ########################### Helper functions ####################################

    def findVector(self, x1, y1, x2, y2):
        vector = [x2-x1, y2-y1]
        return vector

    # CalculateDeltaT
    # Input: vector_m must be vector from microphone a to b
    # Output: time delay at microphone b with respect to a
    def CalculateDeltaT(self, vector_m, length_m, vector_s):

        theta = self.angle_between(vector_m, vector_s)
        
        if (theta < math.pi/2):
            delta_t_ab = length_m * np.cos(theta) / self.speed_of_sound
        if (theta > math.pi/2):
            delta_t_ab = - (length_m * np.cos(theta) / self.speed_of_sound)
        if (theta == math.pi/2):
            delta_t_ab = 0

        return delta_t_ab   # Means b with respect to a

    # --- Supporting functions for CalculateDeltaT ---

    # Acknowledgement: The set of functions to calculate theta (the angle between the two vectors) is inspired by  
    # David Wolever's writeup, in the following source:
    # https://stackoverflow.com/questions/2827393/angles-between-two-n-dimensional-vectors-in-python/13849249#13849249

    def unit_vector(self, vector):
        return vector / np.linalg.norm(vector)

    def angle_between(self, v1, v2):
        v1_unit = self.unit_vector(v1)
        v2_unit = self.unit_vector(v2)
        return np.arccos(np.clip(np.dot(v1_unit, v2_unit), -1.0, 1.0))      # Clip clips the values at the minimum (-1.0) and maximum (1.0)


    def findDistance(self, x1, y1, x2, y2):
        distance = math.sqrt((x2-x1)**2 + (y2-y1)**2)
        return distance

    # --- Unused Functions ---

    def findArctanAngle(self, op1, op2, ad1, ad2):
        angle = np.arctan2(abs(op1 - op2), abs(ad1 - ad2))
        return angle

        
    # not num_peaks

    # I want this frequency 
    # time per step
    # Number of points per peak
    # 16
    # print: transmitting at X Hz
    # 
    

    def CalcTimePerStep(self, L_mag_12, L_mag_13, speed_of_sound, ratio):
        time_per_step = min(L_mag_12, L_mag_13) / speed_of_sound * ratio
        return time_per_step

    def CalcMinNumPoints(self, delta_t12, delta_t13, time_per_step):
        total_number_of_points = (int)(10 * math.floor(max(delta_t12, delta_t13) / time_per_step))
        return total_number_of_points

    def CalculateFrequency(self, num_points_per_peak, time_per_step):
        frequency = 1/(num_points_per_peak * time_per_step)
        return frequency

    def GenerateWaveform(self, total_number_of_points, num_points_per_peak):
        array = np.empty(0)
        for i in range(total_number_of_points):
            value = np.sin(i * 2*math.pi / num_points_per_peak)
            array = np.append(array, value)
        return array

    def PrintInfo(self, speaker_ID, frequency, speed_of_sound, num_points_per_peak, delta_t12, delta_t13, time_per_step, total_number_of_points):
        print "Generating waveforms for each microphone with the following parameters"
        print "Speaker_ID:                      ",speaker_ID
        print "Speed of sound:                  ",speed_of_sound
        print "TDOA of mic2 vs. mic1:           ",delta_t12
        print "TDOA of mic3 vs. mic1:           ",delta_t13
        print "Time per step:                   ",time_per_step
        print "Number of points per peak:       ",num_points_per_peak
        print "Frequency (Implicit):            ",frequency
        print "Total number of points in array: ",total_number_of_points

    def GenerateWaveformMaster(self, speaker_ID, num_points_per_peak, speed_of_sound, delta_t12, delta_t13, L_mag_12, L_mag_13, ratio):
        time_per_step           = self.CalcTimePerStep(L_mag_12, L_mag_13, speed_of_sound, ratio)
        total_number_of_points  = self.CalcMinNumPoints(delta_t12, delta_t13, time_per_step)
        frequency               = self.CalculateFrequency(num_points_per_peak, time_per_step)
        nativeWaveform          = self.GenerateWaveform(total_number_of_points, num_points_per_peak)

        self.PrintInfo(speaker_ID, frequency, speed_of_sound, num_points_per_peak, delta_t12, delta_t13, time_per_step, total_number_of_points)
        
        # Timeshifting
        # Temporary time-deltas for the purpose of time-delay adjustment
        delta_tn1 = 0
        delta_tn2 = delta_t12
        delta_tn3 = delta_t13

        # Handle the cases that the sound reaches microphones 2 or 3 first. 
        # Solely for the purpose of transforming the waveform, slide the time delays over such that the earliest-receiving mic gets a zero-delay 
        if (delta_tn2 < 0 or delta_tn3 < 0):
            adjustment_value = abs(min(delta_tn2, delta_tn3))
            delta_tn1 += adjustment_value
            delta_tn2 += adjustment_value
            delta_tn3 += adjustment_value

        m1Waveform = self.TimeshiftWaveform(nativeWaveform, delta_tn1, time_per_step, total_number_of_points)
        m2Waveform = self.TimeshiftWaveform(nativeWaveform, delta_tn2, time_per_step, total_number_of_points)
        m3Waveform = self.TimeshiftWaveform(nativeWaveform, delta_tn3, time_per_step, total_number_of_points)

        # Output the waveforms for viewing
        #print "Native waveform without adjustment: \n",nativeWaveform
        # plt.figure(1)
        # array_time = np.empty(0)
        # for i in range(total_number_of_points):
        #     array_time = np.append(array_time, i * time_per_step)
        # print "Length, array_time",len(array_time)
        # print "Length, m1Waveform",len(m1Waveform)
        # print "Length, m2Waveform",len(m2Waveform)
        # print "Length, m3Waveform",len(m3Waveform)
        # print "10th value, m1", m1Waveform[9]
        # print "10th value, m2", m2Waveform[9]
        # print "10th value, m3", m3Waveform[9]

        # plt.plot(array_time,m1Waveform, label='m1Waveform')
        # plt.plot(array_time,m2Waveform, label='m2Waveform')
        # plt.plot(array_time,m3Waveform, label='m3Waveform')
        # plt.show()

        return m1Waveform, m2Waveform, m3Waveform, time_per_step


    def TimeshiftWaveform(self, inputWaveform, delta_t, time_per_step, total_number_of_points):

        if delta_t == 0:
            return inputWaveform
        num_steps_delay = (int)(delta_t / time_per_step)
        timeshiftedWaveform = np.zeros(num_steps_delay)
        copytemp = inputWaveform[:-(num_steps_delay)].copy()
        timeshiftedWaveform = np.append(timeshiftedWaveform, copytemp)

        # timeshiftedWaveform = inputWaveform
        # for i in range(num_steps_delay):
        #     np.insert(timeshiftedWaveform,0,0)
        #     timeShiftedWaveform = timeshiftedWaveform[:-1].copy()

        return timeshiftedWaveform

   

    
    



if __name__ == "__main__":
    rospy.init_node("SimulationEnvironment") 
    s = simEnv()
    