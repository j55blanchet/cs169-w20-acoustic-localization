#!/usr/bin/env python

# Team Acoustic
# CS 69-169 Winter 2020
# 
# Acoustic DoA
# simEnv.py
# Mack Reiferson and Siddharth Agrawal
#
# This class provides a simulation environment through which to test angle-of-arrival of a triangularly-configured array of 3 microphones. It creates a room whose dimensions are specified by the Room Parameters, and places speakers and microphones
# as described by the Speaker Parameters and Microphone Parameters. For each speaker and each pair of microphones, it calculates the expected time-difference-of-arrival (TDOA) between those microphones of the speaker's sound.  
# It generates waveforms which are sent to the microphones as ROS messages, with the suitable delay built in to the waveform. It then receives the microphone array's estimation of the direction-of-arrival (DOA) of each speaker's sound, and
# compares to the ground truth which it knows. This information is displayed to the user. 
# 

import rospy
import numpy as np
from acousticlocalization.msg import Sound2DDoA, Sound2DDoAFrame, SpeakerPositionList
from geometry_msgs.msg import Point
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math
import MicReceiverWaveform as mrw


# SPEAKER PARAMETERS
SPEAKER_A_X = 90
SPEAKER_A_Y = 80
SPEAKER_B_X = 75
SPEAKER_B_Y = 25
SPEAKER_C_X = 40
SPEAKER_C_Y = 30

# ROOM PARAMETERS
ROOM_WIDTH  = 100
ROOM_HEIGHT = 100

# MICROPHONE PARAMETERS
M1_X = 50
M1_Y = 50

M12_X = -2
M12_Y = 0
M13_X = -1
M13_Y = 1


# SOUND PARAMETERS
SPEED_OF_SOUND = 343        # meters per second (truly 343 m/s)
RATIO_TIME_STEP = 0.01      # User-defined ratio which expresses the number of steps of time there should be, relative to the amount of time it takes to travel between the nearest pair of microphones. This quantity fixes a time-step-per-point. Essentially a measure of granularity. Sets an upper-bound on frequency.

# NUM_POINTS_PER_PEAK
# User-defined proxy for frequency. Increasing this value will decrease frequency (curve will take NUM_POINTS_PER_PEAK timesteps to complete a wavelength).
NUM_POINTS_PER_PEAK_A = 16    
NUM_POINTS_PER_PEAK_B = 24    
NUM_POINTS_PER_PEAK_C = 32    

# Simulation Environment Class
class simEnv:
    def __init__(self):

        # # Publishers
        self.s_pos_pub = rospy.Publisher("acoustic/speaker_positions", SpeakerPositionList, queue_size=0)       # Publisher for speaker positions (not known by microphone array)
        self.mic_doas_pub = rospy.Publisher("acoustic/doas", Sound2DDoAFrame, queue_size=0)                     # Publisher for DOA's calculated by microphone array

        # Calculate M2 and M3 positions, using positions relative to M1
        M2_X = M1_X + M12_X
        M2_Y = M1_Y + M12_Y
        M3_X = M1_X + M13_X
        M3_Y = M1_Y + M13_Y

        # Room and sound parameters
        self.room_width = ROOM_WIDTH
        self.room_height = ROOM_HEIGHT
        self.speed_of_sound = SPEED_OF_SOUND
        
        # Speaker A Parameters
        self.sA_x = SPEAKER_A_X
        self.sA_y = SPEAKER_A_Y
        self.sA_name = "A"
        self.sA_num_points_per_peak = NUM_POINTS_PER_PEAK_A
        
        # Speaker B Parameters
        self.sB_x = SPEAKER_B_X
        self.sB_y = SPEAKER_B_Y
        self.sB_name = "B"
        self.sB_num_points_per_peak = NUM_POINTS_PER_PEAK_B

        # Speaker C Parameters
        self.sC_x = SPEAKER_C_X
        self.sC_y = SPEAKER_C_Y
        self.sC_name = "C"
        self.sC_num_points_per_peak = NUM_POINTS_PER_PEAK_C
        
        # Microphone Parameters
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

        # Print summary of simulation environment
        print "\n--- Simulation Environment ---\n"

        print "\nVectors from Microphone 1 to other microphones"
        print "Vector from mic 1 to mic 2:", self.L_vec_12
        print "Distance                  :", self.L_mag_12
        print "Vector from mic 1 to mic 3:", self.L_vec_13
        print "Distance                  :", self.L_mag_13

        # Run the simulation for speakers A-C, obtaining the DOA vectors (this function calls the Microphone module)
        return_vector_A = self.RunSimulation(self.sA_name, self.sA_x, self.sA_y, self.sA_num_points_per_peak)
        return_vector_B = self.RunSimulation(self.sB_name, self.sB_x, self.sB_y, self.sB_num_points_per_peak)
        return_vector_C = self.RunSimulation(self.sC_name, self.sC_x, self.sC_y, self.sC_num_points_per_peak)

        # # Construct array of DOAs and speaker positions
        message_doa = self.ConstructSound2DDoAFrame(return_vector_A, return_vector_B, return_vector_C)
        point_A = self.GeneratePoint(self.sA_x, self.sA_y)
        point_B = self.GeneratePoint(self.sB_x, self.sB_y)
        point_C = self.GeneratePoint(self.sC_x, self.sC_y)

        s_pos = SpeakerPositionList()
        s_pos.positions = [point_A, point_B, point_C]

        # # Publish the results for use in localization
        self.mic_doas_pub.publish(message_doa)
        self.s_pos_pub.publish(s_pos)


    ### PUBLISHING FUNCTIONS ###

    # Generates a geometry_msgs Point from a given x and y
    def GeneratePoint(self, x, y):
        point = Point()
        point.x = x
        point.y = y
        return point

    # Generates the custom message Sound2DDoA for composition into Sound2DDoAFrame
    def ConstructSound2DDoA(self, vector, ID):
        angle = np.arctan2( - vector[1], - vector[0])
        doa_msg = Sound2DDoA()
        doa_msg.angle = angle
        doa_msg.sourceId = ID
        return doa_msg

    # Generates the custom message Sound2DDoAFrame for passing to Localization Module
    def ConstructSound2DDoAFrame(self, return_vector_A, return_vector_B, return_vector_C):
        Sound2DDoA_0 = self.ConstructSound2DDoA(return_vector_A, 0)
        Sound2DDoA_1 = self.ConstructSound2DDoA(return_vector_B, 1)
        Sound2DDoA_2 = self.ConstructSound2DDoA(return_vector_C, 2)

        doa_frame = Sound2DDoAFrame()
        doa_frame.header.frame_id = "map"
        doa_frame.doas = [Sound2DDoA_0, Sound2DDoA_1, Sound2DDoA_2]
        return doa_frame

    ### SIMULATION FUNCTION ###

    # Primary driver function for the simulation, for one speaker
    # Generates vectors from speakers to microphone array, and then TDOA's and waveforms. Calls the Microphone Module (MicReceiverWaveform) to solve the TDOAs and waveforms back to the speaker DOAs, and returns them to the caller
    def RunSimulation(self, s_name, sloc_x, sloc_y, num_points_per_peak):

        # Construct vector from the known location of the speaker to mic1 (mic1 is considered origin for the speaker array)
        S_vec = self.unit_vector(self.findVector(sloc_x, sloc_y, self.m1_x, self.m1_y))

        print "\n--- Running simulation for Speaker",s_name, "---\n"
        print "\nVectors from Speaker to Microphone 1"
        print "Vector: ", S_vec

        # Calculate TDOAs
        delta_t12 = self.CalculateDeltaT(self.L_vec_12, self.L_mag_12, S_vec)
        delta_t13 = self.CalculateDeltaT(self.L_vec_13, self.L_mag_13, S_vec)

        # Print TDOAs
        print "\n Given the above inputs, the simulator environment is generating the following expected time-difference-of-arrival (TDOA) of mic2 relative to mic1, and mic3 relative to mic1"
        print "TDOA at mic2 relative to mic1:", delta_t12
        print "TDOA at mic3 relative to mic1:", delta_t13
    
        # Call MicReceiver to decrypt the TDOAs (skips generating waveforms and should produce near-perfect results)
        return_vector = mrw.MicReceiverWaveform().decrypt(delta_t12, delta_t13)
        print "\nBy directly passing the TDOAs to the MicReceiverWaveform's decrypt function, the following result was obtained"
        print "Arrival vector: ",return_vector
        print "Error vector:   ",self.findVector(return_vector[0], return_vector[1], S_vec[0], S_vec[1])

        # Generate the time-shifted waveforms
        m1Waveform, m2Waveform, m3Waveform, time_per_step = self.GenerateWaveformMaster(s_name, num_points_per_peak, self.speed_of_sound, delta_t12, delta_t13, self.L_mag_12, self.L_mag_13, RATIO_TIME_STEP)

        # Call MicReceiverWaveform to take and parse the waveforms, and then internally call decrypt (should produce near but slightly less perfect results)
        return_vector_from_waveforms = mrw.MicReceiverWaveform().parseWaveform(m1Waveform, m2Waveform, m3Waveform, time_per_step)
        if return_vector_from_waveforms is None:
            print "Failed to produce definitive answer."
        else:
            print "\nBy passing the waveforms to the MicReceiverWaveform's parseWaveform function, the following result was obtained"
            print "Arrival vector: ",return_vector_from_waveforms
            print "Error vector:   ",self.findVector(return_vector_from_waveforms[0], return_vector_from_waveforms[1], S_vec[0], S_vec[1])

        # Return the DOA vector obtained from parsing the waveforms
        return return_vector_from_waveforms


    # --- CalculateDeltaT and its supporting functions ---


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

    # Acknowledgement: The set of functions to calculate theta (the angle between the two vectors) is inspired by  
    # David Wolever's writeup, in the following source:
    # https://stackoverflow.com/questions/2827393/angles-between-two-n-dimensional-vectors-in-python/13849249#13849249

    # Find the angle between two vectors
    def angle_between(self, v1, v2):
        v1_unit = self.unit_vector(v1)
        v2_unit = self.unit_vector(v2)
        return np.arccos(np.clip(np.dot(v1_unit, v2_unit), -1.0, 1.0))      # Clip clips the values at the minimum (-1.0) and maximum (1.0)

    # Calculate unit vector
    def unit_vector(self, vector):
        return vector / np.linalg.norm(vector)

    # Calculates Euclidean distance
    def findDistance(self, x1, y1, x2, y2):
        distance = math.sqrt((x2-x1)**2 + (y2-y1)**2)
        return distance

    # Create vector
    def findVector(self, x1, y1, x2, y2):
        vector = [x2-x1, y2-y1]
        return vector


    # --- GenerateWaveformMaster and its supporting functions ---


    # Driver function for generating waveforms, each of which is adjusted by the time-difference-of-arrival relative to the microphone which should receive the signal first
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
        # plt.figure(1)
        # array_time = np.empty(0)
        # for i in range(total_number_of_points):
        #     array_time = np.append(array_time, i * time_per_step)

        # Debug print statements
        # print "Length, array_time",len(array_time)
        # print "Length, m1Waveform",len(m1Waveform)
        # print "Length, m2Waveform",len(m2Waveform)
        # print "Length, m3Waveform",len(m3Waveform)
        # print "10th value, m1", m1Waveform[9]
        # print "10th value, m2", m2Waveform[9]
        # print "10th value, m3", m3Waveform[9]

        # plt.subplot(3, 1, 1)
        # plt.plot(array_time,m1Waveform, label='m1Waveform')

        # plt.subplot(3, 1, 2)
        # plt.plot(array_time,m2Waveform, label='m2Waveform')

        # plt.subplot(3, 1, 3)
        # plt.plot(array_time,m3Waveform, label='m3Waveform')

        # plt.show()

        return m1Waveform, m2Waveform, m3Waveform, time_per_step
    
    # Returns the desired time per step for the user's selected level of granularity (ratio)
    def CalcTimePerStep(self, L_mag_12, L_mag_13, speed_of_sound, ratio):
        time_per_step = min(L_mag_12, L_mag_13) / speed_of_sound * ratio
        return time_per_step

    # Calculate the minimum number of points in a waveform to provide coverage across all microphones after time-delay
    def CalcMinNumPoints(self, delta_t12, delta_t13, time_per_step):
        total_number_of_points = (int)(10 * math.floor(max(delta_t12, delta_t13) / time_per_step))
        return total_number_of_points

    # Calculates the frequency implicit from the chosen num_points_per_peak and ratio
    def CalculateFrequency(self, num_points_per_peak, time_per_step):
        frequency = 1/(num_points_per_peak * time_per_step)
        return frequency

    # Generates a base waveform (sine-wave) from the total number of points, and number of points per peak 
    def GenerateWaveform(self, total_number_of_points, num_points_per_peak):
        array = np.empty(0)
        for i in range(total_number_of_points):
            value = np.sin(i * 2*math.pi / num_points_per_peak)
            array = np.append(array, value)
        return array

    # Print function for intermediate values of GenerateWaveformMaster, before timeshifting
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

    # Produces a waveform timeshifted by the given delta_t. For a given amount of space (total_number_of_points),
    # places zero-values ahead of the time-shift, and copies the beginning of the part of the base waveform into 
    # the remaning space after the time-shift 
    def TimeshiftWaveform(self, inputWaveform, delta_t, time_per_step, total_number_of_points):

        if delta_t == 0:
            return inputWaveform
        num_steps_delay = (int)(delta_t / time_per_step)
        timeshiftedWaveform = np.zeros(num_steps_delay)
        copytemp = inputWaveform[:-(num_steps_delay)].copy()
        timeshiftedWaveform = np.append(timeshiftedWaveform, copytemp)
        return timeshiftedWaveform



if __name__ == "__main__":
    rospy.init_node("SimulationEnvironment") 
    s = simEnv()
    