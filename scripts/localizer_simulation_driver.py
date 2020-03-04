#!/usr/bin/env python
"""
    localizer_simulation_driver.py

    Author: Vlado ...
    Mar. 3rd 2020

    A simulation driver for the acoustic localizer. 
    * Randomly genreates an environment with speaker positions 
    upon initialization. 
    * Sends speaker position list to appropiate topic (/speakerpositions) at regular intervals
        > Also sends visualization_msgs/Marker to rviz so we can see speaker positions
    * Simulates robot's position & sends calculated DoA measurements
        > Randomly initializes robot's position at start
        > Listen's to rviz's "clicked point" topic - when a new point is published, set's robot's position there and sends
        DoAFrame with updated DoA measurements
            > randomize orientation each time 

"""
import rospy
import random 
import math
import numpy as np
import numpy.linalg as lin
from acousticlocalization.msg import SpeakerPosition, SpeakerPositionList, Sound2DDoA, Sound2DDoAFrame

class localizer_simulation_driver():
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.center_x = width/2
        self.center_y = height/2
        self.speaker_list = []
        self.cx_to_w = x
        self.cy_to_w = y
        vect_cent_to_left = np.array([x,0])
    def create_speakers(c):
        for i in range(0,c):
      	    speaker = SpeakerPosition()
	    speaker.x = random.randint(1,10)
            speaker.y = random.randint(1,10)
            vect_to_cent = np.array([self.center_x - speaker.x, self.center_y - speaker.y])

            dot_prod = np.dot(vect_to_cent, vect_cent_to_left)
            vtc_mag = math.sqrt(pow(self.center_x - speaker.x, 2) + pow(self.center_y - speaker.y, 2))

            theta = math.acos(dot_prod/vtc_mag)
            speaker.z = theta
            self.speaker_list.append(speaker)

if __name__=="__main__":
    rospy.init_node('simulation_al')
    speaker_pos_publisher = rospy.Publisher('/speakerpositions', SpeakerPositionList, queue_size=10)
    rate = rospy.Rate(3) # 3 Hz
    new_room = Simulator_al(10,10)
    new_room.create_speakers(3)
    for i in range(0, new_room.speaker_list):
        speaker_pos_publisher.publish(new_room.speaker_list[i])
        rospy.loginfo(new_room.speaker_list[i])
        rate.sleep()
