#!/usr/bin/env python
"""
    localizer_simulation_driver.py

    Author: Vlado ...
    Mar. 3rd 2020

    A simulation driver for the acoustic localizer.

    DONE:  
        * Randomly genreates an environment with speaker positions 
        upon initialization. 
        * Sends speaker position list to appropiate topic (/speakerpositions) at regular intervals
            > Also sends visualization_msgs/Marker to rviz so we can see speaker positions

    TO DO:
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
from visualization_msgs.msg import Marker, MarkerArray
from acousticlocalization.msg import SpeakerPosition, SpeakerPositionList, Sound2DDoA, Sound2DDoAFrame

class localizer_simulation_driver():
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.center_x = width/2
        self.center_y = height/2

        self.speaker_list = []

        self.cx_to_w = self.width
        self.cy_to_w = self.height
        self.vect_cent_to_left = np.array([self.width,0])

        self.speaker_count = 1
        self.marker_array = MarkerArray()

#    def create_robot(self):
            

    def create_speakers(self,c):
        for i in range(0,c):
      	    speaker = SpeakerPosition()
            speaker.speakerId = self.speaker_count 
            self.speaker_count += 1

	    speaker.position.x = random.randint(1,100)
            speaker.position.y = random.randint(1,100)
            vect_to_cent = np.array([self.center_x - speaker.position.x, self.center_y - speaker.position.y])

            dot_prod = np.dot(vect_to_cent, self.vect_cent_to_left)
            vtc_mag = math.sqrt(pow(self.center_x - speaker.position.x, 2) + pow(self.center_y - speaker.position.y, 2)) * self.cx_to_w
            
            theta = math.acos(dot_prod/vtc_mag)
            speaker.position.z = theta
            self.speaker_list.append(speaker)
            
            self.create_marker(speaker)


    def create_marker(self, speaker):
        red = random.random()
        blue = random.random()
        green = random.random()
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.header.stamp = rospy.get_time()
        marker.type = marker.SPHERE
        marker.action = marker.ADD

        marker.id = speaker.speakerId

        marker.scale.x = 1
        marker.scale.y = 1
        marker.scale.z = 1

        marker.color.a = 1
        marker.color.r = red
        marker.color.b = blue
        marker.color.g = green

        marker.pose.orientation.x = math.cos(speaker.position.z)
        marker.pose.orientation.y = math.sin(speaker.position.z)
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1

        marker.pose.position.x = speaker.position.x
        marker.pose.position.y = speaker.position.y
        marker.pose.position.z = 0
        
        self.marker_array.markers.append(marker)
       

if __name__=="__main__":
    rospy.init_node('simulation_al')
    speaker_pos_publisher = rospy.Publisher('speaker_positions', SpeakerPositionList, queue_size=10)
    speaker_marker_pub = rospy.Publisher('speaker_visualization_array', MarkerArray, queue_size=10)
    rospy.sleep(1)
    
    rate = rospy.Rate(3) # 3 Hz
    new_room = localizer_simulation_driver(100,100)
    new_room.create_speakers(3)

    while not rospy.is_shutdown():
        speaker_pos_publisher.publish(new_room.speaker_list)
        rospy.loginfo(new_room.marker_array)
        speaker_marker_pub.publish(new_room.marker_array)
        rate.sleep()        
