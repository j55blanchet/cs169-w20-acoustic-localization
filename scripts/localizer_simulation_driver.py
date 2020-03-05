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

        * Simulates robot's position & sends calculated DoA measurements
            > Randomly initializes robot's position at start
            > randomize orientation each time 
        > Listen's to rviz's "clicked point" topic - when a new point is published, set's robot's position there and sends
        DoAFrame with updated DoA measurements
    ISSUES:
       * Rviz does not visualize speakers
       * Rviz clicked_point doesn't work
  

"""
import rospy
import random 
import math
import numpy as np
import numpy.linalg as lin
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped
from acousticlocalization.msg import SpeakerPosition, SpeakerPositionList, Sound2DDoA, Sound2DDoAFrame, RobotPosition

class localizer_simulation_driver():
    def __init__(self, width, height):
        #initialize basic environment variables
        self.width = width
        self.height = height
        self.center_x = width/2
        self.center_y = height/2
        
        #initialize a speaker list containing all their data
        self.speaker_list = []

        #Vectors that will be used for angle calculations later on
        self.cx_to_w = self.width
        self.cy_to_w = self.height
        self.vect_cent_to_left = np.array([self.width,0])

        #Initializing the robot position
        self.robot = RobotPosition()
        
        #initializing the Robot Marker
        self.robot_marker = Marker()

        #id-ing the speakers
        self.speaker_count = 1
        #markers associated with the speakers
        self.marker_array = MarkerArray()


#A function that creates our robot
                    
    def create_robot(self):

        self.robot.robotId = 7
        self.robot.position.x = random.randint(1,self.width)
        self.robot.position.y = random.randint(1,self.height)
        
        self.robot.angle = (random.random() * 2)*math.pi 
        
        # vector to center from our robot
        vect_to_cent = np.array([self.center_x - self.robot.position.x, self.center_y - self.robot.position.y])
        
        # dot product angle calculation
        dot_prod = np.dot(vect_to_cent, self.vect_cent_to_left)
        vtc_mag = math.sqrt(pow(self.center_x - self.robot.position.x, 2) + pow(self.center_y - self.robot.position.y, 2)) * self.cx_to_w
        
        theta = math.acos(dot_prod/vtc_mag)
        self.robot.position.z = theta


        self.robot_marker.header.frame_id = "/map"
        self.robot_marker.header.stamp = rospy.Time.now()
        self.robot_marker.type = self.robot_marker.SPHERE
        self.robot_marker.action = self.robot_marker.ADD

        self.robot_marker.id = self.robot.robotId

        self.robot_marker.scale.x = 1
        self.robot_marker.scale.y = 1
        self.robot_marker.scale.z = 1

        self.robot_marker.color.a = 1
        self.robot_marker.color.r = 0
        self.robot_marker.color.b = 1
        self.robot_marker.color.g = 0

        self.robot_marker.pose.orientation.x = math.cos(self.robot.angle)
        self.robot_marker.pose.orientation.y = math.sin(self.robot.angle)
        self.robot_marker.pose.orientation.z = 0
        self.robot_marker.pose.orientation.w = 1

        self.robot_marker.pose.position.x = self.robot.position.x
        self.robot_marker.pose.position.y = self.robot.position.y
        self.robot_marker.pose.position.z = 0
            

#A function that creates c speakers and calculates their angle to the robot
    def create_speakers(self,c):

        # creating c speakers
        for i in range(0,c):
      	    speaker = SpeakerPosition()
            speaker.speakerId = self.speaker_count 
            self.speaker_count += 1

	    speaker.position.x = random.randint(1,self.width)
            speaker.position.y = random.randint(1,self.height)
            vect_to_cent = np.array([self.center_x - speaker.position.x, self.center_y - speaker.position.y])

            dot_prod = np.dot(vect_to_cent, self.vect_cent_to_left)
            vtc_mag = math.sqrt(pow(self.center_x - speaker.position.x, 2) + pow(self.center_y - speaker.position.y, 2)) * self.cx_to_w
            
            theta = math.acos(dot_prod/vtc_mag)
            speaker.position.z = theta - self.robot.position.z
            self.speaker_list.append(speaker)
            
            self.create_marker(speaker)

#A function that converts our speaker data into Markers 
    def create_marker(self, speaker):
        red = random.random()
        blue = random.random()
        green = random.random()
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.header.stamp = rospy.Time.now()
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

        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1

        marker.pose.position.x = speaker.position.x
        marker.pose.position.y = speaker.position.y
        marker.pose.position.z = 0
        
        self.marker_array.markers.append(marker)

    def update(self,data):
        self.robot.position.x = data.point.x
        self.robot.position.y = data.point.y 
        self.robot_marker.pose.position.x = self.robot.position.x
        self.robot_marker.pose.position.y = self.robot.position.y
        
        vect_to_cent = np.array([self.center_x - self.robot.position.x, self.center_y - self.robot.position.y])
        dot_prod = np.dot(vect_to_cent, self.vect_cent_to_left)
        vtc_mag = math.sqrt(pow(self.center_x - self.robot.position.x, 2) + pow(self.center_y - self.robot.position.y, 2)) * self.cx_to_w
        new_theta = math.acos(dot_prod/vtc_mag)
        delta_theta = self.robot.position.z - new_theta
        self.robot.position.z = new_theta

        self.update_DOA(delta_theta)
 
    def update_DOA(self, delta_theta):
        for i in range(0, len(self.speaker_list)):
            self.speaker_list[i].position.z = self.speaker_list[i].position.z + delta_theta 
        speaker_pos_publisher.publish(self.speaker_list)
        robot_pos_publisher.publish(self.robot)
        speaker_marker_pub.publish(self.marker_array)
        robot_marker_pub.publish(self.robot_marker)
  
    def listener(self):
        rospy.Subscriber("clicked_point", PointStamped, self.update)
        speaker_pos_publisher.publish(self.speaker_list)
        robot_pos_publisher.publish(self.robot)
        robot_marker_pub.publish(self.robot_marker)
        speaker_marker_pub.publish(self.marker_array)

        rospy.spin()   

if __name__=="__main__":
    rospy.init_node('simulation_al')
    speaker_pos_publisher = rospy.Publisher('speaker_positions', SpeakerPositionList, queue_size=10)
    robot_pos_publisher = rospy.Publisher('robot_pos', RobotPosition, queue_size = 10)
    speaker_marker_pub = rospy.Publisher('speaker_visualization_array', MarkerArray, queue_size=10)
    robot_marker_pub = rospy.Publisher('robot_vis_array', Marker, queue_size = 10)
    rospy.sleep(2)
    
    rate = rospy.Rate(3) # 3 Hz
    new_room = localizer_simulation_driver(100,100)
    new_room.create_robot()
    new_room.create_speakers(3)
   
    while not rospy.is_shutdown():
      new_room.listener()
      #  rospy.Subscriber("clicked_point", PointStamped, new_room.update)
      #  speaker_pos_publisher.publish(new_room.speaker_list)
      #  robot_pos_publisher.publish(new_room.robot)
      #  rospy.loginfo(new_room.marker_array)
      #  rospy.loginfo(new_room.robot)
      #  speaker_marker_pub.publish(new_room.marker_array)
      #rate.sleep()        
