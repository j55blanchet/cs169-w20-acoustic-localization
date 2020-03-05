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
            > randomize orientation each time 
        > Listen's to rviz's "clicked point" topic - when a new point is published, set's robot's position there and sends
        DoAFrame with updated DoA measurements

  

"""
import rospy
import random 
import math
import time
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped, Pose2D
from acousticlocalization.msg import SpeakerPosition, SpeakerPositionList, Sound2DDoA, Sound2DDoAFrame

def _normalize_angle(angle):
    # From https://stackoverflow.com/questions/2320986/easy-way-to-keeping-angles-between-179-and-180-degrees
    return angle - (math.ceil((angle + math.pi)/(2*math.pi))-1)*2*math.pi


class _CONSTANTS:
    START_SLEEP = 1
    SIZE = 8
    SPEAKER_COUNT = 3

class AcousticLocalizerSimulationNode:

    def __init__(self):

        self.pub_speaker_positions = rospy.Publisher("acoustic/speaker_positions", SpeakerPositionList, queue_size=1)
        self.pub_doa_frame = rospy.Publisher("acoustic/doas", Sound2DDoAFrame, queue_size=1)
        rospy.loginfo("Sleeping for {} seconds before starting localization simulation".format(_CONSTANTS.START_SLEEP))
        time.sleep(_CONSTANTS.START_SLEEP)

    def rand_val(self, min_val=-_CONSTANTS.SIZE / 2, max_val=_CONSTANTS.SIZE / 2):
        return min_val + (random.random() * (max_val - min_val))

    def new_environment(self):

        rospy.loginfo("Creating new environment")

        speakers = []
        for i in range(_CONSTANTS.SPEAKER_COUNT):
            speaker = SpeakerPosition()
            speaker.speakerId = i
            speaker.position.x = self.rand_val()
            speaker.position.y = self.rand_val()
            speakers.append(speaker)
            rospy.loginfo("    Speaker at {:.4f}, {:.4f}".format(speaker.position.x, speaker.position.y))

        speakers_msg = SpeakerPositionList()
        speakers_msg.positions = speakers
        self.pub_speaker_positions.publish(speakers_msg)
        # TODO: create markers

        robot_pose = Pose2D()
        robot_pose.x = self.rand_val()
        robot_pose.y = self.rand_val()
        robot_pose.theta = self.rand_val(- math.pi, math.pi)
        rospy.loginfo("    Robot at {}, {}, theta: {}".format(robot_pose.x, robot_pose.y, robot_pose.theta))
        # TODO: create marker

        doa_frame_msg = Sound2DDoAFrame()
        
        for spk in speakers:
            doa_msg = Sound2DDoA()
            doa_msg.sourceId = spk.speakerId
            angle_to_speaker = math.atan2(spk.position.y - robot_pose.y, spk.position.x - robot_pose.x)
            doa_msg.angle = angle_to_speaker - robot_pose.theta
            doa_frame_msg.doas.append(doa_msg)

        self.pub_doa_frame.publish(doa_frame_msg)
    
    def test_environment_1(self):
        rospy.loginfo("Running test environment 1")

        spk_1 = SpeakerPosition()
        spk_1.position.x = 1.0
        spk_1.position.y = 1.0
        spk_1.speakerId = 1

        spk_2 = SpeakerPosition()
        spk_2.position.x = 2.0
        spk_2.position.y = 1.0
        spk_2.speakerId = 2

        spk_3 = SpeakerPosition()
        spk_3.position.x = 1.0
        spk_3.position.y = 2.0
        spk_3.speakerId = 3

        speakers = [spk_1, spk_2, spk_3]

        speakers_msg = SpeakerPositionList()
        speakers_msg.positions = speakers
        self.pub_speaker_positions.publish(speakers_msg)
        # TODO: create markers

        robot_pose = Pose2D()
        robot_pose.x = 2.0
        robot_pose.y = 2.0
        robot_pose.theta = math.pi * 0.75
        rospy.loginfo("    Robot at {}, {}, theta: {}".format(robot_pose.x, robot_pose.y, robot_pose.theta))
        # TODO: create marker

        doa_frame_msg = Sound2DDoAFrame()
        
        for spk in speakers:
            doa_msg = Sound2DDoA()
            doa_msg.sourceId = spk.speakerId
            angle_to_speaker = math.atan2(spk.position.y - robot_pose.y, spk.position.x - robot_pose.x)
            rospy.loginfo("angle to speaker {}: {}".format(spk.speakerId, angle_to_speaker))
            doa_msg.angle = _normalize_angle(angle_to_speaker - robot_pose.theta)
            doa_frame_msg.doas.append(doa_msg)

        self.pub_doa_frame.publish(doa_frame_msg)

    def spin(self):
        self.test_environment_1()
        time.sleep(10)
        while not rospy.is_shutdown():
            self.new_environment()
            time.sleep(10)
        

if __name__ == "__main__":
    rospy.init_node("AcousticLocalizerSimulationNode")
    node = AcousticLocalizerSimulationNode()
    node.spin()