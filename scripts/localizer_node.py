#!/usr/bin/env python
"""
    acoustic_localizer.py

    Author: Julien Blanchet
    Feb. 8 2020

    A localization package that will use direction of arrival (DoA) measurements
    of incoming sounds to generate a localization estimate of a robot's position
"""


import rospy
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from acousticlocalization.msg import Sound2DDoAFrame, Sound2DDoA, SpeakerPositionList, SpeakerPosition
from acoustic_localizer import estimate_pose_2d_doa, EstimateParams as AcousticEstimatorParams

class AcousticLocalizerNode:

    def __init__(self):
        rospy.init_node("AcousticLocalizer")
        
        self.pub_pose = rospy.Publisher(
            name="pose", 
            data_class=PoseStamped, 
            queue_size=1
        )

        self.sub_doa = rospy.Subscriber(
            name="acoustic/doa", 
            data_class=Sound2DDoAFrame, 
            callback=self.on_doa,
            queue_size=1
        )

        self.sub_speaker_pos = rospy.Subscriber(
            name="acoustic/speaker_positions",
            data_class=SpeakerPositionList,
            callback=self.on_speaker_pos,
            queue_size=1
        )

        self.speaker_positions = {}

    def on_speaker_pos(self, msg):
        updated_speaker_positions = {}

        for speaker_position in msg.positions:
            updated_speaker_positions[speaker_position.speakerId] = speaker_position.position

        self.speaker_positions = updated_speaker_positions
        rospy.logdebug("Received new speaker position list. New list: {}".format(self.speaker_positions))

    def on_doa(self, msg):
        rospy.logdebug("Received DoA Frame: {0}".format(msg))

        msg = Sound2DDoAFrame()
        doas_with_positions = map(lambda doamsg: (doamsg.angle, self.speaker_positions.get(doamsg.sourceId)), msg.doas)
        doas_with_positions = filter(lambda doa_w_pos: doa_w_pos[1] is not None, doas_with_positions)

        if len(doas_with_positions) < 3:
            rospy.logerr("Cannot perform localization: cannot identify 3 DoAs with known speaker locations")
            return

        s1_doa, s1_pos = doas_with_positions[0]
        s2_doa, s2_pos = doas_with_positions[1]
        s3_doa, s3_pos = doas_with_positions[2]
        
        rospy.loginfo("Esimating state with following info:\n" + \
                      "s1 doa: {}    pos: {}\n".format(s1_doa, s1_pos) + \
                      "s2 doa: {}    pos: {}\n".format(s2_doa, s2_pos) + \
                      "s3 doa: {}    pos: {}\n".format(s3_doa, s3_pos))

        estimated_state = estimate_pose_2d_doa(
            s1_doa=s1_doa,
            s2_doa=s2_doa,
            s3_doa=s3_doa,
            s1x = s1_pos.x, s1y = s1_pos.y,
            s2x = s2_pos.x, s2y = s2_pos.y,
            s3x = s3_pos.x, s3y = s3_pos.y
        )
        
        if estimated_state is None:
            rospy.logerr("Could not compute an estimated state")
            return
        
        pose_estimate = PoseStamped()
        pose_estimate.pose.position.x = estimated_state.x
        pose_estimate.pose.position.y = estimated_state.y
        pose_estimate.pose.position.z = 0

        orientation = quaternion_from_euler(0, 0, estimated_state.theta)
        pose_estimate.pose.orientation.x = orientation.x
        pose_estimate.pose.orientation.y = orientation.y
        pose_estimate.pose.orientation.z = orientation.z
        pose_estimate.pose.orientation.w = orientation.w

        pose_estimate.header = Header()
        pose_estimate.header.stamp = msg.header.stamp
        pose_estimate.header.frame_id = "map"

        self.pub_pose.publish(pose_estimate)

    def spin(self):
        rospy.spin()

if __name__ == "__main__":
    localizer = AcousticLocalizerNode()
    localizer.spin()

