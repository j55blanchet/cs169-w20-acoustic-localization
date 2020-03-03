#!/usr/bin/env python
"""
    acoustic_localizer.py

    Author: Julien Blanchet
    Feb. 8 2020

    A localization package that will use direction of arrival (DoA) measurements
    of incoming sounds to generate a localization estimate of a robot's position
"""


import rospy
from geometry_msgs.msg import PointStamped
from acousticlocalization.msg import Sound2DDoAFrame
from acoustic_localizer import estimate_position2d

class CONSTANTS:
    pass

class AcousticLocalizerNode:

    def __init__(self):
        rospy.init_node("AcousticLocalizer")
        
        self.pub_pose = rospy.Publisher(
            name="pose", 
            data_class=PointStamped, 
            queue_size=1
        )

        self.sub_doa = rospy.Subscriber(
            name="acoustic/doa", 
            data_class=Sound2DDoAFrame, 
            callback=self.on_doa,
            callback_args=None,
            queue_size=1
        )

        self.propogation_rate = CONSTANTS.propogation_rate

    def on_doa(self, msg):
        rospy.logdebug("Received DoA Frame: {0}".format(msg))

        # TODO: probably want to save this somewhere. If we'll use a graph optimization
        #       technique, then we'd like to save a somewhat full history. If we use a
        #       kalman filter, we don't need to save it - just perform the udpate

        estimate = self.estimate_state()
        
        self.publish_estimation()
    
    def estimate_state(self, msg):
        """Estimate the position of the robot
        
        Arguments:
            msg {Sound2DDoAFrame} -- A list of estimated direction of arrivals
        
        Returns:
            PointStamped -- The estimated position of the robot
        """

        point = Point()
        


        pose = PoseWithCovarianceStamped()
        pose.pose.pose.position = point
        pose.header
        state = Vector3Stamped()
        return 

    def publish_estimation(self):
        pass

    def spin(self):
        while not rospy.is_shutdown():
            self.propogate()
            self.publish_estimation()
            self.propogation_rate.sleep()

if __name__ == "__main__":
    localizer = AcousticLocalizer()
    localizer.spin()

