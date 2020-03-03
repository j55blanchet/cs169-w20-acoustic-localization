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

from acousticlocalization.msg import SpeakerPosition, SpeakerPositionList, Sound2DDoA, Sound2DDoAFrame

