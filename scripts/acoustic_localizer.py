#!/usr/bin/env python
"""
    acoustic_localizer.py

    Author: Julien Blanchet
    Feb. 8 2020

    A localization package that will use direction of arrival (DoA) measurements
    of incoming sounds to generate a localization estimate of a robot's position
"""

from __future__ import print_function
import collections
import math

class _CONSTANTS:
    MIN_DISTANCE = 0.1
    DISTANCE_INCREMENT = 0.05

Position = collections.namedtuple('Position', 'x y')
SpeakerInfo = collections.namedtuple('SpeakerInfo', 'pos doa')

def _find_poses(source_dist, sp_source_pos, sp_source_doa, sp_other_pos, sp_other_doa):

    candidate_poses = []

    return candidate_poses

def estimate_position2d(s1_doa, s2_doa, s3_doa, s1x, s1y, s2x, s2y, s3x, s3y):
    """Estimates the position of a receiver using direction of arrival
    (DoA) measurements of incoming sound, referencing the speaker's known location in
    order to make this determination. 
    
    Arguments:
        s1_doa {float} -- Direction of arrival of the sound coming from speaker 1
        s2_doa {float} -- Direction of arrival of the sound coming from speaker 2
        s3_doa {float} -- Direction of arrival of the sound coming from speaker 3
        s1_pos {(float, float)} -- X & Y position of speaker 1
        s2_pos {(float, float)} -- X & Y position of speaker 2
        s3_pos {(float, float)} -- X & Y position of speaker 3
    
    Returns:
        (float, float, float) -- estimated state of the receiver: x, y, and yaw
    """

    speakers = [
        SpeakerInfo(Position(s1x, s1y), s1_doa),
        SpeakerInfo(Position(s2x, s2y), s2_doa),
        SpeakerInfo(Position(s3x, s3y), s3_doa)
    ]

    source_speaker = speakers[0]
    dest_speaker = speakers[1]
    ref_speaker = speakers[2]
    

    d = _CONSTANTS.MIN_DISTANCE

    best_candidate = None
    best_candidate_error = None

    while True:
        candidate_poses = _find_poses(d, source_speaker[0], source_speaker[1], dest_speaker[0], dest_speaker[1])
        
        if len(candidate_poses) < 1:
            break

        for pose in candidate_poses:


        d += _CONSTANTS.DISTANCE_INCREMENT
        

    return (0.0, 0.0)






def _run_test(test_num, s1_doa, s2_doa, s3_doa, s1x, s1y, s2x, s2y, s3x, s3y, expect_x, expect_y, expect_theta):
    print("Test: ", test_num)
    print("Direction of Arrivals - ", "s1:", s1_doa, "s2:", s2_doa, "s3:", s3_doa, sep=" ")
    print("Positions - ", "s1: {}, {}   s2: {}, {}    s3: {}, {}".format(s1x, s1y, s2x, s2y, s3x, s3y))
    print("Expected: x, y, theta: {},  {},  {}".format(expect_x, expect_y, expect_theta))

    
    

if __name__ == "__main__":
    print("Performing test on acoustic localization algorithm")


    _run_test(
        test_num=1,
        s1_doa = math.pi * 0.5,
        s2_doa = math.pi * 0.75,
        s3_doa = math.pi * 0.25,
        s1x = 1.0, s1y = 1.0,
        s2x = 2.0, s2y = 1.0,
        s3x = 1.0, s3y = 2.0,
        expect_x = 2.0,
        expect_y = 2.0,
        expect_theta = math.pi * 0.75
    )