#!/usr/bin/env python
"""
    acoustic_localizer.py

    Author: Julien Blanchet
    Feb. 8 2020

    A localization package that will use direction of arrival (DoA) measurements
    of incoming sounds to generate a localization estimate of a robot's position

    The main exports of this package are:
        estimate_pose_2d_doa (function)
        EstimateParams (supporting data type)
"""

from __future__ import print_function, division
import collections
import math
import time
import random

###
### Data Types - we define our own types here to avoid depending on any other library
###   
Pose = collections.namedtuple('Pose', 'x y theta')
Position = collections.namedtuple('Position', 'x y')
SpeakerInfo = collections.namedtuple('SpeakerInfo', 'pos doa')
EstimateParams = collections.namedtuple('EstimateParams', 'angle_acceptance angle_divisions distance_increment')

_default_params = EstimateParams(
    angle_acceptance = math.pi / 360.0,
    angle_divisions = 360,
    distance_increment = 0.001
)

###
### Helper Functions
###

def _normalize_angle(angle):
    """Normalize an angle to be between -pi and pi
    
    Arguments:
        angle {float} -- Raw angle measurement, in radians
    
    Returns:
        float -- Normalized angle measurement, in radians
    """
    # From https://stackoverflow.com/questions/2320986/easy-way-to-keeping-angles-between-179-and-180-degrees
    return angle - (math.ceil((angle + math.pi)/(2*math.pi))-1)*2*math.pi

def _dist_line_to_point(line_x, line_y, line_theta, px, py):
    """Find the distance between the target point and the nearest point on a line
    
    Arguments:
        line_x {float} -- X coordinate of a location on the line
        line_y {float} -- Y coordinate of a location on the line (cooresponding to the same location as the x)
        line_theta {float} -- Angle that the line makes from the x, y coordinate
        px {float} -- X coordinate of the point to check the distance to
        py {float} -- Y coordinate of the point to check the distance to 
    
    Returns:
        float -- The distance from the point to the line
    """
    
    # First step: solve for y = mx + b
    m = math.tan(line_theta)
    b = line_y - (m * line_x)
    
    # Second step: solve for ax + by + c = 0
    c = b
    b = -1
    a = m
    
    # Third step: plug into equation for distance to point from a line
    # Referred to: https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#Line_defined_by_an_equation
    return abs((a * px + b * py + c)) / (math.sqrt(a ** 2 + b ** 2)) 

def _calculate_error(pose, ref_doa, ref_pos, angle_acceptance):
    """Given a candidate robot pose that has heard a signal from a reference speaker at a specified direction of arrival,
    determine how close the speaker is to the ray where it should be.
    
    Arguments:
        pose {Pose} -- Candidate robot pose to calculate the error of (relative to a reference speaker)
        ref_doa {float} -- Direction of arrival, in radians, of the signal heard from the reference speaker (relative to the robot's heading)
        ref_pos {Position} -- Position of the reference speaker, in world coordinates
        angle_acceptance {float} -- Parameter which determines how aggresively to discount the candidate pose when the DoA measurement is out of
                                    line with the actual angle between the candidate pose and the speaker. Should always be less than half pi
    
    Returns:
        [float] -- Distance between the speaker's location and the ray projected by the candidate pose's DoA measurement,
                   or infinity of the angles don't match up
    """
    est_theta_to_ref = _normalize_angle(pose.theta + ref_doa)
    actual_theta_to_ref = math.atan2(ref_pos.y - pose.y, ref_pos.x - pose.x)
    angle_error = _normalize_angle(est_theta_to_ref - actual_theta_to_ref)

    if abs(angle_error) > angle_acceptance:
        return float("inf")
    else:
        return _dist_line_to_point(pose.x, pose.y, est_theta_to_ref, ref_pos.x, ref_pos.y)

def _find_candidate_poses(source_dist, primary_speaker_position, primary_speaker_doa, secondary_speaker_position, secondary_speaker_doa, params):
    """Finds candidate robot poses that are a specified distance from the source speaker given
    the specified direction-of-arrival (DoA) measurements.
    
    Arguments:
        source_dist {float}                    -- How far away from the primary speaker we should locate the candidate poses
        primary_speaker_position {Position}    -- Where the primary speaker is, in world coordinates
        primary_speaker_doa {float}            -- The angle in which the signal from the primary speaker was detected, relative to the robot's heading
        secondary_speaker_position {Position}  -- Where a secondary speaker is, in world coordinates
        secondary_speaker_doa {float}          -- The angle in which the signal from the secondary speaker was detected, relative to the robot's heading
        params {EstimateParams}                -- Parameters to use while conducting the search
    
    Returns:
        Pose[] -- A list of candidate poses that are the specified distance away and satisfy the direction of arrival constraints. 
                  This list is expected to contain between zero and two items
    """

    candidate_poses = []
    last_error = float("inf")
    candidate_pose = None

    # We go over by one so that the last angle division has a chance to get reported as a candidate pose.
    #    > This means that the first angle division (i=0) will get checked twice. However, it won't get reported
    #      as the candidate pose twice
    for i in range(params.angle_divisions + 1):
        theta = (i / params.angle_divisions) * 2 * math.pi

        # Compute the candidate pose
        px = math.cos(theta) * source_dist + primary_speaker_position.x
        py = math.sin(theta) * source_dist + primary_speaker_position.y
        ptheta = _normalize_angle(theta + math.pi - primary_speaker_doa)
        test_pose = Pose(px, py, ptheta)

        # Figure out how good of a pose this is. Note that there can be two valid poses that are the specified distance away - 
        #   therefore, we look for local minima in errors as we iterate in the circle.
        error = _calculate_error(test_pose, secondary_speaker_doa, secondary_speaker_position, params.angle_acceptance)    
        if error < last_error:
            candidate_pose = test_pose
        elif error > last_error and candidate_pose is not None:
            candidate_poses.append(candidate_pose)
            candidate_pose = None

        last_error = error

    return candidate_poses


###
### Core Algorithm
###

def estimate_pose_2d_doa(s1_doa, s2_doa, s3_doa, s1x, s1y, s2x, s2y, s3x, s3y, params=_default_params):
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
        SpeakerInfo(Position(s1x, s1y), _normalize_angle(s1_doa)),
        SpeakerInfo(Position(s2x, s2y), _normalize_angle(s2_doa)),
        SpeakerInfo(Position(s3x, s3y), _normalize_angle(s3_doa))
    ]

    # For now, it's arbitrary which speaker we consider the primary vs secondary vs tertiary.
    random.shuffle(speakers)
    primary_speaker = speakers[0]
    secondary_speaker = speakers[1]
    tertiary_speaker = speakers[2]
    
    # The distance away from the primary speaker on which to look for candidate poses. 
    # We start close and keep increasing until we find no more candidate poses
    d = params.distance_increment

    best_candidate = None
    best_candidate_error = float("inf")
    candidate_count = 0
    
    while True:

        # We reference the primary and secondary speakers to get a list of candidate poses
        candidate_poses = _find_candidate_poses(d, primary_speaker[0], primary_speaker[1], secondary_speaker[0], secondary_speaker[1], params)

        if len(candidate_poses) < 1:
            # Once we reach a certain distance we won't find any more candidate poses,
            # and it's time to break off the search
            break

        for pose in candidate_poses:
            
            # We cross reference with the tertiary speaker to determine which candidate pose is the best
            error = _calculate_error(pose, tertiary_speaker.doa, tertiary_speaker.pos, params.angle_acceptance)
            if error < best_candidate_error:
                best_candidate_error = error
                best_candidate = pose

            candidate_count += 1

        d += params.distance_increment

    print("   tested {} candidates".format(candidate_count))
    print("   found a solution" if best_candidate is not None else "    didn't find a solution")
    return best_candidate

###
### Testing - The following code is used to validate the algorithm
###

def _time(f):
    """Utility function to measure how long (in seconds) a function takes to execute."""
    begin = time.time()
    res = f()
    end = time.time()
    return (end - begin, res)

_testcount = 1
    
def _test_find_poses(source_dist, sp_source_pos, sp_source_doa, sp_other_pos, sp_other_doa, expected_poses, params=_default_params):
    global _testcount
    found_poses = _find_candidate_poses(source_dist, sp_source_pos, sp_source_doa, sp_other_pos, sp_other_doa, params)

    print("")
    print("Test #{}".format(_testcount))
    _testcount += 1
    print("\tFound {} poses".format(len(found_poses)))

    for pose in found_poses:
        print("\t\t{}".format(pose))

    print("\tExpected poses")
    for pose in expected_poses:
        print("\t\t{}".format(pose))


def _test_estimate_position2d(s1_doa, s2_doa, s3_doa, s1x, s1y, s2x, s2y, s3x, s3y, expect_x, expect_y, expect_theta, params=_default_params):
    global _testcount
    print()
    print("Test: #{}".format(_testcount))
    _testcount += 1
    # print("\tDoA - ", "s1:", s1_doa, "s2:", s2_doa, "s3:", s3_doa, sep=" ")
    # print("\tPositions - ", "s1: {}, {}   s2: {}, {}    s3: {}, {}".format(s1x, s1y, s2x, s2y, s3x, s3y))
    time, estimated_pose = _time(lambda: estimate_pose_2d_doa(s1_doa, s2_doa, s3_doa, s1x, s1y, s2x, s2y, s3x, s3y))
    print("\tExpected:\n\t\t{}".format(Pose(expect_x, expect_y, expect_theta)))
    print("\tEstimated:\n\t\t{}".format(estimated_pose))
    print("\tTook {:.3f}s".format(time))
    

if __name__ == "__main__":

    print("=============================")
    print("===   Starting Test Run   ===")
    print("=============================")
    print("")
    print("____________________")
    print("Testing pose finding")
    print("")
    
    _test_find_poses(
        source_dist=0.5,
        sp_source_pos=Position(2, 1),
        sp_source_doa=math.pi * 0.5,
        sp_other_pos=Position(1, 1),
        sp_other_doa=math.atan2(0.5, 1),
        expected_poses=[Pose(2, 1.5, math.pi)]
    )

    _test_find_poses(
        source_dist=math.sqrt(3),
        sp_source_pos=Position(1, 2),
        sp_source_doa= -math.pi,
        sp_other_pos=Position(1, 1),
        sp_other_doa= -math.pi * 5 / 6,
        expected_poses=[
            Pose(1 + math.sqrt(3), 2, 0),
            Pose(x=1.8660254037844388, y=0.5000000000000002, theta=-1.0471975511965965)
        ]
    )
    
    print("\n")
    print("__________________________________")
    print("Testing DoA Localization Algorithm")
    
    _test_estimate_position2d(
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

    _test_estimate_position2d(
        s1_doa = math.radians(-77.53) + math.radians(-45) + math.radians(-18.43),
        s2_doa = math.radians(-77.53) + math.radians(-45),
        s3_doa = math.radians(-77.53),
        s1x = 2.0, s1y = 4.0,
        s2x = 1.0, s2y = 3.0,
        s3x = 3.0, s3y = 2.0,
        expect_x = 4.0,
        expect_y = 4.0,
        expect_theta = math.radians(-39.04)
    )

    _test_estimate_position2d(
        s1_doa = math.radians(45) + math.radians(18.43),
        s2_doa = math.radians(45) + math.radians(18.43) + math.radians(18.43),
        s3_doa = math.radians(45),
        s1x = 0.0, s1y = 2.0,
        s2x = 0.0, s2y = 1.0,
        s3x = 0.0, s3y = 3.0,
        expect_x = 3.0,
        expect_y = 2.0,
        expect_theta = math.radians(180 - 45 - 18.43)
    )

    _test_estimate_position2d(
        s1_doa = math.radians(78.69) + math.radians(11.31),
        s2_doa = math.radians(78.69) + math.radians(11.31) + math.radians(11.31),
        s3_doa = math.radians(78.69),
        s1x = 1.0, s1y = 3.0,
        s2x = 0.5, s2y = 3.0,
        s3x = 1.5, s3y = 3.0,
        expect_x = 1.0,
        expect_y = 0.5,
        expect_theta = math.radians(0)
    )