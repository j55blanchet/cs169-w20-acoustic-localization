#!/usr/bin/env python
"""
    acoustic_localizer.py

    Author: Julien Blanchet
    Feb. 8 2020

    A localization package that will use direction of arrival (DoA) measurements
    of incoming sounds to generate a localization estimate of a robot's position
"""

def estimate_position2d(doa_frame, speaker_positions):
    """Estimates the position of a receiver using direction of arrival
    (DoA) measurements of incoming sound (references the speaker's known location in
    order to make this determination). 

    This uses an error minimization approach, in which the estimated triangulation given least squared error approach
    
    Arguments:
        doa_frame {[type]} -- [description]
        speaker_positions {[type]} -- [description]
    
    Returns:
        [type] -- [description]
    """

    return (0.0, 0.0)

