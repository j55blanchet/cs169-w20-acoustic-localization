# cs169-w20-acoustic-localization

This package provides algorithms and simulations for passive acoustic localization. In particular, this package assumes a robot with three non-colinear microphones and three speakers of known location (`SpeakerPositionList`). Using this information, the direction of arrival (DoA) of audio signals can be deduced (`Sound2DDoaFrame`) and the robot pose can be inferred (`PoseStamped`).

## Setup
1. Put this package inside of an existing catkin workspace (within the `src` folder). The package name is `acousticlocalization`, so it's recommended you use that as the name of the folder.
1. Run `catkin_make install` at the root of the catkin workspace (in order to build custom messages).
1. Run `catkin_make` (just to be safe)

## Running

#### DoA Validation
* To run a DoA visualization, run `rosrun acousticlocalization simEnv.py`
* To modify values of the speaker locations or microphone locations, simply modify the parameters of the file `simEnv` in the `scripts` folder. 

#### Localizer 
* To run the unit tests for the acoustic localizer: `roslaunch acousticlocalization acoustic_localizer_unit_tests.launch`
* To run a localization visualization, in which speakers are placed randomly and the robot has to localize itself on many different points on the screen: `roslaunch acousticlocalization localizer_visualization.launch`
* To view the estimate, make sure you have a roscore running. Afterward, type `rosrun rviz rviz`in a new terminal. The appropriate markers will visualize. Configure rviz to visualize `speaker_visualization_array`, `pose`, and `robot_vis`. The RED markers show the speaker locations, the WHITE marker shows the origin(0,0), and the BLUE marker shows the robot position. Our estimate will be a red arrow with the origin at the estimated location and the tip pointing toward the speakers.

## Development Notes

**Custom Messages**: to add additional custom messages:
1. Create the appropiate `.msg` file in the `msg/` folder,
1. Edit `CMakeLists.txt` and add the file to the `add_message_files(...)` code block (in a new line)
1. Edit `CMakeLists.txt` and add any new dependencies to the `generate_messages(...)` code block (in a new line)
1. Run `catkin_make install` in the root of your workspace
