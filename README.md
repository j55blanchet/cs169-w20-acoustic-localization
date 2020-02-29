# cs169-w20-acoustic-localization


## Setup
1. Put this package inside of an existing catkin workspace (within the `src` folder). The package name is `acousticlocalization`, so it's recommended you use that as the name of the folder.
1. Run `catkin_make install` at the root of the catkin workspace (in order to build custom messages).
1. Run `catkin_make` (just to be safe)

## Running
**TBD**: we'll probably have different launch files for the separate parts:
* A launch file that simulates DoA component
* A launch file that simulates localizer component
* A launch file that demonstrates / simulates a complete package

## Development Notes

**Custom Messages**: to add additional custom messages:
1. Create the appropiate `.msg` file in the `msg/` folder,
1. Edit `CMakeLists.txt` and add the file to the `add_message_files(...)` code block (in a new line)
1. Run `catkin_make install` in the root of your workspace