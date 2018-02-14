
# Calibrating the MISUMI cameras using ROS#
[Source](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration)

## Experimental setup
You will need:
* Two cameras capable of sending feeds over ROS 
* A checker board of known dimensions. I used an 8x6 checker board with 30mm x 30mm squares. (Note that calibration uses the interior vertex points of the checkerboard, so an "8x6" board is nine squares wide and seven high )
* An uncluttered area 


## Publish images over ROS
* Plug in the cameras (one in the front and another in the back)
* In a terminal window, run "roscore"
* open a second terminal window
* cd to ~catkin_ws_camera/devel_release and run "source setup.bash"
* cd to ~catkin_ws_camera/src/dvrk_vision/launch/ and run "roslaunch just_cams.launch" 


## Calibrate 
* Open a new terminal terminal window
* Run "rosdep install camera_calibration"
* Run "rostopic list"
* If the operation is successful, the topics provided by the camera will appear as: /camera/camera_info & /camera/image_raw. Note that because we are calibrating two cameras at once, the output reads "/stereo/left/" and "stereo/right" in lieu of "camera"

## Calibrate the first camera
* Run "rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.03 image:=/stereo/left/image_raw camera:=/stereo/left" (Mofidy the size of the baord and the square according to the board you are using)



## Calibrate the second camera
* Repeat the previous steps
* This time, run "rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.03 image:=/stereo/right/image_raw camera:=/stereo/right" (Mofidy the size of the baord and the square according to the board you are using)
