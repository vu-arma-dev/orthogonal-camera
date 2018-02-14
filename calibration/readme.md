
# Calibrating the MISUMI cameras using ROS
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
* Open a new terminal window
* Run "rosdep install camera_calibration"
* Run "rostopic list"
* If the operation is successful, the topics provided by the camera will appear as: /camera/camera_info & /camera/image_raw. Note that because we are calibrating two cameras at once, the output reads "/stereo/left/" and "stereo/right" in lieu of "camera"

## Calibrate the first camera
* Run "rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.03 image:=/stereo/left/image_raw camera:=/stereo/left" (Mofidy the size of the baord and the square according to the board you are using)
* Move the checker board around (x, y, z, pitch, yaw, roll) until you obtain green lines for x, y, z, size, and skew. A "Calibrate" button will appear once the data collected by the camera is satisfactory 
* Press "Calibrate" and wait for a minute while it calibrates 
* Once it is calibrated, you will see "Calibrate", "save", and "commit" buttons. Press save and take note of location shown in the terminal. It will most likely save the file in a tmp location. 
* Press commit to store the data to the camera. At this point, the GUI exists. 
* Locate your saved calibration file (it will include pictures, a yaml file and a text file). The yaml file is important, save it. 

## Calibrate the second camera
* Work in the same terminal as camera 1
* Repeat the same steps as for the first camera but this time, run "rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.03 image:=/stereo/right/image_raw camera:=/stereo/right" (Mofidy the size of the baord and the square according to the board you are using)

## Rectifying an image 
* This section is to be updated using the [image_proc package](http://wiki.ros.org/image_proc) info