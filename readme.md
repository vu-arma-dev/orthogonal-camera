# Orthogonal Cameras or "ortho_cams"

## Step 0 : Setup
* roscore

## Step 1
* open a new command window
* cd to ~catkin_ws_camera/devel_release and run "source setup.bash"
* cd to ~catkin_ws_camera/src/dvrk_vision/launch/ and run "roslaunch dvrk_vision just_cams.launch single_image_from_camera:=false width:=640"

camera_info_url_left:= misumi1_calib_data.yaml
camera_info_url_right:= misumi2_calib_data.yaml


## Step 2
* open a new command window
* cd to ~catkin_ws_camera/devel_release and run "source setup.bash"
* cd to  ~/catkin_ws_camera/src/ortho_cams/src/ortho_cams
* run "ipython ortho_cams_pyqt_v2.py"

## Color threshold: 
0 - Orange; 1 - BLue; 2 - Green;  3- Red
## Usage
* Stuff goes here