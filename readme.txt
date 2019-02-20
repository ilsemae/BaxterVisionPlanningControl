
First:

	cd ~/ros_ws
	catkin_make
	. ~/ros_ws/devel/setup.bash
	. baxter.sh

To enable the robot:

	rosrun baxter_tools enable_robot.py -e


-------------------------------------------------------------------------

To launch the master program:

	roslaunch group7_final final.launch


-------------------------------------------------------------------------

To view current image on a camera:

    rosservice call /cameras/open '{name: right_hand_camera, settings: {width: 1024, height: 600 }}'
    rosrun image_view image_view image:=/cameras/right_hand_camera/image

    rosservice call /cameras/open '{name: left_hand_camera, settings: {width: 1024, height: 600 }}'
    rosrun image_view image_view image:=/cameras/left_hand_camera/image

    rosrun image_view image_view image:=/camera/rgb/image_color


-------------------------------------------------------------------------

To start kinect:

    ssh -X en-cs-lgg03-10
    htop
    F3
    "freenect"
    use arrows to select the freenect node found
    F9 to kill all freenect nodes
    F10 to quit
    cd ~/ros_ws
    . baxter.sh
    roslaunch freenect_launch freenect.launch &

-------------------------------------------------------------------------

To change phases manually while we test our code:

    rostopic pub /game_server/game_state game_server/GameState 1 60 [0,0] [0,0]  -1



