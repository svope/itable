itable_calib is ROS package for Kinect v2 - projector calibration

Dependencies:
	ROS Jade
	OpenCV 2.4.x
	SDL v2
	SDL image v2
	Expected usage with cooperation with https://github.com/code-iai/iai_kinect2 by Thiemo Wiedemeyer <wiedemeyer@cs.uni-bremen.de>, Institute for Artificial Intelligence, University of Bremen
	
Controls:
	Space 	- find pairs of 3D and corresponding 2D points
	ESC   	- end of calibration
	Arrows 	- positioning of chessboard in SDL screen
	+		- scale up chessboard in SDL screen
	-		- scale down chessboard in SDL screen
	
How to use:
	edit launch file
	roslaunch itable_calib itable_calib.launch
	capture some points...
	pressing ESC will calculate calibration data and save it to itable_calib/calibration_data.yaml 	

