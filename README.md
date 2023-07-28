# ergonomic_assessment
This repo provides vision-based real-time ergonomic feedback based on the [Rapid Upper Limb Assessment (RULA)](https://doi.org/10.1016/0003-6870(93)90080-S).
Based on body tracking data, we calculate all relevant joint angles and use them to compute RULA scores.

For more information, please refer to our article:

Eversberg, L., C. Sohst, and J. Lambrecht. "Assistenzsystem zur Verbesserung der Ergonomie / Assistance system to improve ergonomics – Preventing musculoskeletal disorders in manufacturing with artificial intelligence". wt Werkstattstechnik online 112.9 (2022). [http://dx.doi.org/10.37544/1436-4980-2022-09-68](http://dx.doi.org/10.37544/1436-4980-2022-09-68)

# Demo Video
[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/GOXe7FzxO5g/0.jpg)](https://www.youtube.com/watch?v=GOXe7FzxO5g)

# Installation
Clone this forked ROS melodic package into your catkin_ws/src folder.
The package uses the topic \/body\_tracking\_data from the [Microsoft Azure Kinect ROS driver](https://github.com/leoneversberg/Azure_Kinect_ROS_Driver). 
This cloned ROS driver version sets the body tracking marker color according to the joint confidence value.


# Launch ROS
Start ROS via ``roslaunch ergonomic_assessment rula.launch``


# Use Rosbags
To use recorded rosbags, run ``rosbag play './rosbags/body_tracking_data.bag' --loop``


