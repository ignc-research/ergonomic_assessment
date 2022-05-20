# ergonomic_assessment
This repo provides vision-based real-time ergonomic feedback based on the [Rapid Upper Limb Assessment (RULA)](https://doi.org/10.1016/0003-6870(93)90080-S).
Based on body tracking data, we calculate all relevant joint angles and use them to compute RULA scores.

# Demo Video
[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/GOXe7FzxO5g/0.jpg)](https://www.youtube.com/watch?v=GOXe7FzxO5g)

# Installation
Clone this ROS package into your catkin_ws/src folder.
The package uses the topic \/body\_tracking\_data from the [Microsoft Azure Kinect ROS driver](https://github.com/microsoft/Azure_Kinect_ROS_Driver/blob/melodic/docs/usage.md#topics). 


# Launch ROS
Start ROS via ``roslaunch ergonomic_assessment rula.launch``


# Use Rosbags
To use recorded rosbags, run ``rosbag play './rosbags/body_tracking_data.bag' --loop``


