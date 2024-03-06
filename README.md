# ROS_PointCloudFiltering
filtering of a ROS pointcloud2 with testing publisher and rviz lanuch file

build steps:
Add folder to ros workspace  
in root of workspace:  
  catkin_make  
  chmod -x LidarFilter.launch  
  source devel/setup.bash  
  roslanuch <ros_ws_name> LidarFilter.launch  

Run Rviz for visulization:  
  ros rviz rviz   
  add a pointcloud2 node  
  set the points to /output_points  
