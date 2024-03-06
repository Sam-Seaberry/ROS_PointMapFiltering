#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>

/*
Used to test pointClouldFilter node. This node publishes a pointcloud
from a .pcd file.

*/

int main(int argc, char** argv){
    ros::init(argc, argv, "cloud_PublsiherNode");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("input_Cloud", 1);
    
    pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
    pcl::PCDReader reader;
    
    // Replace the path below with the path where you saved your file
    reader.read ("src/quantillion/raw/table_scene_lms400.pcd", *cloud);

    //re-send msg every 10s 
    // Here not nessisary as data doesnt change
    // but good to add to mimic real world
    ros::Rate loop_rate(10);

    while(ros::ok()){
        pub.publish(cloud);
        ros::spinOnce();

        loop_rate.sleep();
    }


    return 0;
}