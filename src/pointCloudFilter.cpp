#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

//Used to convert to/from PCl to ROS pointcloud dt
#include <pcl_conversions/pcl_conversions.h>

//Used to set/send pointcloud msgs
#include <sensor_msgs/PointCloud2.h>

//Used to filter poitcloud
#include <pcl/filters/voxel_grid.h>


//Global declaration not advised but safe 
// in small projects
ros::Publisher pub;

void callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
    // Container for original & filtered data
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2 cloud_filtered;

    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *cloud);

    
    // Perform the actual filtering
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloudPtr);
   
    //Set filter target
    if(sor.getFilterFieldName() != "intensity"){
        sor.setFilterFieldName("intensity");
    }
    //filter intensities (min, max)
    // Max here set arbitrary
    sor.setFilterLimits(10.00 ,1000000.00);

    //Set leaf size for visualization 0.1 normal
    //0.01 used for finer visualization of pointcloud
    sor.setLeafSize(0.01, 0.01, 0.01);
    sor.filter(cloud_filtered);

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(cloud_filtered, output);


    //Set header time and frame so points can be visualized 
    output.header.stamp = ros::Time::now();
    output.header.frame_id = "pointcloud";
    
    
    pub.publish(output);
}
int main(int argc, char **argv){
    ros::init(argc, argv, "laser_FilterNode");

    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("input_Cloud", 10, callback);

    pub = nh.advertise<sensor_msgs::PointCloud2>("output_points", 10);

    ros::spin();

    
}
