#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <logical_camera_plugin/logicalImage.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>
//#include <geometry_msgs/TransformStamped.h>



void acmlMessageRecieved(const sensor_msgs::Imu &msg) {
  ROS_INFO_STREAM("Recieved IMU Scan: ");  
  ROS_INFO_STREAM(msg);  
}

void logicalCameraReceived(const logical_camera_plugin::logicalImage &msg) {
    ROS_INFO_STREAM("Recieved Camera: ");
    /*
    tf::StampedTransform transform;
        try{
	    listener.lookupTransform("/base_link", "/world", ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
    ROS_INFO_STREAM("Transformation: " << std::endl << transform);
    */
  ROS_INFO_STREAM(msg);
  geometry_msgs::Pose imagePose;
  std::string name = msg.modelName;
  imagePose.position.x = msg.pose_pos_x;
  imagePose.orientation.x = msg.pose_rot_x;
  ROS_INFO_STREAM(imagePose);
}


int main(int argc, char **argv){
    ros::init(argc, argv, "camera");
    ros::NodeHandle nh;

    ros::Subscriber acmlScan = nh.subscribe("/acml_pose", 1000, &acmlMessageRecieved);

    ros::Subscriber camera = nh.subscribe("/objectsDetected", 1000, &logicalCameraReceived);

    tf::TransformListener listener;

    ros::Rate rate(10.0);

    while (ros::ok() ) {
  
        ros::spinOnce();
    }
    return 0;
}
