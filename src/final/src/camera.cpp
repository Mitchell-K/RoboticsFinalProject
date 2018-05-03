#include "ros/ros.h"
#include <logical_camera_plugin/logicalImage.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <vector>


// Struct for found treasures
struct Treasure{
    std::string name;
    float x, y, z, w;
};

// Global for pose
geometry_msgs::Pose amclPose;

// Vector of Treasures
std::vector<Treasure> treasures;


void amclMessageRecieved(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msgAMCL) {
    //ROS_INFO_STREAM("Recieved AMCL msg: ");  
    amclPose = msgAMCL->pose.pose;   
    //ROS_INFO_STREAM(amclPose);  
}

void logicalCameraReceived(const logical_camera_plugin::logicalImage &msg) {
    bool alreadyFound = false;
    for (std::vector<Treasure>::iterator it = treasures.begin(); it != treasures.end(); ++it) {
    	if (it->name == msg.modelName) {
	    alreadyFound = true;
	    break;
	}
    }
    if (not alreadyFound) {
	// Save the location of image relative to absolute
        geometry_msgs::Pose imagePose;
        imagePose.position.x = msg.pose_pos_x + amclPose.position.x;
        imagePose.position.y = msg.pose_pos_y + amclPose.position.y;
        imagePose.orientation.z = msg.pose_rot_z + amclPose.orientation.z;
        imagePose.orientation.w = msg.pose_rot_w + amclPose.orientation.w;

	// Save treasure struct
	Treasure tempTreasure;
	tempTreasure.name = msg.modelName;
	tempTreasure.x = imagePose.position.x;
	tempTreasure.y = imagePose.position.y;
	tempTreasure.z = imagePose.orientation.z;
	tempTreasure.w = imagePose.orientation.w;
	treasures.push_back(tempTreasure);

	ROS_INFO_STREAM("Found treasure(" << treasures.size() << "):"<< msg.modelName << " at location: " << std::endl << imagePose);
    }
}


int main(int argc, char **argv){
    ros::init(argc, argv, "camera");
    ros::NodeHandle nh;

    ros::Subscriber amclScan = nh.subscribe("amcl_pose", 1000, &amclMessageRecieved);

    ros::Subscriber camera = nh.subscribe("/objectsDetected", 1000, &logicalCameraReceived);


    while (ros::ok() ) {  
        ros::spinOnce();
    }
    return 0;
}
