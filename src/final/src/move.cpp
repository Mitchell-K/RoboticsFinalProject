#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Pose2D.h>
#include <angles/angles.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/OccupancyGrid.h>
#include <vector>


//Globals
bool newGoal, sentGoal;
bool waitingForMap;
nav_msgs::OccupancyGrid map;
float curX = 0, curY = 0;

// Create struct to hold simple map coordinates
struct Cord{
    float x;
    float y;
    int index;
};
std::vector<Cord> prevCords;

// Checks if coordinate is free in cost map
bool isFree(Cord tempCord){
    int xGrid = map.info.width/2 + tempCord.x/map.info.resolution;
    int yGrid = map.info.height/2 + tempCord.y/map.info.resolution;
    int i = xGrid + map.info.width*yGrid;
    //ROS_INFO_STREAM("Checking (" << tempCord.x << ", " << tempCord.y << ") on costmap" << i << '\t'<< (int)map.data[i]  );
    if ( (int)map.data[i] >= 90 | (int)map.data[i] == -1 ) {
	ROS_INFO_STREAM("Bad coordinate");
	return false;
    }
    return true;
}
// Checks if we've been near this coordinate before
bool beenNearCord(const Cord tempCord){
    if (sqrt( pow(0-tempCord.x,2) - pow(0-tempCord.y,2) ) < 3.0){
	    ROS_INFO_STREAM("Too close to center");
	    return true;
        }
    std::vector<Cord>::iterator it;
    for(it = prevCords.begin(); it != prevCords.end(); it++)    {
	if (sqrt( float(pow(it->x-tempCord.x,2)) - float(pow(it->y-tempCord.y,2)) ) < 2.0){
	    ROS_INFO_STREAM("Been near, trying new cordinate.");
	    return true;
        }
    }
    return false;
}

void mapLoad(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    map = *msg;
    waitingForMap = false;	
    //ROS_INFO_STREAM("Map Loaded" << map.info);
}

// Called when new goal is activated
void serviceActivated() {
    ROS_INFO_STREAM("Service received goal");
}

// Called when goal is finished
void serviceDone(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result) {
    ROS_INFO_STREAM("Service completed");
    ROS_INFO_STREAM("Final state " << state.toString().c_str());
    if (state.toString().c_str() == "ABORTED")prevCords.pop_back();
    newGoal = true; // enable receiving a new goal
    sentGoal = false; // enable to recieve new goal
    ROS_INFO_STREAM("Waiting for new taget pose...");
}

// Called while attempting goal.
void serviceFeedback(const move_base_msgs::MoveBaseFeedbackConstPtr& fb) {
    //ROS_INFO_STREAM("Service still running");
    //ROS_INFO_STREAM("Current pose (x,y) " << fb->base_position.pose.position.x << "," << fb->base_position.pose.position.y);
    curX = fb->base_position.pose.position.x;
    curY = fb->base_position.pose.position.y;
}

int main(int argc,char **argv) {

    ros::init(argc,argv,"move");
    ros::NodeHandle nh;

    ros::Subscriber mapSub = nh.subscribe("map", 1000, mapLoad);
    waitingForMap = true;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",true);
    ROS_INFO_STREAM("Waiting for server to be available...");

    while (!ac.waitForServer()) {
    }
    ROS_INFO_STREAM("done!");

    if (waitingForMap){
	ROS_INFO_STREAM("Waiting for map");
    	ros::spinOnce();
    }

    newGoal = true;
    sentGoal = false;
    bool firstStart = true;
    int count = 0;    

    ros::Time startGoal;

    Cord goalCord;
    while (ros::ok() ) {        
        if (firstStart & !sentGoal){
	    // Create goal for robot
    	    move_base_msgs::MoveBaseGoal goal;
	    // Set header information for goal message
    	    goal.target_pose.header.frame_id = "map";
    	    goal.target_pose.header.stamp = ros::Time::now();
	    //Set goal coordinates
	    switch(count) {
		case 0: 
			goalCord.x = 8.0;
			goalCord.y = 8.0;
			break;
		case 1: 
			goalCord.x = 8.0;
			goalCord.y = -8.0;
			break;
		case 2: 
			goalCord.x = -8.0;
			goalCord.y = -8;
			break;
		case 3: 
			goalCord.x = -8.0;
			goalCord.y = 8.0;
			//End of start
			firstStart = false;
			break;
		default:
			break;	    		
	    }
	    goal.target_pose.pose.position.x = goalCord.x;
    	    goal.target_pose.pose.position.y = goalCord.y;	    
	    goal.target_pose.pose.orientation.w = 1;

	    //Send goal to robot
	    ROS_INFO_STREAM("Sending goal: Case: " << count++ );
            prevCords.push_back(goalCord);
    	    ac.sendGoal(goal,&serviceDone,&serviceActivated,&serviceFeedback);
	    startGoal = ros::Time::now();
	    sentGoal = true;   
	}
        // If we need a new pose wait
	else if (newGoal & !firstStart) {
	    //int randNum = rand()%(max-min + 1) + min;
	    Cord tempCord;
	    tempCord.x = float(rand())/float(RAND_MAX) * 17 - 8;
	    tempCord.y = float(rand())/float(RAND_MAX) * 17 - 8;
	    ROS_INFO_STREAM("Attempting goal: (" << tempCord.x << ", " << tempCord.y << ")"  );
	    bool coordSafe = isFree(tempCord);
            bool tooClose = sqrt( pow(tempCord.x-curX,2) - pow(tempCord.y-curY,2) ) < 3.0;
	    bool alreadyTried = beenNearCord(tempCord);
	    if ( coordSafe && !tooClose && !alreadyTried){
		goalCord.x = tempCord.x;
		goalCord.y = tempCord.y;
		newGoal = false;
	    }
	}
	// Have a target pose, send the goal
	else if ( not sentGoal ) {
	    // Create goal for robot
   	    move_base_msgs::MoveBaseGoal goal;
	    // Set header information for goal message
    	    goal.target_pose.header.frame_id = "map";
    	    goal.target_pose.header.stamp = ros::Time::now();
			
	    //Set goal coordinates
    	    goal.target_pose.pose.position.x = goalCord.x;
    	    goal.target_pose.pose.position.y = goalCord.y;
	    goal.target_pose.pose.orientation.w = 1;


	    //Send goal to robot
	    ROS_INFO_STREAM("Sending goal: (" << goalCord.x << ", " << goalCord.y << ")"  );
	    prevCords.push_back(goalCord);
    	    ac.sendGoal(goal,&serviceDone,&serviceActivated,&serviceFeedback);
	    startGoal = ros::Time::now();
	    sentGoal = true;
	}
        else if (ros::Time::now() >= (startGoal + ros::Duration(45.0)) ){
	    ROS_INFO_STREAM("Service running longer than 45 seconds, cancelling goal");
	    ac.cancelGoal();
	    // Set flags now to avoid delay during cancel call.
	    newGoal = true; // enable receiving a new goal
    	    sentGoal = false; // enable to recieve new goal	
	}
        ros::spinOnce();
    }   
    return 0;
}
