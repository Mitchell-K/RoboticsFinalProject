#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Pose2D.h>
#include <angles/angles.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/OccupancyGrid.h>


//Globals
bool newGoal;
bool sentGoal;
bool waitingForMap;
nav_msgs::OccupancyGrid map;
float curX = 0, curY = 0;

bool isFree(float xx, float yy){
    int xGrid = map.info.width/2 + xx/map.info.resolution;
    int yGrid = map.info.height/2 + yy/map.info.resolution;
    int i = xGrid + map.info.width*yGrid;
    ROS_INFO_STREAM("Checking (" << xx << ", " << yy << ") on costmap");
    if ( (int)map.data[i] >= 90 | (int)map.data[i] == -1 ) {
	ROS_INFO_STREAM("Bad coordinate");
	return false;
    }
    return true;
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

    newGoal = true;
    sentGoal = false;
    bool firstStart = true;
    int count = 0;    

    ros::Time startGoal;

    float goalX, goalY;

    while (ros::ok() ) {
        if (waitingForMap){
	    ROS_INFO_STREAM("Waiting for map");
	}
        if (firstStart & !sentGoal){
	    // Create goal for robot
    	    move_base_msgs::MoveBaseGoal goal;
	    // Set header information for goal message
    	    goal.target_pose.header.frame_id = "map";
    	    goal.target_pose.header.stamp = ros::Time::now();
	    //Set goal coordinates
	    switch(count) {
		case 0: 
    	    		goal.target_pose.pose.position.x = 8;
    	    		goal.target_pose.pose.position.y = 8;
			break;
		case 1: 
    	    		goal.target_pose.pose.position.x = 8;
    	    		goal.target_pose.pose.position.y = -8;
			break;
		case 2: 
    	    		goal.target_pose.pose.position.x = -8;
    	    		goal.target_pose.pose.position.y = -8;
			break;
		case 3: 
    	    		goal.target_pose.pose.position.x = -8;
    	    		goal.target_pose.pose.position.y = 8;
			//End of start
			firstStart = false;
			break;
		default:
			break;	    		
	    }	    
	    goal.target_pose.pose.orientation.w = 1;


	    //Send goal to robot
	    ROS_INFO_STREAM("Sending goal: Case: " << count++ );
    	    ac.sendGoal(goal,&serviceDone,&serviceActivated,&serviceFeedback);
	    startGoal = ros::Time::now();
	    sentGoal = true;   
	}
        // If we need a new pose wait
	// *****This will need more loagic **************
	else if (newGoal & !firstStart) {
	    //int randNum = rand()%(max-min + 1) + min;
	    float tempX = rand()%(17) - 8;
	    float tempY = rand()%(17) - 8;
	    ROS_INFO_STREAM("Attempting goal: (" << tempX << ", " << tempY << ")"  );
	    bool coordSafe = isFree(tempX, tempY);
            bool tooClose = sqrt( pow(tempX-curX,2) - pow(tempY-curY,2) ) < 3.0;
	    if ( coordSafe && !tooClose ){
		goalX = tempX;
		goalY = tempY;
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
    	    goal.target_pose.pose.position.x = goalX;
    	    goal.target_pose.pose.position.y = goalY;
	    goal.target_pose.pose.orientation.w = 1;


	    //Send goal to robot
	    ROS_INFO_STREAM("Sending goal: (" << goalX << ", " << goalY << ")"  );
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
