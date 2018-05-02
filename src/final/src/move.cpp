#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Pose2D.h>
#include <angles/angles.h>
#include <tf2/LinearMath/Quaternion.h>

//Globals
bool newGoal;
bool sentGoal;

float curX = 0, curY = 0;

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

    float goalX, goalY, goalTheta;

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
    	    		goal.target_pose.pose.position.x = 7;
    	    		goal.target_pose.pose.position.y = 7;
			break;
		case 1: 
    	    		goal.target_pose.pose.position.x = -7;
    	    		goal.target_pose.pose.position.y = 7;
			break;
		case 2: 
    	    		goal.target_pose.pose.position.x = -7;
    	    		goal.target_pose.pose.position.y = -7;
			break;
		case 3: 
    	    		goal.target_pose.pose.position.x = 7;
    	    		goal.target_pose.pose.position.y = -7;
			break;
		default:
			//End of start
			firstStart = false;
			break;		    		
	    }	    
 	    // Make quaternion for goal
	    tf2::Quaternion q;
	    q.setRPY(0, 0, 45); // Sets angle of robot to 45 degrees relative to map, dont edit first two numbers
	    // Set the goal Orientation
    	    goal.target_pose.pose.orientation.x = q.x();
	    goal.target_pose.pose.orientation.y = q.y();
	    goal.target_pose.pose.orientation.z = q.z();
	    goal.target_pose.pose.orientation.w = q.w();


	    //Send goal to robot
	    ROS_INFO_STREAM("Sending goal: Case: " << count++ );
    	    ac.sendGoal(goal,&serviceDone,&serviceActivated,&serviceFeedback);
	    startGoal = ros::Time::now();
	    sentGoal = true;   
	}
        // If we need a new pose wait
	// *****This will need more loagic **************
	else if (newGoal) {
	    //int randNum = rand()%(max-min + 1) + min;
	    float tempX = rand()%(17) - 8;
	    float tempY = rand()%(17) - 8;
	    float tempTheta = 45;
	    if ( sqrt( pow(tempX-curX,2) - pow(tempY-curY,2) ) > 3.0){
		goalX = tempX;
		goalY = tempY;
		goalTheta = tempTheta;
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

	    // Make quaternion for goal
	    tf2::Quaternion q;
	    q.setRPY(0, 0, goalTheta); // Sets angle of robot to 45 degrees relative to map, dont edit first two numbers
	    // Set the goal Orientation
    	    goal.target_pose.pose.orientation.x = q.x();
	    goal.target_pose.pose.orientation.y = q.y();
	    goal.target_pose.pose.orientation.z = q.z();
	    goal.target_pose.pose.orientation.w = q.w();


	    //Send goal to robot
	    ROS_INFO_STREAM("Sending goal: (" << goalX << ", " << goalY << ", " << goalTheta << ")"  );
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
