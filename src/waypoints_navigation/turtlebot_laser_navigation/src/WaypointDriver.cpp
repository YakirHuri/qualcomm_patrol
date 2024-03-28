/*
 * WaypointDriver.cpp
 *
 *  Created on: Nov 9, 2016
 *      Author: blackpc
 */


#include <turtlebot_laser_navigation/WaypointDriver.h>


WaypointDriver::WaypointDriver(const Waypoints& waypoints)
	: waypoints_(waypoints), dockingActionClient_("dock_drive_action") {
	ros::NodeHandle node;	
	
	startStopDockingPublisher_ = node.advertise<std_msgs::Bool>(
			"/start_stop_docking", 1, false);

	unDockingPublisher_ = node.advertise<std_msgs::Bool>(
		"/undock_from_charger", 1, false);

	velocityPublisher_ = node.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity" , 1, true);		

	moveToNextPointSub_  = node.subscribe("/move_to_next_point", 1,
         &WaypointDriver::waitForAckNextPointCallback, this); 


   canMoveNextPoint_ = false;

   currentPointReached_ = true;
}

WaypointDriver::~WaypointDriver() {
}


void WaypointDriver::waitForAckNextPointCallback(const std_msgs::BoolConstPtr &msg){

	cerr<<" inside ccccccccccccccccccccccccaliback "<<endl;
	//can move to next point
	if( msg->data == true ){

		if(currentPointReached_ ){

			cerr<<"CALLBACK: canMoveNextPoint_ is true "<<endl;
			canMoveNextPoint_ = true;
			return;
		} else {

			cerr<<"11111111111111111111111111111111111 "<<endl;
		}
	} else {
		
		canMoveNextPoint_ = false;
		cerr<<"222222222222222222222222222222222222 "<<endl;


	}

	 
}
void WaypointDriver::start() {

	ros::NodeHandle node("~");


	node.param("wait", waypointWait, 0.0);

	//rotate in place
	node.param("enable_rotate_in_place", enableRotateInPlace, false);
	node.param("duration_rotation_seconds", durationRotationSeconds_, 0);
	node.param("rotation_speed", rotationSpeed_, 0.0);

	//move to next point
	node.param("wait_for_ack_next_point", waitForAckNextPointFlag_, false);


	MoveBaseController moveBaseController;


	ROS_INFO("Wait time = %fs", waypointWait);
	ROS_INFO("Waiting for /move_base action server...");
	moveBaseController.waitForServer();
	ros::Duration(1).sleep();
	ROS_INFO("Connected to /move_base!\n");

	undockFromCharger();

	ros::Duration(1).sleep();

	ROS_INFO("Starting navigation (%lu waypoints):", waypoints_.size());

	

	for (int i = 0; i < waypoints_.size(); ++i) {
		auto waypoint = waypoints_[i];

		cerr<<"waypoint.pose.position.x "<<waypoint.pose.position.x<<", waypoint.pose.position.y "<<waypoint.pose.position.y<<endl;
		//wait for user or ignore in disable
		
		cerr<<" canMoveNextPoint_ "<<canMoveNextPoint_<<" waitForAckNextPointFlag_ "<<waitForAckNextPointFlag_<<endl;
		while (ros::ok()) {
			
			if ( ( canMoveNextPoint_ && waitForAckNextPointFlag_) 
				|| waitForAckNextPointFlag_ == false){

				cerr<<" break from the loop "<<endl;	
				break;	
			}

			ros::spinOnce();

		}

		
		canMoveNextPoint_ = false;
		currentPointReached_ = false;

		cerr<<" start navigate waypoint "<<endl;
		//navigate to the point			
		moveBaseController.navigate(waypoint);
		bool result = moveBaseController.wait();
		ROS_INFO("\t #%i: %s", i + 1, result ? "Waypoint reached!" : "Failed to reach waypoint");
		cerr<<" Waypoint reached! -> currentPointReached_ "<<currentPointReached_<<endl;
		//point reached
		currentPointReached_ = true;

		auto start = ros::WallTime::now();			
		// if need also to rotate in place
		if( enableRotateInPlace ){
			
			//ros::Rate rate(20);

			geometry_msgs::Twist rotationVelocity;
			rotationVelocity.linear.x= 0;
			rotationVelocity.linear.y= 0;
			rotationVelocity.linear.z= 0;
			rotationVelocity.angular.x = 0;
			rotationVelocity.angular.y = 0;		
			//clockwise	
			rotationVelocity.angular.z = -abs(rotationSpeed_);

			while (true){

				auto end = ros::WallTime::now();
				auto duration = (end - start).toSec();
				if( duration > durationRotationSeconds_){ 
					break;
				}

				velocityPublisher_.publish(rotationVelocity);
			}
		}

		//wait 
		ros::Duration(waypointWait).sleep();		
		
	}

	ROS_INFO("================================================");
	ROS_INFO("Navigation done");

	//autoDock
	//dock command
	std_msgs::Bool msg;
	msg.data = true;
	startStopDockingPublisher_.publish(msg);
}

void WaypointDriver::undockFromCharger() {
	ROS_INFO(" - Undocking from charging station...");

	//un-dock command
	std_msgs::Bool msg;
	msg.data = true;
	unDockingPublisher_.publish(msg);

}

void WaypointDriver::autoDock() {
	ROS_INFO(" - Starting auto-docking algorithm...");
	ROS_INFO("     -- Waiting for docking action server...");

	if (!dockingActionClient_.waitForServer(ros::Duration(10.0))) {
		ROS_WARN("     -- Failed to connect to action server, auto-docking disabled");
		return;
	}

	ROS_INFO("     -- Connected to action server!");
	ROS_INFO("     -- Docking...");

	kobuki_msgs::AutoDockingGoal dockingGoal;

	dockingActionClient_.sendGoal(dockingGoal);

	dockingActionClient_.waitForResult();

	ROS_INFO(" - Docking process finished!");
}

