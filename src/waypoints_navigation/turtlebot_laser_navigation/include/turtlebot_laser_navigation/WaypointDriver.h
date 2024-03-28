/*
 * WaypointDriver.h
 *
 *  Created on: Nov 9, 2016
 *      Author: blackpc
 */

#ifndef INCLUDE_TURTLEBOT_LASER_NAVIGATION_WAYPOINTDRIVER_H_
#define INCLUDE_TURTLEBOT_LASER_NAVIGATION_WAYPOINTDRIVER_H_


#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <actionlib/client/simple_action_client.h>
#include <kobuki_msgs/AutoDockingAction.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <turtlebot_laser_navigation/MoveBaseController.h>


using namespace std;


class WaypointDriver {

public:

	typedef vector<geometry_msgs::PoseStamped> Waypoints;

public:

	WaypointDriver(const Waypoints& waypoints);

	virtual ~WaypointDriver();

	void start();

public:

	Waypoints waypoints_;

private:

	void undockFromCharger();

	void autoDock();

	void markersTimerCallback(const ros::TimerEvent&);

	void visualizeGoals();

	void waitForAckNextPointCallback(const std_msgs::BoolConstPtr &msg);


private:

	ros::Publisher startStopDockingPublisher_;
	ros::Publisher unDockingPublisher_;
	ros::Publisher velocityPublisher_;

	ros::Subscriber moveToNextPointSub_;

	actionlib::SimpleActionClient<kobuki_msgs::AutoDockingAction> dockingActionClient_;

	double waypointWait = 1.0;

	//rotate in place
	bool enableRotateInPlace;
	int durationRotationSeconds_ = 0;
	double rotationSpeed_;

	//move to next point
	bool waitForAckNextPointFlag_;
	bool currentPointReached_ = true;
	bool canMoveNextPoint_;


};

#endif /* INCLUDE_TURTLEBOT_LASER_NAVIGATION_WAYPOINTDRIVER_H_ */
