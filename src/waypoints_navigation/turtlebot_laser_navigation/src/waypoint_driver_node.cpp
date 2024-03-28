/*
 * waypoint_driver_node.cpp
 *
 *  Created on: Nov 9, 2016
 *      Author: blackpc
 */


#include <tf/tf.h>

#include <turtlebot_laser_navigation/WaypointDriver.h>


using namespace std;


WaypointDriver::Waypoints readWaypoints() {
	ros::NodeHandle nodePrivate("~");

	vector<string> waypointsList;

	WaypointDriver::Waypoints waypoints;


	if (nodePrivate.getParam("waypoints", waypointsList)) {

		geometry_msgs::PoseStamped pose;

		int line = 1;

		for(auto waypointString : waypointsList) {
			double heading = 0;

			auto parsedValues = sscanf(waypointString.c_str(), "%lf,%lf,%lf",
					&pose.pose.position.x,
					&pose.pose.position.y,
					&heading);

			pose.header.frame_id = "map";
			pose.header.stamp = ros::Time(0);

			pose.pose.orientation = tf::createQuaternionMsgFromYaw(heading);

			waypoints.push_back(pose);

			ROS_INFO_STREAM(pose);

			if (parsedValues < 3) {
				ROS_ERROR("Failed to parse a waypoint (line %i)", line);
				exit(0);
			}

			line++;
		}

	} else {
		ROS_ERROR("Error: waypoints parameter does not exists or empty");
		exit(0);
	}

	return waypoints;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "waypoint_driver_node");

	ros::NodeHandle node;
	ros::NodeHandle nodePrivate("~");

	auto waypoints = readWaypoints();

	if (waypoints.size() == 0) {
		ROS_WARN("Waypoints list is empty, exiting");
		exit(0);
	}

	WaypointDriver driver(waypoints);
	driver.start();


	ros::spin();

	return 0;
}
