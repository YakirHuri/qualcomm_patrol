/*
 * waypoint_driver_node.cpp
 *
 *  Created on: Nov 9, 2016
 *      Author: blackpc
 */


#include <tf/tf.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;



vector < geometry_msgs::PoseStamped> wayPointsForMarkers_;


ros::Publisher goals_marker_array_publisher_ ;

bool init_ =false;



void visualizeGoals()
{	

	if( wayPointsForMarkers_.size() == 0 || !init_) {
		return;
	}

 	visualization_msgs::MarkerArray markers;
	for (int i = 0; i < wayPointsForMarkers_.size(); i++) {	
		visualization_msgs::Marker marker;
		marker.lifetime = ros::Duration(3.0);
		marker.action = visualization_msgs::Marker::ADD;
		marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
		marker.text = to_string(i);
		marker.header.frame_id = "map";
		marker.header.stamp  = ros::Time::now(); 
		marker.id = rand();
		marker.pose.orientation.w = 1.0;               
		marker.pose.position.x = wayPointsForMarkers_[i].pose.position.x;
		marker.pose.position.y = wayPointsForMarkers_[i].pose.position.y;
		marker.pose.position.z = 0.3;
		marker.scale.x = 0.7;
		marker.scale.y = 0.7;                
		marker.scale.z = 0.7;
		
		marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;
	
		markers.markers.push_back(marker);              
	}

	goals_marker_array_publisher_.publish(markers);
}



void readWaypoints() {
	ros::NodeHandle nodePrivate("~");

	vector<string> waypointsList;


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

			wayPointsForMarkers_.push_back(pose);

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

	init_ = true;

}


void markersTimerCallback(const ros::TimerEvent&) {

	if(wayPointsForMarkers_.size() > 0 ){

		visualizeGoals();
	} else {

	}


}




int main(int argc, char **argv) {
	ros::init(argc, argv, "waypoints_markers_publishers");

	ros::NodeHandle node_;

	goals_marker_array_publisher_ =
        node_.advertise<visualization_msgs::MarkerArray>("/goals", 10);	

	readWaypoints();
	ros::Timer timer = node_.createTimer(ros::Duration(0.1), markersTimerCallback);

	ros::spin();

	return 0;
}
