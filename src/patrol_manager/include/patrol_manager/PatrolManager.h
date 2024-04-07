/*
 * PatrolManager.h
 *
 *  Created on: Nov 9, 2021
 *      Author: yakirhuri
 */

#ifndef INCLUDE_patrol_manager_PatrolManager_H_
#define INCLUDE_patrol_manager_PatrolManager_H_


#include <vector>
#include<string.h>
#include <tf/tf.h>
#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <nav_msgs/Path.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <string>

#include <angles/angles.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>


#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

#include "MoveBaseController.h"

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;


struct WayPoint {

	geometry_msgs::PoseStamped w_pose_;

	bool status_ = false;
};


enum RobotState
{
  IDLE = 0,
  PATROL = 1,
  STOP = 2,
  CONTINUE = 3,
  GO_HOME = 4
};


class PatrolManager {



public:

	PatrolManager(vector<WayPoint> waypoints){

		waypoints_ = waypoints;


		ros::NodeHandle node_p("~");

		node_p.param("wait_duration", wait_duration_seconds_, 1.0);		

		node_p.param("initial_pose_x", initial_pose_x_, 0.0);
		node_p.param("initial_pose_y", initial_pose_y_, 0.0);
		node_p.param("initial_pose_a", initial_pose_a_, 0.0);	


		initialWaypoint.header.frame_id = globalFrame_;
        initialWaypoint.header.stamp = ros::Time::now();
		initialWaypoint.pose.orientation =  tf::createQuaternionMsgFromYaw(initial_pose_a_);
		initialWaypoint.pose.position.x = 	initial_pose_x_;
		initialWaypoint.pose.position.y = 	initial_pose_y_;

		//pubs
		goals_marker_array_publisher_ =
        	node.advertise<visualization_msgs::MarkerArray>("/waypoints_markers", 10);		

		patrol_state_publisher_ =
        	node.advertise<std_msgs::String>("/patrol_state", 10);	

		initial_point_marker_publisher_ =
        	node.advertise<visualization_msgs::Marker>("/initial_point_marker", 10);			


		//subs
		action_sub_ = node.subscribe<std_msgs::String>("/action", 1, 
			&PatrolManager::actionCallback, this);	

		setPatrolState(IDLE);

		waypointsTimer_ = node.createTimer(ros::Rate(1), 
                &PatrolManager::waypointsTimerCallback, this);

		robotStateTimer_ = node.createTimer(ros::Rate(1), 
                &PatrolManager::robotStateTimerCallback, this);

		initialPointTimer_ = node.createTimer(ros::Rate(1), 
                &PatrolManager::initialPointCallback, this);			
	
	}

	virtual ~PatrolManager(){

	}

	static void mySigintHandler(int sig, void *ptr)
    {

        cerr << " user pressed CTRL+C " << endl;
    }

	void waypointsTimerCallback(const ros::TimerEvent&) {

      try
      {
        publishWaypointsWithStatus();
      }
      catch (std::runtime_error e)
      {
          ROS_ERROR("%s", e.what());
      }
     

    }	

	void initialPointCallback(const ros::TimerEvent&) {

      try
      {
        publishInitialPose();

      }
      catch (std::runtime_error e)
      {
          ROS_ERROR("%s", e.what());
      }
     

    }	

	void robotStateTimerCallback(const ros::TimerEvent&) {

      try
      {	
		string patrolStateStr = "none";
		switch (patrolState_)
		{
			case IDLE: {
				patrolStateStr = "IDLE";
				break;
				
			}
			case PATROL: {
				patrolStateStr = "PATROL";	
				break;			
			}
			case STOP: {
				patrolStateStr = "STOP";
				break;					
			}			
			case CONTINUE: {
				patrolStateStr = "CONTINUE";
				break;
			}
			case GO_HOME: {
				patrolStateStr = "GO_HOME";
				break;
			}
						
		}

		std_msgs::String msg;
		msg.data = patrolStateStr;
        patrol_state_publisher_.publish(msg);
      }
      catch (std::runtime_error e)
      {
          ROS_ERROR("%s", e.what());
      }
     

    }	

	void setPatrolState(RobotState state)
	{	
		patrolState_ = state;		
	}

	bool checkLocalizationOk() {

        tf::StampedTransform transform;

        try
        {
            //get current robot pose
            tfListener_.lookupTransform(globalFrame_, odomFrame_,
                                       ros::Time(0), transform);

            return true;
        }

        catch (...)
        {
            //cerr << " error between map to odom"<<endl;
            return false;
        }
    }

	double distanceCalculate(cv::Point2d p1, cv::Point2d p2)
    {
        double x = p1.x - p2.x; //calculating number to square in next step
        double y = p1.y - p2.y;
        double dist;

        dist = pow(x, 2) + pow(y, 2); //calculating Euclidean distance
        dist = sqrt(dist);

        return dist;
    }

	
	void run(){		


		ROS_INFO("Waiting for /move_base action server...");
		moveBaseController_.waitForServer();
		
		ros::Duration(10).sleep();
		
		ROS_INFO("Connected to /move_base !!!!!!!!!!!!!!\n");

		ros::Rate loop_rate(5);
		while( ros::ok() ) {

			ros::spinOnce();

			switch (patrolState_)
			{
				case IDLE: {

					if	( lastAction_ =="start_patrol") {

						initLastAction();	

						setPatrolState(PATROL);
						
						break;

					}
					else if	( lastAction_ =="go_home") {

						initLastAction();	

						setPatrolState(GO_HOME);
						
						break;

					}
					else {
						
						initLastAction();

						setPatrolState(IDLE);
						
						break;
					}
					
				}
				case PATROL: {
					
					//each iteration is 1 round of navigation	
					while( ros::ok() ) {

						ros::spinOnce();
						cerr<<"PATROL "<<endl;

						auto state = runPatrolRound();
						initLastAction();	

						if (state == STOP){
							setPatrolState(STOP);
							break;
						}

						if (state == GO_HOME){
							setPatrolState(GO_HOME);
							last_waypoint_ = 0;
							break;
						} 
						initWaypoints();
					}					
				}
				case STOP: {					

					while( ros::ok() ) {

						ros::spinOnce();

						if	( lastAction_ =="go_home") {
							cerr<<" GOT go_home - > GO_HOME "<<endl;
							initLastAction();	
							last_waypoint_ = 0;

							setPatrolState(GO_HOME);
							break;

						} 
						
						if ( lastAction_ =="continue") { 
							cerr<<" GOT continue - > CONTINUE "<<endl;
							initLastAction();	
							setPatrolState(CONTINUE);
							break;
						}

						setPatrolState(STOP);
						initLastAction();
					}

					initLastAction();
					break;
					
				}
				
				case CONTINUE: {
					cerr<<"CONTINUE "<<endl;	
					setPatrolState(PATROL);
					break;
				}
				case GO_HOME: {
					cerr<<"GO_HOME "<<endl;
					last_waypoint_ = 0;

					if ( goHome() == true){
						setPatrolState(IDLE);
						break;
					}
					initLastAction();				
				}
						
			}

			loop_rate.sleep();


		}


	}

	void initLastAction(){

		lastAction_ = "none";
	}
	RobotState runPatrolRound(){		

		//verify amcl works and the robot have location
		bool recv_map_odom = false;
		cerr<<" waiting for robot's location ... "<<endl;

		while( ros::ok() && !recv_map_odom) {	
			
			// cerr<<" waiting for robot's location ... "<<endl;
			ros::spinOnce();

			recv_map_odom =  checkLocalizationOk();
			
		}		


		for (int i = last_waypoint_; i < waypoints_.size(); i++){
			
			bool reachedGoal = false;		
			
			cerr<<" NAVIGATE TO PIOINT "<<i<<endl;
			moveBaseController_.navigate(waypoints_[i].w_pose_);

			while(ros::ok()) {

				ros::spinOnce();

				updateRobotLocation();

				if	( lastAction_ =="stop") {

					last_waypoint_ = (i) % waypoints_.size();

					moveBaseController_.moveBaseClient_.cancelAllGoals();

					cerr<<" want to stop, next waypoint wiil be  "<<last_waypoint_<<endl;
					
					RobotState state = STOP;
					return state;

				} else if ( lastAction_ == "go_home") {

					last_waypoint_ = 0;

					cerr<<" want to go home!! "<<endl;


					moveBaseController_.moveBaseClient_.cancelAllGoals();
					
					RobotState state = GO_HOME;
					return state;
				}

				float distDromRobot =
					distanceCalculate(cv::Point2d(waypoints_[i].w_pose_.pose.position.x, waypoints_[i].w_pose_.pose.position.y),
									cv::Point2d(robotPose_.pose.position.x, robotPose_.pose.position.y));

				moveBaseController_.moveBaseClient_.waitForResult(ros::Duration(0.1));
				
				if( waypoints_[i].status_ ==  true ||
					moveBaseController_.moveBaseClient_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED 
					||  moveBaseController_.moveBaseClient_.getState() == actionlib::SimpleClientGoalState::ABORTED
					||  moveBaseController_.moveBaseClient_.getState() == actionlib::SimpleClientGoalState::REJECTED) {
					
					if( moveBaseController_.moveBaseClient_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED ||
						waypoints_[i].status_ ==  true){
						waypoints_[i].status_ = true;
						cerr<<" goal reached !! "<<endl;
						
					} else {

						if (distDromRobot < 0.15)
						{	
							waypoints_[i].status_ = true;
							moveBaseController_.moveBaseClient_.cancelGoal();

						} else {

							waypoints_[i].status_ = false;
						}						

					}

					// if goal reached, wait for X seconds
					if( waypoints_[i].status_){

						auto start = ros::WallTime::now();

						while (ros::ok()) {

							ros::spinOnce();

							auto end = ros::WallTime::now();

							auto duration = (end - start).toSec();


							if( duration > wait_duration_seconds_){
								break;
							}							
						}
					} 					

					break;
				} 
			}

		}

		cerr<<"FINSIHED ROUND !!!! "<<endl;

		RobotState state = PATROL;
		last_waypoint_ = 0;
		return state;
	}
	
	

private:

	void actionCallback(const std_msgs::String::ConstPtr& actionMsg)
	{	
		lastAction_ = actionMsg->data;
		cerr<<"got from callback "<<lastAction_<<endl;
		
	}


	void initWaypoints(){

		for (int i = 0; i < waypoints_.size(); i++){
			
			waypoints_[i].status_ = false;		

		}
	}


	bool goHome() {

		setYawTolerance(0.0);		

		cerr<<" nav to initialWaypoint "<<endl;

		moveBaseController_.navigate(initialWaypoint);

		// nav to docking station
		while(ros::ok()) {

			ros::spinOnce();

			updateRobotLocation();

			moveBaseController_.moveBaseClient_.waitForResult(ros::Duration(0.1));

			float distDromRobot =
					distanceCalculate(cv::Point2d(initialWaypoint.pose.position.x, initialWaypoint.pose.position.y),
									cv::Point2d(robotPose_.pose.position.x, robotPose_.pose.position.y));

			
			if (distDromRobot < 0.15)
			{	
				
				moveBaseController_.moveBaseClient_.cancelGoal();
				return true;

			}

			if( moveBaseController_.moveBaseClient_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED 
				||  moveBaseController_.moveBaseClient_.getState() == actionlib::SimpleClientGoalState::ABORTED
				||  moveBaseController_.moveBaseClient_.getState() == actionlib::SimpleClientGoalState::REJECTED){
				
				if( moveBaseController_.moveBaseClient_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
					
					cerr<<" reached the start !!!! "<<endl;

					
				}  else {

					cerr<<"failed to reached the start !!!! "<<endl;

				}			
				
			}
		}

		return true;

	}


	bool checkIfRobtotIsCharged() {

		return true;
	}

	void publishInitialPose() {
		
		visualization_msgs::Marker marker;
		marker.action = visualization_msgs::Marker::ADD;
		marker.type = visualization_msgs::Marker::ARROW;
		marker.header.frame_id = globalFrame_;
		marker.header.stamp  = ros::Time::now(); 
		marker.id = 1;
		marker.ns ="initial_point";
		marker.pose.position.x = initial_pose_x_;
		marker.pose.position.y = initial_pose_y_;
		marker.pose.position.z = 1.0;

		marker.pose.orientation = tf::createQuaternionMsgFromYaw(initial_pose_a_);
		marker.scale.x = 0.5;
		marker.scale.y = 0.5;                
		marker.scale.z = 0.5;

		marker.color.r = 0.0f;
		marker.color.g = 0.0f;
		marker.color.b = 1.0f;
		marker.color.a = 1.0;

		initial_point_marker_publisher_.publish(marker);	

	}

	void setYawTolerance(float yawTolerance = 0.0)
	{
		dynamic_reconfigure::ReconfigureRequest srv_req;
		dynamic_reconfigure::ReconfigureResponse srv_resp;
		dynamic_reconfigure::DoubleParameter float_param;
		dynamic_reconfigure::Config conf;

		float_param.name = "yaw_goal_tolerance";
		float_param.value = yawTolerance;
		conf.doubles.push_back(float_param);

		srv_req.config = conf;
		if (ros::service::call("/move_base/DWAPlannerROS/set_parameters", srv_req, srv_resp))
		{
		cerr << " after set reverse " << endl;
		}
		else
		{
		cerr << "errrrrrrrrrrrrrrrrrrrrr " << endl;
		}

		ros::Duration(0.5).sleep();
		
	}


	
	
	void publishWaypointsWithStatus() {

		visualization_msgs::MarkerArray markers;

		for (int i = 0; i < waypoints_.size(); i++) {

			visualization_msgs::Marker marker;
			marker.lifetime = ros::Duration(100.0);
			marker.action = visualization_msgs::Marker::ADD;
			marker.type = visualization_msgs::Marker::ARROW;
			marker.header.frame_id = globalFrame_;
			marker.header.stamp  = ros::Time::now(); 
			marker.id = i;
			marker.ns = "waypoints";
			marker.pose.position = waypoints_[i].w_pose_.pose.position;
			marker.pose.position.z = 1.0;
			marker.pose.orientation = waypoints_[i].w_pose_.pose.orientation;
			marker.scale.x = 0.5;
			marker.scale.y = 0.5;                
			marker.scale.z = 0.5;

			if( waypoints_[i].status_){
				marker.color.r = 0.0f;
				marker.color.g = 1.0f;
				marker.color.b = 0.0f;
				marker.color.a = 1.0;
		
			} else {
				marker.color.r = 1.0f;
				marker.color.g = 0.0f;
				marker.color.b = 0.0f;
				marker.color.a = 1.0;
			}
			
			
		
			markers.markers.push_back(marker);              
		}

		goals_marker_array_publisher_.publish(markers);

	}

	bool updateRobotLocation()
    {

        tf::StampedTransform transform;

        try
        {
            // get current robot pose
            tfListener_.lookupTransform(globalFrame_, baseFrame_,
                                        ros::Time(0), transform);

            robotPose_.header.frame_id = globalFrame_;
            robotPose_.header.stamp = ros::Time::now();
            robotPose_.pose.position.x = transform.getOrigin().x();
            robotPose_.pose.position.y = transform.getOrigin().y();
            robotPose_.pose.position.z = 0;
            robotPose_.pose.orientation.x = transform.getRotation().x();
            robotPose_.pose.orientation.y = transform.getRotation().y();
            robotPose_.pose.orientation.z = transform.getRotation().z();
            robotPose_.pose.orientation.w = transform.getRotation().w();       

            return true;
        }

        catch (...)
        {
            cerr << " error between " << globalFrame_ << " to " << baseFrame_ << endl;
            return false;
        }
    }

	
	

	
	
private:

	ros::NodeHandle node;

	ros::Timer waypointsTimer_;

	ros::Timer robotStateTimer_;

	ros::Timer initialPointTimer_;


	geometry_msgs::PoseStamped robotPose_; // on map frame

	geometry_msgs::PoseStamped initialWaypoint;

	RobotState patrolState_ ;
	
	//move-base
	MoveBaseController moveBaseController_;


	tf::TransformListener tfListener_;
	string globalFrame_ = "map";
	string baseFrame_ = "base_link";
	string odomFrame_ = "odom";

	//ros params	
	double wait_duration_seconds_ = 0.0;

	int last_waypoint_ = 0;

	double initial_pose_x_ = 0.0;
	double initial_pose_y_ = 0.0;
	double initial_pose_a_ = 0.0;

	string lastAction_ = "none";

	ros::Publisher goals_marker_array_publisher_ ;
	ros::Publisher patrol_state_publisher_;
	ros::Publisher initial_point_marker_publisher_;

	ros::Subscriber action_sub_;


	// system params	
	vector<WayPoint> waypoints_;

};

#endif /* INCLUDE_patrol_manager_PatrolManager_H_ */
