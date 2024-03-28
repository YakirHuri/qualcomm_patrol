

#include <patrol_manager/PatrolManager.h>
#include <signal.h>
#include <ros/ros.h>

using namespace std;



bool readWaypoints(vector<WayPoint>& waypoints) {
	
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

			WayPoint waypoint;
			waypoint.w_pose_ = pose;
			waypoints.push_back(waypoint);

			if (parsedValues < 3) {
				ROS_ERROR("Failed to parse a waypoint (line %i)", line);
				return false;
			}

			line++;
		}

	} else {


		ROS_ERROR("Error: waypoints parameter does not exists or empty");
		
		return false;
	}

	return true;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "patrol_manager_node");

	vector<WayPoint> waypoints;

	if (readWaypoints(waypoints) == false) {
		
		ROS_WARN("Waypoints list is empty, exiting");
		cerr<<" Waypoints list is empty, exiting "<<endl;
		
		return -1;

	} else {

		signal(SIGINT, (void (*)(int))PatrolManager::mySigintHandler); 
	
		PatrolManager PatrolManager(waypoints);

		PatrolManager.run();

		ros::spin();
	}




	return 0;
}



// #include <opencv2/aruco.hpp>
// #include <opencv2/opencv.hpp>
// using namespace cv;


// static bool readCameraParameters(string filename, Mat_<float> &camMatrix, Mat_<float> &distCoeffs) {
   

// 	float fx_ = 1280;
// 	float fy_ = 720;

// 	float cx_ = fx_ / 2;
// 	float cy_ = fy_ / 2;

// 	camMatrix << fx_, 0., cx_,
//                         0., fy_, cy_,
//                         0., 0., 1.;

            
// 	distCoeffs <<0, 0,
// 				0, 0,
// 				0 ;
// }

// int main() {

// 	string filename = "";

// 	cv::VideoCapture cap(0); 	
//     if(!cap.isOpened())
//     {
//         cout << "Error can't find the file"<<endl;
//     }
// 	Mat_<float>  cameraMatrix, distCoeffs;
// 	cameraMatrix = Mat_<float>(3,3); 

// 	distCoeffs = Mat_<float>(1,5); 
// 	// You can read camera parameters from tutorial_camera_params.yml
// 	readCameraParameters(filename, cameraMatrix, distCoeffs); // This function is located in detect_markers.cpp
// 	cv::Ptr<cv::aruco::Dictionary> dictionary = 
// 		cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
// 	while (  true	)
// 	{
// 		cv::Mat image, imageCopy;
// 		// image = imread("/home/yakir/Pictures/aruco.png",1);
// 		cap.read(image);
// 		image.copyTo(imageCopy);
// 		std::vector<int> ids;
// 		std::vector<std::vector<cv::Point2f>> corners;
// 		cv::aruco::detectMarkers(image, dictionary, corners, ids);
// 		// if at least one marker detected
// 		if (ids.size() > 0)
// 		{	
// 			cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
// 			std::vector<cv::Vec3d> rvecs, tvecs;
// 			cv::aruco::estimatePoseSingleMarkers(corners, 0.1, cameraMatrix, distCoeffs, rvecs, tvecs);
// 			// draw axis for each marker
// 			for (int i = 0; i < ids.size(); i++)
// 				cv::drawFrameAxes(imageCopy, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);
// 		}
// 		cv::imshow("image", imageCopy);
// 		char key = (char)cv::waitKey(10);
// 		if (key == 27)
// 			break;

// 	}

// }