// Include the ROS C++ APIs
#include <ros/ros.h>
#include <iostream>
#include <stdlib.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
//stuff for reading yaml files
#include <yaml-cpp/yaml.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/Pose.h>
#include "spqr_location_service/GetLocation.h"
#include <cmath>
#include <string.h>
#include <fstream>
#include <sstream>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <graph.h>

typedef Eigen::Matrix<double, 2, 1> Vector2;

struct callBack_userdata{
	float mapResolution;
        cv::Mat defaultmap;
	cv::Mat mapDraw;
	cv::Mat map;
	cv::Mat mapMask;
	cv::Mat laserScanMap;
        cv::Mat obstacleMask;
	cv::Mat mapAStarAS;
	cv::Mat mapCubicAS;
        cv::Mat mapBezierAS; 
	cv::Mat mapReplanTopolGraphAS; 
	
	geometry_msgs::Pose mapOrigin;
  //add vector (posx posy) black points
  
	std::vector <float> robot_state;
	std::vector <std::pair<float,float> > scan_rays;
	std::vector <float> scan_atan;
  //add vector scan-rays update
};

class Simulator {

    private: 
	 ros::NodeHandle nh;
	 callBack_userdata user;
	

    public:
	Simulator(){};
	Graph g;
	std::vector<int> AStarPath;
	std::vector<Vector2> cubicSplinePos;
	std::vector<Vector2> brezierPos;
	
	callBack_userdata getUserdata() { return user; }
	std::vector<std::string> explode(const std::string& str, const char& ch);
	std::pair<std::string, std::string> getMapImgInfos(std::string config_file_path);
	cv::Mat manuallyConvertMapToImg(const nav_msgs::OccupancyGrid& map, int &map_height, int &map_width);
	void rebuildMapFromTresholds(cv::Mat &inputMap, cv::Mat &outputMap, float map_occupied_threshold, float map_free_threshold);
	void loadMapFromMapServer();
	cv::Point2f pixelToPoint(int x, int y, float map_resolution, geometry_msgs::Pose map_origin);
	cv::Point2f pointToPixel(cv::Point2f pt, float map_resolution, geometry_msgs::Pose map_origin);
        void drawRobot(cv::Mat inputMap, float mapRes, geometry_msgs::Pose mapOrig, std::vector <float> robot_state, std::vector <std::pair<float,float> > laser);
	void drawAStarPath(cv::Mat inputMap, pGraph nodelist, float mapRes, geometry_msgs::Pose mapOrig, std::vector<int> path);
	void drawBrezierCurvePath(cv::Mat inputMap, const std::vector<Vector2>& complete_path, float mapRes, geometry_msgs::Pose mapOrig);
	void drawCubicSplinePath(cv::Mat inputMap, const std::vector<Vector2>& complete_path, float mapRes, geometry_msgs::Pose mapOrig);
	void drawReplanGraph(cv::Mat inputMap, float mapRes, geometry_msgs::Pose mapOrig);  
	
};


