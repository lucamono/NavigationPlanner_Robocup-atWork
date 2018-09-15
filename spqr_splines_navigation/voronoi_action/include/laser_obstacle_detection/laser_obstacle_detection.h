//Synchronization between 

#include <iostream>

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include "spqr_location_service/GetLocation.h"
#include <cmath>
#include <math.h>
#include <draw_simulator.h>
#include <eigen3/Eigen/Dense>


using namespace std;

typedef std::pair<cv::Point, cv::Point> boundingBox;

class LaserDetection {
public:
  LaserDetection() :

    poseAMCL_sub_( nh_, "/amcl_pose", 1 ),
    base_scan_sub_( nh_, "/base_scan", 1 ),
    sync( MySyncPolicy( 10 ), poseAMCL_sub_, base_scan_sub_ )
    //class constructor
    {	
	int distanceMapSize = 8;
	initEnvironment(distanceMapSize);
	sync.registerCallback( boost::bind( &LaserDetection::callback, this, _1, _2 ) );
    }

    callBack_userdata user;
    Simulator sim;

    //coords of opencv min_max_loc of the obstacle 
    vector<pair<int,int> > objectsDetectedPixel;
    vector<pair<float,float> > objectsDetectedPos;
    //the shape of the obstacle
    std::vector<boundingBox> obstacles;

    bool obstacleFounded = false;
    
    //init variables poseRobot and LaserScan
    double poseAMCLx, poseAMCLy, poseAMCLa;
    float angle_min,angle_max,angle_increment,time_increment,scan_time,range_min,range_max; 
    vector<float> ranges,intensities;

    void callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& amclPose, const sensor_msgs::LaserScanConstPtr& base_scan);
    void initEnvironment(int erosion_size);
    boundingBox getObstacleBBox(cv::Mat obstacleMap, cv::Point max_loc, float mapRes, geometry_msgs::Pose mapOrig, std::vector <float> robot_state);

private:
    ros::NodeHandle nh_;

    typedef message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> AmclPoseSubscriber;
    typedef message_filters::Subscriber<sensor_msgs::LaserScan> LaserSubscriber;
    AmclPoseSubscriber poseAMCL_sub_;
    LaserSubscriber base_scan_sub_;
    typedef message_filters::sync_policies::ApproximateTime< geometry_msgs::PoseWithCovarianceStamped, sensor_msgs::LaserScan> MySyncPolicy;

    message_filters::Synchronizer< MySyncPolicy > sync;
        
    float toEulerianAngle(geometry_msgs::Quaternion q){
	float ysqr = q.y * q.y;
	// yaw (z-axis rotation)
	float t3 = +2.0 * (q.w * q.z + q.x * q.y);
	float t4 = +1.0 - 2.0 * (ysqr + q.z * q.z);
	float yaw = std::atan2(t3, t4);
	return yaw;
    }

};

