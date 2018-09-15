#ifndef DRAW_H
#define DRAW_H

#include <vector>
#include <string>
#include <geometry_msgs/Pose.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/operations.hpp>
#include <graph.h>

class Draw {

    public:
	Draw(){};
	cv::Point2f pixelToPoint(int x, int y, float map_resolution, geometry_msgs::Pose map_origin);
	cv::Point2f pointToPixel(cv::Point2f pt, float map_resolution, geometry_msgs::Pose map_origin);
	void drawRotatedRectangle(cv::Mat& image, cv::Point centerPoint, cv::Size rectangleSize, double rotationDegrees, cv::Scalar color);
	void drawAllRobotOrientation(cv::Mat image, navNode nodesNav, cv::Size rectangleSize, double rotationDegrees, float mapRes, geometry_msgs::Pose mapOrig);
	void drawNode(cv::Mat inputMap, pGraph nlist, float mapRes, geometry_msgs::Pose mapOrig);
	void drawInfoNode(cv::Mat inputMap, pGraph nlist,int id, float mapRes, geometry_msgs::Pose mapOrig);
	void drawNavNode(cv::Mat inputMap, navNode nlist, float mapRes, geometry_msgs::Pose mapOrig);
	void drawFindNode(cv::Mat inputMap, pGraph nlist, float mapRes, geometry_msgs::Pose mapOrig, int id);
	void drawShortestPath(cv::Mat inputMap, pGraph nodelist, float mapRes, geometry_msgs::Pose mapOrig, std::vector<int> path);
};

#endif