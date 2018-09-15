#ifndef GRAPH_H
#define GRAPH_H

#include <vector>
#include <stdlib.h>
#include <iostream>
#include <math.h>
#include <algorithm>
#include <list>
#include <fstream>
#include <string>
#include <ros/package.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/Pose.h>

//structure of the pddl planner graph
typedef std::vector <float> weights;
typedef std::vector <int> id_adjacency;

struct graphNode{
	int id;
	std::string label;
	float pos_x;
	float pos_y;
	id_adjacency list_id;
	weights distances;

	graphNode(){}
	graphNode(float x_, float y_){
	  pos_x = x_;
	  pos_y = y_;
	}
};
typedef struct std::vector<graphNode> pGraph;


//structure of the navigation graph
typedef std::vector <std::pair<int,std::vector<float> >> connectivityCost;

struct navigationNode{
	int id;
	std::string label;
	float pos_x;
	float pos_y;
	connectivityCost id_ListCost;
};
typedef struct std::vector<navigationNode> navNode;

class Graph {

    private: 
	pGraph nodeList;
	navNode navNodeList;

    public:
	Graph(){};
	pGraph getNodeList() { return nodeList; }
	graphNode getNode(int id) {return nodeList.at(id); }
	void addNode(graphNode& newNode, std::vector<int> listId, int flag, std::string label="");
	void removeNode(int id);
	void addArc(int curr,int succ);
	void removeArc(graphNode curr_node, graphNode succ_node);
	int getIdFromLabel(std::string lab);
	std::string getLabelFromId(int id);
	void exportGraph();
	pGraph importGraph();
	pGraph replanTopologicalGraph(pGraph actualGraph);
	
	navNode getNavNodeList() { return navNodeList; }
	navigationNode getNavNode(int id) {return navNodeList.at(id); }
	navNode generateNavigationGraph(cv::Mat inputMap,  pGraph topologicalNodes, float mapRes, geometry_msgs::Pose mapOrig);
	bool checkNavigationConnectivity(navNode navNodeList);
};

#endif
