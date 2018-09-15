#include "graph.h"
#include <draw.h>

std::string package_path = ros::package::getPath("spqr_topological_graph");  
std::string path_graph = package_path + "/graphs/graph.txt";

//################################TOPOLOGICAL GRAPH PART#######################################

void Graph::addNode(graphNode& newNode, std::vector<int> listId, int flag, std::string label){
	std::vector<int> temp;
	//case of other nodes	
	if(nodeList.size() > 0){
	   newNode.id = nodeList.size();
	   //if the node is of the location service
	   if(flag==1){
	   	newNode.label=label;
	   }
	   //else is an additional node
	   else{
		newNode.label=std::to_string(newNode.id);
	   }
	   for(int i=0; i < listId.size(); i++){
		int k=0;
		bool notFound=true;
	   	while(k<nodeList.size() && notFound){
		    //if the adjacency node exist on graph
		    if(nodeList.at(k).id==listId.at(i)){
			//update the adjacency list of that node
			nodeList.at(k).list_id.push_back(newNode.id);
			//update the weights list of both nodes
			float distance_x = pow (newNode.pos_x-nodeList.at(k).pos_x, 2.0);
			float distance_y = pow (newNode.pos_y-nodeList.at(k).pos_y, 2.0);
			float deltaPos = distance_x + distance_y;
			float weight = sqrt(deltaPos);
			newNode.distances.push_back(weight);
			nodeList.at(k).distances.push_back(weight);				
                        notFound=false;
		    }	
		    k++;
		}
		//if the adjacency node not exist on graph, remove the adjacency node on the process 
		if(notFound){
		    std::cout << "Error, No adjacency node exist with Id# " << listId.at(i) << std::endl; 		
		    temp.push_back(listId.at(i));
		}
	    }
	}
	//case of root node
	else{
	   newNode.id=0;
	    //if the node is of the location service
	   if(flag==1){
	   	newNode.label=label;
	   }
	   //else is an additional node
	   else{
		newNode.label=std::to_string(newNode.id);
	   }
	}
	//delete all the node that are not part of the graph
	for(int i=0;i<temp.size();i++){	
		listId.erase(std::remove(listId.begin(), listId.end(), temp.at(i)), listId.end());
	}
	//update the graph        
	newNode.list_id=listId;
	nodeList.push_back(newNode);	
}

void Graph::removeArc(graphNode curr_node, graphNode succ_node){
	std::vector<int> temp_adj;
	std::vector<float> temp_dist;
	std::vector<int> temp2_adj;
	std::vector<float> temp2_dist;
	temp_adj=curr_node.list_id;
	temp_dist=curr_node.distances;
	temp2_adj=succ_node.list_id;
	temp2_dist=succ_node.distances;
	
	//update the lists of both nodes
	int k=0;
	bool notFound=true;	
	while(k<temp_adj.size() && notFound){
	     //if the adjacency node exist on graph
	     if(temp_adj.at(k)==succ_node.id){
		   temp_adj.erase(std::remove(temp_adj.begin(), temp_adj.end(), succ_node.id), temp_adj.end());	
		   temp_dist.erase(std::remove(temp_dist.begin(), temp_dist.end(), temp_dist.at(k)), temp_dist.end());		
                   notFound=false;
	     }	
	     k++;
	}
	k=0;
	notFound=true;
	while(k<temp2_adj.size() && notFound){
	     //if the adjacency node exist on graph
	     if(temp2_adj.at(k)==curr_node.id){
		   temp2_adj.erase(std::remove(temp2_adj.begin(), temp2_adj.end(), curr_node.id), temp2_adj.end());	
		   temp2_dist.erase(std::remove(temp2_dist.begin(), temp2_dist.end(), temp2_dist.at(k)), temp2_dist.end());		
                   notFound=false;
	     }	
	     k++;
	}
	
	//update Graph
	k=0;
	notFound=true;	
	while(k<nodeList.size() && notFound){
	     //if the adjacency node exist on graph
	     if(nodeList.at(k).id==curr_node.id){
		   nodeList.at(k).list_id=temp_adj;	
		   nodeList.at(k).distances=temp_dist;				
                   notFound=false;
	     }	
	     k++;
	}
	k=0;
	notFound=true;	
	while(k<nodeList.size() && notFound){
	     //if the adjacency node exist on graph
	     if(nodeList.at(k).id==succ_node.id){
		   nodeList.at(k).list_id=temp2_adj;	
		   nodeList.at(k).distances=temp2_dist;							
                   notFound=false;
	     }	
	     k++;
	}
}

void Graph::removeNode(int id){
	if( id >= nodeList.size()){
		std::cout << "The node " << id << " is not in the graph!" << std::endl;
		return;
	}
	nodeList.erase(nodeList.begin() + id);
	for(int i = id; i< nodeList.size();i++){
		nodeList.at(i).id=nodeList.at(i).id-1;
		if(isdigit(nodeList.at(i).label[0])){
		  nodeList.at(i).label= std::to_string(nodeList.at(i).id);
		}
	}
	for(int i = 0; i< nodeList.size();i++){
		for (int j=0; j<nodeList.at(i).list_id.size(); j++){
			if(nodeList.at(i).list_id.at(j) == id){
				nodeList.at(i).list_id.erase(nodeList.at(i).list_id.begin() + j);
				nodeList.at(i).distances.erase(nodeList.at(i).distances.begin() + j);
				break;
			}
		}
		for (int j=0; j<nodeList.at(i).list_id.size(); j++){
			if(nodeList.at(i).list_id.at(j) > id)
				nodeList.at(i).list_id.at(j) -= 1;
		}
	}
}

void Graph::addArc(int curr,int succ){
	if(curr > nodeList.size() || succ > nodeList.size()){
	std::cout << "The node is not in the graph!" << std::endl;
	return;	
	}
	std::vector<int>::iterator it;
	it = find (nodeList.at(succ).list_id.begin(), nodeList.at(succ).list_id.end(),curr);
        if (it != nodeList.at(succ).list_id.end()){
            std::cout << "Element already connected in the graph!" << std::endl;
	    return;	
	}
  	else{	
		float distance_x = pow (nodeList.at(curr).pos_x-nodeList.at(succ).pos_x, 2.0);
		float distance_y = pow (nodeList.at(curr).pos_y-nodeList.at(succ).pos_y, 2.0);
		float deltaPos = distance_x + distance_y;
		float weight = sqrt(deltaPos);
		nodeList.at(curr).list_id.push_back(succ);
		nodeList.at(curr).distances.push_back(weight);
		nodeList.at(succ).list_id.push_back(curr);
		nodeList.at(succ).distances.push_back(weight);
    		std::cout << "The nodes are connected. " << weight << std::endl;
	}
}

int Graph::getIdFromLabel(std::string lab){
    int k=0;
    int res = -1;
    bool notFound = true;
    while(k < nodeList.size() && notFound){
	if(nodeList.at(k).label == lab){
	    res = nodeList.at(k).id;
	    notFound = false;
	}
	k++;
    }
    if(notFound)
	std::cout << "No node found with label: " << lab << std::endl;	
    return res;
}

std::string Graph::getLabelFromId(int id){
    int k=0;
    std::string res = "ERROR";
    bool notFound = true;
    while(k < nodeList.size() && notFound){
	if(nodeList.at(k).id == id){
	    res = nodeList.at(k).label;
	    notFound = false;
	}
	k++;
    }
    if(notFound)
	std::cout << "No node found with id: " << id << std::endl;	
    return res;
}


void Graph::exportGraph(){
	FILE *fd;
	//write mode file
	
	fd=fopen(path_graph.c_str(), "w");
	if( fd==NULL ) {
		perror("Error I/O");
		exit(1);
	}
	// save the graph on file
	for(int i=0; i < nodeList.size(); i++ ){
		fprintf(fd, "%d %s %f %f %d ", nodeList.at(i).id,nodeList.at(i).label.c_str(),nodeList.at(i).pos_x,nodeList.at(i).pos_y,nodeList.at(i).list_id.size());
		for(int j=0; j < nodeList.at(i).list_id.size(); j++ ){
			fprintf(fd, "%d ", nodeList.at(i).list_id.at(j));
		}
		for(int j=0; j < nodeList.at(i).list_id.size(); j++ ){
			fprintf(fd, "%f ", nodeList.at(i).distances.at(j));
		}
		fprintf(fd, "\n");
		// close the file
	}
	fclose(fd);
}

pGraph Graph::importGraph(){
	pGraph importedGraph;
	//open file graph
	std::ifstream Graph_file(path_graph);
	if (Graph_file){
		std::string line;
		while (getline (Graph_file,line)){
			Graph_file.ignore(0);
			graphNode importedNode;
			//split the string in vector of  (elements)
			std::vector<std::string> elements;
			std::string delimiter = " ";
			int pos = line.find(delimiter);
			std::string token;
			while (pos !=-1) {
				token = line.substr(0, pos);
				elements.push_back(token);
				line.erase(0, pos + delimiter.length());
				pos = line.find(delimiter);
			}
			elements.push_back(line);
			if(elements.size()>0){
				importedNode.id = atoi(elements.at(0).c_str());
				importedNode.label = elements.at(1).c_str();
				importedNode.pos_x = atof(elements.at(2).c_str());
				importedNode.pos_y = atof(elements.at(3).c_str());
				//if exists adjacency
				if(atoi(elements.at(4).c_str())>0){
					for(int i=5; i < 5 + atoi(elements.at(4).c_str()); i++ ){
						importedNode.list_id.push_back(atoi(elements.at(i).c_str()));
					}
					for(int i=5+atoi(elements.at(4).c_str()); i < elements.size(); i++ ){
						importedNode.distances.push_back(atof(elements.at(i).c_str()));
					}
				}
				importedGraph.push_back(importedNode);
			}
		}
		Graph_file.close();
        this->nodeList = importedGraph;
	}
	return importedGraph;
}

pGraph Graph::replanTopologicalGraph(pGraph actualGraph)
{
    pGraph resGraph;
    
    return resGraph;
}

//################################NAVIGATION GRAPH PART#######################################

bool Graph::checkNavigationConnectivity(navNode navNodeList){
    std::cout << "[STEP1]: check connectivity..." << std::endl; 
    bool flag = true;
    std::vector<int> listNode;
    for(int i=0; i < navNodeList.size(); i++){
	if(navNodeList.at(i).id_ListCost.size() > 0){
	    int count=0;
	    for(int k=0; k < navNodeList.at(i).id_ListCost.at(0).second.size();k++){
		if(navNodeList.at(i).id_ListCost.at(0).second.at(k)==1)
		    count++;
	    }
	    if(count == navNodeList.at(i).id_ListCost.at(0).second.size()){
		listNode.push_back(navNodeList.at(i).id);
	        flag = false;
	    }
	}
      
    }
    if(flag)
	std::cout << "[DONE]: navigation graph is connected" << std::endl;
    else{
	std::cout << "[ERROR] NAVIGATION GRAPH IS NOT CONNECTED. CHECK idNODE: ";
	for(int i=0; i < listNode.size();i++)
	    std::cout << listNode.at(i) << " | " ;
	std::cout << std::endl;
    }
	return flag;
}

navNode Graph::generateNavigationGraph(cv::Mat inputMap, pGraph navNodes, float mapRes, geometry_msgs::Pose mapOrig){
    navNode navNodesGraph;
    Draw d;
    cv::Mat GroundTruthmask =  inputMap == 0;
    cv::cvtColor(GroundTruthmask, GroundTruthmask, CV_BGR2GRAY);
    int erosion_size = 6;  
    cv::Mat element = getStructuringElement(cv::MORPH_CROSS,
    cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
    cv::Point(erosion_size, erosion_size) );
    dilate(GroundTruthmask,GroundTruthmask,element);
    for(int i=0; i<GroundTruthmask.rows; i++){
	  for(int j=0; j<GroundTruthmask.cols; j++){
 		  if(GroundTruthmask.at<uchar>(i,j)==0){
			  GroundTruthmask.at<uchar>(i,j)=255;
		  }
		  else{
			  GroundTruthmask.at<uchar>(i,j)=0;
		  }	
	  }
    }

    cv::Mat costMap=GroundTruthmask.clone();
    cv::Mat costMapInverted;
    cv::Mat temp;
    //filter the overall image 
    cv::bitwise_not ( costMap, costMapInverted );
    cv::Mat result;
    
    //draw the robot orientation for check
    cv::Point centerRectangle_pixel;
    cv::Size rectangleSize = cv::Size2f(25,12);
    double rotationDegrees;
    
    //evaluate all robot position and orientation on the map
    int theta;
    int discretizationAngle = 8;
    cv::Scalar color;
    
    cv::Mat defaultMap=GroundTruthmask.clone();
    
    for(int i=0; i < navNodes.size(); i++){
        //update the Navigation nodes
        navigationNode nodeTemp;
	nodeTemp.id = navNodes.at(i).id;
	nodeTemp.label = navNodes.at(i).label;
	nodeTemp.pos_x = navNodes.at(i).pos_x;
	nodeTemp.pos_y = navNodes.at(i).pos_y;
	nodeTemp.id_ListCost;
	connectivityCost idCostList;
	theta = 0;
	std::vector<float> weights;
	for (int j=0; j < discretizationAngle; j++){
	    temp=GroundTruthmask.clone();
	    //the coordinates about ROI center 
	    centerRectangle_pixel = d.pointToPixel(cv::Point2f(navNodes.at(i).pos_x, navNodes.at(i).pos_y), mapRes, mapOrig);
	    rotationDegrees = theta;
	    //filter the image
	    color = cv::Scalar(255.0, 255.0, 255.0); // white
	    d.drawRotatedRectangle(temp, centerRectangle_pixel, rectangleSize, rotationDegrees, color);
	    cv::Rect myROI(centerRectangle_pixel.x-rectangleSize.width, centerRectangle_pixel.y-rectangleSize.width,
			   40,40);//map in 430, 430-630: leggenda
	    temp = temp(myROI);
	    cv::Rect AllImg(0,0,defaultMap.rows,defaultMap.cols);
	    cv::Rect res = AllImg & myROI;
	    cv::Mat tempCostMap = costMapInverted;
	    tempCostMap = tempCostMap(myROI);
	    //check disparity between default ROI map and ROI map with robot inside 
	    cv::bitwise_and(tempCostMap,temp,result);
	    double min, max;
	    cv::Point min_loc, max_loc; 
	    cv::minMaxLoc(result, &min, &max);	
	  
	    //if the robot crash with the wall
	    if (max == 255){
		weights.push_back(1);
// 		std::cout << "1 ###########################" << std::endl; 
	    }
	    else{
		weights.push_back(0);
// 		std::cout << "0 ###########################" << std::endl; 
	    }
	    //cv::waitKey(0);
	    //update new orientation
	    theta += 45;
	}
	
	std::pair<int,std::vector<float> > id_singleCost;
	id_adjacency connectivity = navNodes.at(i).list_id;
	
	id_singleCost.second = weights;
	for(int k = 0; k < connectivity.size() ; k++){
	    id_singleCost.first = connectivity.at(k);
	    idCostList.push_back(id_singleCost);
	}
	nodeTemp.id_ListCost=idCostList;
	//update navGraph
	navNodesGraph.push_back(nodeTemp);
	
    }
    
    d.drawAllRobotOrientation(inputMap, navNodesGraph, rectangleSize, 45, mapRes, mapOrig);
   //  d.drawNavNode(inputMap, navNodesGraph, mapRes, mapOrig);
    cv::imshow("Admissible Orientation", inputMap);
    cv::waitKey(1);
    return navNodesGraph;
}