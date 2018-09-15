#include "draw.h"


cv::Point2f Draw::pixelToPoint(int x, int y, float map_resolution, geometry_msgs::Pose map_origin)
{
    return cv::Point2f(x*map_resolution + map_origin.position.x, y*map_resolution + map_origin.position.y);
}

cv::Point2f Draw::pointToPixel(cv::Point2f pt, float map_resolution, geometry_msgs::Pose map_origin)
{
    return cv::Point2f((pt.x - map_origin.position.x)/map_resolution,(pt.y - map_origin.position.y)/map_resolution);
}

void Draw::drawRotatedRectangle(cv::Mat& image, cv::Point centerPoint, cv::Size rectangleSize, double rotationDegrees, cv::Scalar color)
{
    // Create the rotated rectangle
    cv::RotatedRect rotatedRectangle(centerPoint, rectangleSize, rotationDegrees);
    // We take the edges that OpenCV calculated for us
    cv::Point2f vertices2f[4];
    rotatedRectangle.points(vertices2f);
    // Convert them so we can use them in a fillConvexPoly
    cv::Point vertices[4];    
    for(int i = 0; i < 4; ++i){
        vertices[i] = vertices2f[i];
    }
    // Now we can fill the rotated rectangle with our specified color
    cv::fillConvexPoly(image, vertices, 4, color);
}

void Draw::drawAllRobotOrientation(cv::Mat image, navNode nodesNav,  cv::Size rectangleSize, double rotationDegrees, float mapRes, geometry_msgs::Pose mapOrig)
{
    cv::Point centerPoint_pixel;
    int weight;
    for(int i = 0; i < nodesNav.size(); i++){
        centerPoint_pixel = pointToPixel(cv::Point2f(nodesNav.at(i).pos_x,nodesNav.at(i).pos_y),mapRes,mapOrig);
	int theta = 0;
	if(nodesNav.at(i).id_ListCost.size()>0){
	    for(int k=0; k < nodesNav.at(i).id_ListCost.at(0).second.size();k++){
		weight =  nodesNav.at(i).id_ListCost.at(0).second.at(k);
		cv::Scalar color;
		if(weight == 1){
		    color = cv::Scalar(0, 0, 255); // red
		    // Create the rotated rectangle
		    cv::RotatedRect rotatedRectangle(centerPoint_pixel, rectangleSize, theta);
		    // We take the edges that OpenCV calculated for us
		    cv::Point2f vertices2f[4];
		    rotatedRectangle.points(vertices2f);
		    // Convert them so we can use them in a fillConvexPoly
		    cv::Point vertices[4];    
		    for(int i = 0; i < 4; ++i){
			vertices[i] = vertices2f[i];
		    }
		    // Now we can fill the rotated rectangle with our specified color
		    cv::fillConvexPoly(image, vertices, 4, color);
		}
		theta += rotationDegrees;	
	    }
	    theta = 0;
	    for(int k=0; k < nodesNav.at(i).id_ListCost.at(0).second.size();k++){
		weight =  nodesNav.at(i).id_ListCost.at(0).second.at(k);
		cv::Scalar color;
		if(weight == 0){
		    color = cv::Scalar(0, 255, 0); // green
		    // Create the rotated rectangle
		    cv::RotatedRect rotatedRectangle(centerPoint_pixel, rectangleSize, theta);
		    // We take the edges that OpenCV calculated for us
		    cv::Point2f vertices2f[4];
		    rotatedRectangle.points(vertices2f);
		    // Convert them so we can use them in a fillConvexPoly
		    cv::Point vertices[4];    
		    for(int i = 0; i < 4; ++i){
			vertices[i] = vertices2f[i];
		    }
		    // Now we can fill the rotated rectangle with our specified color
		    cv::fillConvexPoly(image, vertices, 4, color);	  
		}
		theta += rotationDegrees;	
	    }
	}
    }
}

void Draw::drawNode(cv::Mat inputMap, pGraph nodelist, float mapRes, geometry_msgs::Pose mapOrig){
    int radius = 0.5;
    for(int i = 0; i< nodelist.size(); i++)
    {
        //the list of adjacency nodes, given a node
	id_adjacency adjacencylist =  nodelist.at(i).list_id;
	for(int k=0; k < adjacencylist.size();k++)
	{
		cv::line(inputMap,  pointToPixel(cv::Point2f(nodelist.at(i).pos_x,nodelist.at(i).pos_y),mapRes,mapOrig) , 
			 pointToPixel(cv::Point2f(nodelist.at(adjacencylist.at(k)).pos_x, nodelist.at(adjacencylist.at(k)).pos_y),mapRes,mapOrig), 
			 cv::Scalar(0, 255, 0),0.5,8,0);		
        }
    }
    for(int i = 0; i< nodelist.size(); i++)
    {
        //the list of adjacency nodes, given a node
	if( isalpha(nodelist.at(i).label[0]))
	{
	    cv::circle(inputMap, pointToPixel(cv::Point2f(nodelist.at(i).pos_x,nodelist.at(i).pos_y),mapRes,mapOrig), radius, cv::Scalar(255, 0, 0), -1);	
	}
	else
	{
	    cv::circle(inputMap, pointToPixel(cv::Point2f(nodelist.at(i).pos_x,nodelist.at(i).pos_y),mapRes,mapOrig), radius, cv::Scalar(0, 0, 255), -1);	
	}
    }
    cv::rectangle(inputMap, cv::Point(0,0), cv::Point(inputMap.cols, 50),  cv::Scalar(0, 0, 128), -1);
    cv::putText(inputMap, "SPQR@Work Topological Plan", cv::Point(5, 35),CV_FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(200, 200, 200), 2, CV_AA);
    //resize(inputMap,inputMap,cv::Size(inputMap.cols*3,inputMap.rows*3));
    cv::imshow("Map", inputMap);
    //std::cout << std::endl;
}

void Draw::drawInfoNode(cv::Mat inputMap, pGraph nodelist, int id, float mapRes, geometry_msgs::Pose mapOrig)
{
    int radius = 0.5;
     std::string label_node = "";
    //the list of nodes in the graph
    for(int i = 0; i< nodelist.size(); i++)
    {
	for(int k=0; k < nodelist.at(i).list_id.size();k++)
	{   
	    //the list of adjacency nodes, given a node
	    id_adjacency adjacencylist =  nodelist.at(i).list_id;
	    //draw line
	    cv::line(inputMap,  pointToPixel(cv::Point2f(nodelist.at(i).pos_x, nodelist.at(i).pos_y),mapRes,mapOrig) , 
			    pointToPixel(cv::Point2f(nodelist.at(adjacencylist.at(k)).pos_x, nodelist.at(adjacencylist.at(k)).pos_y), mapRes, mapOrig), 
			    cv::Scalar(205, 205, 205),0.5,8,0);	
	    
	}
    }
    for(int i = 0; i< nodelist.size(); i++)
    {
	 
	if(nodelist.at(i).id != id)
	{
	    cv::circle(inputMap, pointToPixel(cv::Point2f(nodelist.at(i).pos_x, nodelist.at(i).pos_y),mapRes,mapOrig), radius, cv::Scalar(205, 205, 205), -1);	
	}
	else
	{
	    cv::circle(inputMap, pointToPixel(cv::Point2f(nodelist.at(i).pos_x, nodelist.at(i).pos_y),mapRes,mapOrig), radius + 1, cv::Scalar(0, 0, 255), -1);
	    std::string label;
	    label = nodelist.at(i).label;
	    label.append("(");
	    label.append(std::to_string(nodelist.at(i).id));
	    label.append(")");
	    cv::Point p1 = pointToPixel(cv::Point2f(nodelist.at(i).pos_x, nodelist.at(i).pos_y), mapRes, mapOrig);
	    label_node = label;
	    cv::putText(inputMap, label, p1, cv::FONT_HERSHEY_PLAIN, 0.8, CV_RGB(0,0,0), 1.0);
	    id_adjacency adj =  nodelist.at(i).list_id;
	    for(int kk=0; kk <  adj.size(); kk++ ){
	      cv::circle(inputMap, pointToPixel(cv::Point2f(nodelist.at(adj.at(kk)).pos_x, nodelist.at(adj.at(kk)).pos_y), mapRes, mapOrig), radius + 1, cv::Scalar(255, 255, 0), -1);
	    }

	    
	}
	
    }
    cv::rectangle(inputMap, cv::Point(0,0), cv::Point(inputMap.cols, 50),  cv::Scalar(0, 200, 0), -1);
    cv::putText(inputMap, label_node, cv::Point(5, 35),CV_FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(200, 200, 200), 2, CV_AA);
    cv::imshow("Map", inputMap);
}

void Draw::drawNavNode(cv::Mat inputMap, navNode nodelist, float mapRes, geometry_msgs::Pose mapOrig){
    int radius = 1;
    //the list of nodes in the graph
    for(int i = 0; i< nodelist.size(); i++)
    {
	cv::circle(inputMap, pointToPixel(cv::Point2f(nodelist.at(i).pos_x,nodelist.at(i).pos_y),mapRes,mapOrig), radius, cv::Scalar(0, 0, 0), -1);	
    }
    cv::imshow("COSTMAP", inputMap);
    //std::cout << std::endl;
}

void Draw::drawFindNode(cv::Mat inputMap, pGraph nodelist, float mapRes, geometry_msgs::Pose mapOrig, int id){
    bool found=false;
    std::string upperText;
    int radius = 0.5;
    for(int i = 0; i< nodelist.size(); i++)
    {
	if(nodelist.at(i).id != id)
	{
	    cv::circle(inputMap, pointToPixel(cv::Point2f(nodelist.at(i).pos_x, nodelist.at(i).pos_y),mapRes,mapOrig), radius, cv::Scalar(205, 205, 205), -1);	
	}
	else
	{
	    cv::circle(inputMap, pointToPixel(cv::Point2f(nodelist.at(i).pos_x, nodelist.at(i).pos_y),mapRes,mapOrig), radius + 1, cv::Scalar(0, 0, 255), -1);
	    std::string label;
	    label = nodelist.at(i).label;
	    label.append("(");
	    label.append(std::to_string(nodelist.at(i).id));
	    label.append(")");
	    cv::Point p1 = pointToPixel(cv::Point2f(nodelist.at(i).pos_x, nodelist.at(i).pos_y), mapRes, mapOrig);
	    cv::putText(inputMap, label, p1, cv::FONT_HERSHEY_PLAIN, 0.8, CV_RGB(0,0,0), 1.0);
	    found = true;
	}
	
    }
    if(found){
	cv::rectangle(inputMap, cv::Point(0,0), cv::Point(inputMap.cols, 50),  cv::Scalar(0, 255, 0), -1);
	upperText = "Node founded on Graph";
    }
    else{
	cv::rectangle(inputMap, cv::Point(0,0), cv::Point(inputMap.cols, 50),  cv::Scalar(0, 0, 255), -1);
	upperText = "Node not founded on Graph";
      
    }
    cv::putText(inputMap, upperText, cv::Point(5, 35),CV_FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(200, 200, 200), 2, CV_AA);
    cv::imshow("Map", inputMap);
    cv::waitKey(0);
}

void Draw::drawShortestPath(cv::Mat inputMap, pGraph nodelist, float mapRes, geometry_msgs::Pose mapOrig, std::vector<int> path){
    std::string upperText;
    int radius = 0.5;
    bool notFound;
    int j;
    for(int i = 0; i< nodelist.size(); i++)
    {	
	notFound = true;
	j=0;
	while(j < path.size() && notFound)
	{
	    if(nodelist.at(i).id == path[j])
		notFound=false;	
	    j++;
	}
	if(!notFound){
	    cv::circle(inputMap, pointToPixel(cv::Point2f(nodelist.at(i).pos_x, nodelist.at(i).pos_y),mapRes,mapOrig), radius + 1, cv::Scalar(0, 0, 255), -1);
	}
	else{
	    cv::circle(inputMap, pointToPixel(cv::Point2f(nodelist.at(i).pos_x, nodelist.at(i).pos_y),mapRes,mapOrig), radius, cv::Scalar(205, 205, 205), -1);	
	}
    }
    cv::rectangle(inputMap, cv::Point(0,0), cv::Point(inputMap.cols, 50),  cv::Scalar(0, 0, 255), -1);
    upperText = "Shortest Path";
    cv::putText(inputMap, upperText, cv::Point(5, 35),CV_FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(200, 200, 200), 2, CV_AA);
    cv::imshow("Map", inputMap);
    cv::waitKey(0);
}