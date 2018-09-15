#include <draw_simulator.h>

std::vector<std::string> Simulator::explode(const std::string& str, const char& ch) {
  std::string next;
  std::vector<std::string> result;
  // For each character in the string
  for (std::string::const_iterator it = str.begin(); it != str.end(); it++) {
    // If we've hit the terminal character
    if (*it == ch) {
      // If we have some characters accumulated
      if (!next.empty()) {
	// Add them to the result vector
	result.push_back(next);
	next.clear();
      }
    } else {
      // Accumulate the next character into the sequence
      next += *it;
    }
  }
  if (!next.empty())
    result.push_back(next);
  return result;
}

std::pair<std::string, std::string> Simulator::getMapImgInfos(std::string config_file_path){
  YAML::Node map_config_file = YAML::LoadFile(config_file_path);
  std::string map_file_name = map_config_file["image"].as<std::string>();
  std::vector<std::string> map_file_path_splitted = explode(config_file_path, '/');
  std::string map_img_file_path = "/";
  for(int i=0; i<map_file_path_splitted.size()-1; i++){
    map_img_file_path += map_file_path_splitted[i] + "/";
  }
  map_img_file_path += map_file_name;
  return std::pair<std::string, std::string>(map_img_file_path, map_config_file["resolution"].as<std::string>());
}

cv::Mat Simulator::manuallyConvertMapToImg(const nav_msgs::OccupancyGrid& map, int &map_height, int &map_width){
  cv::Mat mat;
  mat.create(map_height, map_width, CV_8SC1);
  int rows = mat.rows;
  int cols = mat.cols;
  if (mat.isContinuous()) {
      cols = rows*cols;
      rows = 1;
  } 

  for (int r = 0; r < rows; ++r)
  {
      uchar *pOutput = mat.ptr<uchar>(r);

      for (int c = 0; c < cols; ++c)
      {
	  *pOutput = (uchar)map.data.at(c);
	  ++pOutput;
      }
  }
  return mat;
}

void Simulator::rebuildMapFromTresholds(cv::Mat &inputMap, cv::Mat &outputMap, float map_occupied_threshold, float map_free_threshold){
  cv::Vec3b not_visited_color(205, 205, 205);
  cv::Vec3b free_cell_color(254, 254, 254);
  cv::Vec3b occupied_cell_color(0, 0, 0); 
  int _map_occupied_threshold = round(map_occupied_threshold);
  int _map_free_threshold = round(map_free_threshold);
  //change the outputMap
  for(int r=0; r<inputMap.rows; r++){
    for(int c=0; c<inputMap.cols; c++){
      int pixel_intensity = (int)inputMap.at<signed char>(r,c);
      if(pixel_intensity == -1){
	outputMap.at<cv::Vec3b>(r,c) = not_visited_color;
      }
      else{
	if(pixel_intensity > _map_occupied_threshold){
	  //std::cout<<r<<" , "<<c<<std::endl;
	  outputMap.at<cv::Vec3b>(r,c) = occupied_cell_color;
	}
	else if(pixel_intensity < _map_free_threshold){
	  outputMap.at<cv::Vec3b>(r,c) = free_cell_color;
	}
      }
    }
  }
}

void Simulator::loadMapFromMapServer(){
// load the map
  cv::Mat map_img;
  geometry_msgs::Pose map_origin;
  float map_resolution;
  float map_occupied_threshold = 65.0;
  float map_free_threshold = 19.6;
  int map_height;
  int map_width;
  nav_msgs::OccupancyGrid map;
  ros::ServiceClient map_service_client_ = this->nh.serviceClient<nav_msgs::GetMap>("/static_map");
  nav_msgs::GetMap srv_map;
  if (map_service_client_.call(srv_map)){     ROS_INFO("Map service called successfully");
      map = nav_msgs::OccupancyGrid(srv_map.response.map);
      map_resolution = map.info.resolution;
      map_height = map.info.height;
      map_width = map.info.width;
      map_origin = map.info.origin;
      std::cout << map_width << std::endl;
      cv::Mat map_img_tmp = cv::Mat(cv::Size(map_width, map_height), CV_8SC1, (signed char*)map.data.data());
      map_img = cv::Mat(cv::Size(map_width, map_height), CV_8UC3);
      rebuildMapFromTresholds(map_img_tmp, map_img, map_occupied_threshold, map_free_threshold);
      cv::flip(map_img, map_img, 0);
      this->user.map = map_img;
      this->user.mapResolution = map_resolution;
      this->user.mapOrigin = map_origin;
    }
    else
    {
      ROS_ERROR("Failed to call map service");
    }
}

cv::Point2f Simulator::pixelToPoint(int x, int y, float map_resolution, geometry_msgs::Pose map_origin){
  return cv::Point2f(x*map_resolution + map_origin.position.x, y*map_resolution + map_origin.position.y);
}

cv::Point2f Simulator::pointToPixel(cv::Point2f pt, float map_resolution, geometry_msgs::Pose map_origin){
  return cv::Point2f((pt.x - map_origin.position.x)/map_resolution,(pt.y - map_origin.position.y)/map_resolution);
}

void Simulator::drawRobot(cv::Mat inputMap, float mapRes, geometry_msgs::Pose mapOrig, std::vector <float> robot_state, std::vector <std::pair<float,float> > laser){
/*	//draw laser rays
	for(int i=0; i<laser.size(); i++){
		line(inputMap,  pointToPixel(cv::Point2f(robot_state.at(0),robot_state.at(1)),mapRes,mapOrig) , 
			        pointToPixel(cv::Point2f(laser.at(i).first,laser.at(i).second), mapRes,mapOrig),cv::Scalar(0, 0, 255),1,8,0); 		
    
	}
*/
	cv::Point2f pix=pointToPixel(cv::Point2f(robot_state.at(0),robot_state.at(1)),mapRes,mapOrig);
	pix.y=inputMap.rows-pix.y;
 	//draw chassis of robot
 	circle(inputMap,pix, 4, cv::Scalar(255, 0, 0), -1);	
	
	cv::Point2f pix2=pointToPixel(cv::Point2f(robot_state.at(0)+ (1.5*cos(robot_state.at(2))),robot_state.at(1)+(1.5*sin(robot_state.at(2)))),mapRes,mapOrig);
	pix2.y=inputMap.rows-pix2.y;
 	//draw orientation of robot
 	line(inputMap,  pix , pix2,cv::Scalar(0, 255, 0),1,8,0); 		
    
  //  cv::imshow("Map_visualizer", inputMap);
  //  cv::waitKey(10);
 }
 
 void Simulator::drawAStarPath(cv::Mat inputMap, pGraph nodelist, float mapRes, geometry_msgs::Pose mapOrig, std::vector<int> path){
    //flip according to map server
    cv::flip(inputMap, inputMap, 0);
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
    //flip according simulator
    cv::flip(inputMap, inputMap, 0);
    cv::rectangle(inputMap, cv::Point(0,0), cv::Point(inputMap.cols, 50),  cv::Scalar(0, 0, 255), -1);
    upperText = "Shortest Path";
    cv::putText(inputMap, upperText, cv::Point(5, 35),CV_FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(200, 200, 200), 2, CV_AA);
    cv::imshow("AStarPath", inputMap);
}

void Simulator::drawBrezierCurvePath(cv::Mat inputMap, const std::vector<Vector2>& complete_path, float mapRes, geometry_msgs::Pose mapOrig)
{
    cv::Scalar black(0,0,0);
    std::string upperText;
    //flip according to map server
    cv::flip(inputMap, inputMap, 0);
    for(int i=0; i < complete_path.size(); i++)
    {
      cv::Point2f p = pointToPixel(cv::Point2f(complete_path[i].x(),complete_path[i].y()), mapRes, mapOrig);
      inputMap.at<cv::Vec3b>(p)[0] = black[0];
      inputMap.at<cv::Vec3b>(p)[1] = black[1];
      inputMap.at<cv::Vec3b>(p)[2] = black[2];
    }
    //flip according simulator
    cv::flip(inputMap, inputMap, 0);
    cv::rectangle(inputMap, cv::Point(0,0), cv::Point(inputMap.cols, 50),  cv::Scalar(255, 0, 0), -1);
    upperText = "Bezier curve";
    cv::putText(inputMap, upperText, cv::Point(5, 35),CV_FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(200, 200, 200), 2, CV_AA);
    cv::imshow("brezier curve", inputMap);
}

void Simulator::drawCubicSplinePath(cv::Mat inputMap, const std::vector<Vector2>& complete_path, float mapRes, geometry_msgs::Pose mapOrig)
{  
    cv::Scalar red(0,0,255);
    std::string upperText;
    //flip according to map server
    cv::flip(inputMap, inputMap, 0);
    for(int i=0; i < complete_path.size(); i++){
      cv::Point2f p = pointToPixel(cv::Point2f(complete_path[i].x(),complete_path[i].y()), mapRes, mapOrig);
      inputMap.at<cv::Vec3b>(p)[0] = red[0];
      inputMap.at<cv::Vec3b>(p)[1] = red[1];
      inputMap.at<cv::Vec3b>(p)[2] = red[2];
    }
    //flip according simulator
    cv::flip(inputMap, inputMap, 0);
    cv::rectangle(inputMap, cv::Point(0,0), cv::Point(inputMap.cols, 50),  cv::Scalar(0, 255, 0), -1);
    upperText = "Cubic spline";
    cv::putText(inputMap, upperText, cv::Point(5, 35),CV_FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(200, 200, 200), 2, CV_AA);
    cv::imshow("cubic spline", inputMap);
}

void Simulator::drawReplanGraph(cv::Mat inputMap, float mapRes, geometry_msgs::Pose mapOrig)
{
  
}

