#include <laser_obstacle_detection.h>

int frameCounter=0;
int k=0;


void LaserDetection::callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& amclPose, const sensor_msgs::LaserScanConstPtr& base_scan){	
       //draw AStarPath Image of the simulator
       this->sim.drawAStarPath(this->user.mapAStarAS, this->sim.g.getNodeList(), this->user.mapResolution , this->user.mapOrigin, this->sim.AStarPath);
       //draw cubic and bezier splines
       this->sim.drawCubicSplinePath(this->user.mapCubicAS, this->sim.cubicSplinePos, this->user.mapResolution , this->user.mapOrigin);
       this->sim.drawBrezierCurvePath(this->user.mapBezierAS, this->sim.brezierPos, this->user.mapResolution , this->user.mapOrigin);
       this->sim.drawReplanGraph(this->user.mapReplanTopolGraphAS, this->user.mapResolution, this->user.mapOrigin);
       
       
       this->user.mapDraw=this->user.map.clone();
       clock_t start = clock();
       //update frame for check obstacles
       frameCounter++;
       //show the groundtruth mask
       //cv::imshow("GroundTruthMask", this->user.mapMask);
       cv::waitKey(10);

       //init to zero the obstacle_mask
       cv::Mat laserScanMap;
       laserScanMap=this->user.laserScanMap;
       
       this->poseAMCLx = amclPose->pose.pose.position.x;
       this->poseAMCLy = amclPose->pose.pose.position.y;
       //convert from quaternion to euleur
       this->poseAMCLa = toEulerianAngle(amclPose->pose.pose.orientation);  
       
       //compute laser scans
       this->angle_min=base_scan->angle_min;
       this->angle_increment=base_scan->angle_increment;
       this->ranges=base_scan->ranges;
       this->intensities=base_scan->intensities;
       this->range_min=base_scan->range_min;
       this->range_max=base_scan->range_max;     
       float phi = this->angle_min;
       for(int i=0;i<this->ranges.size();i++){
	  if(this->ranges.at(i) > this->range_min && this->ranges.at(i)<this->range_max)
	  {	 
		 Eigen::Vector2f posScan; 
	 	 posScan(0)=this->ranges.at(i)*cos(phi);
	 	 posScan(1)=this->ranges.at(i)*sin(phi);
		 Eigen::Vector2f poseRobot;
		 poseRobot(0)=this->poseAMCLx;
		 poseRobot(1)=this->poseAMCLy;
		 //trasform the scan point from robot reference frame to world reference frame
		 Eigen::MatrixXf R(2,2);
	 	 float cos_r = cos(this->poseAMCLa);
	 	 float sin_r = sin(this->poseAMCLa);
		 R(0,0)=cos_r;
		 R(0,1)=-sin_r;
		 R(1,0)=sin_r;
		 R(1,1)=cos_r;
	 	 Eigen::Vector2f poseScanWorld;
		 poseScanWorld = R*posScan + poseRobot;  
	    	 //convert points in pixels
	         cv::Point2f posScanWorld;
		 posScanWorld.x=poseScanWorld(0);
		 posScanWorld.y=poseScanWorld(1);
	 	 cv::Point2f pixelObstacle=this->sim.pointToPixel(posScanWorld, this->user.mapResolution, this->user.mapOrigin);
	 	 k+=5;
	 	 //update the obstacle mask if( this->user.obstacleMask.at<uchar>(this->user.map.rows-pixelObstacle.y,pixelObstacle.x)<=100)
	 	 if(this->user.laserScanMap.at<uchar>(this->user.laserScanMap.rows-pixelObstacle.y,pixelObstacle.x)<=100)
	      		 this->user.laserScanMap.at<uchar>(this->user.laserScanMap.rows-pixelObstacle.y,pixelObstacle.x)+=k;
	      	  }
	      phi+=this->angle_increment;
	}
       //Draw tools       
       std::vector <float> robot_state;
       robot_state.push_back(poseAMCLx);
       robot_state.push_back(poseAMCLy);
       robot_state.push_back(poseAMCLa);
       
       std::vector <std::pair<float,float> > laserScan;
       this->sim.drawRobot(this->user.mapDraw, this->user.mapResolution, this->user.mapOrigin,robot_state,laserScan);
       
       //show the groundtruth mask
       //cv::imshow("GroundTruthMask", this->user.mapMask);
       this->user.laserScanMap.setTo(0, this->user.laserScanMap == 1);
       this->user.laserScanMap.setTo(k-(k-1), this->user.laserScanMap > 1);
    
       //show the laserScan mask
       //cv::imshow("LaserScan", this->user.laserScanMap);
       cv::waitKey(10);
       
       //init covariance Matrix
       Eigen::MatrixXd cov(6,6);
       
       //take tha covariance matrix from amclPose topic
       boost::array<double, 36ul> m = amclPose->pose.covariance;
       cov(0,0) = m[0]; cov(0,1) = m[1]; cov(0,2) = m[2]; cov(0,3) = m[3]; cov(0,4) = m[4]; cov(0,5) = m[5];
       cov(1,0) = m[6]; cov(1,1) = m[7]; cov(1,2) = m[8]; cov(1,3) = m[9]; cov(1,4) = m[10]; cov(1,5) = m[11];
       cov(2,0) = m[12]; cov(2,1) = m[13]; cov(2,2) = m[14]; cov(2,3) = m[15]; cov(2,4) = m[16]; cov(2,5) = m[17];
       cov(3,0) = m[18]; cov(3,1) = m[19]; cov(3,2) = m[20]; cov(3,3) = m[21]; cov(3,4) = m[22]; cov(3,5) = m[23];
       cov(4,0) = m[24]; cov(4,1) = m[25]; cov(4,2) = m[26]; cov(4,3) = m[27]; cov(4,4) = m[28]; cov(4,5) = m[29]; 
       cov(5,0) = m[30]; cov(5,1) = m[31]; cov(5,2) = m[32]; cov(5,3) = m[33]; cov(5,4) = m[34]; cov(5,5) = m[35]; 
       
       //compute the eigen values of the covariance matrix
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver(cov);
	if (eigensolver.info() != Eigen::Success) abort();
	Eigen::MatrixXd eigenValuesCovariance(6,1);
	eigenValuesCovariance = eigensolver.eigenvalues();
	 /*   cout << "The eigenvalues of A are:\n" << eigenValuesCovariance << endl;
	      cout << "Here's a matrix whose columns are eigenvectors of A \n"
		  << "corresponding to these eigenvalues:\n"
		  << eigensolver.eigenvectors() << endl;  */
       //starting detection of objects
       if(frameCounter==10){
           frameCounter=0;
	   if(eigenValuesCovariance(3,0) < 0.01 && eigenValuesCovariance(4,0) < 0.01 && eigenValuesCovariance(5,0) < 0.008){
	      //filtering the obstacle mask
	      this->user.obstacleMask=this->user.laserScanMap>0;
	      cv::bitwise_and(this->user.mapMask,this->user.obstacleMask,this->user.obstacleMask);
	     // cv::imshow("ObstacleMask", this->user.obstacleMask); 
	      cv::Mat filteredObs;
	      cv::boxFilter(this->user.obstacleMask,filteredObs,-1,cv::Size(3,3));
	      double min, max;
	      cv::Point min_loc, max_loc; 
	      cv::minMaxLoc(filteredObs, &min, &max);		
	      if(abs(min-max)>80){
		    //NEW THRESHOLD ABOUT THE RANGE OF DETECTION
		    if(max > 0){ 
		    cv::minMaxLoc(filteredObs, &min, &max, &min_loc, &max_loc);
		    //std::cout << max_loc << std::endl;
		    //save the pixel about the obstacle
		    cv::Point posOnImage;
		    posOnImage.x=max_loc.x;
		    posOnImage.y=max_loc.y;
		    pair<float,float> pixelObs;
		    pixelObs.first=posOnImage.x;
		    pixelObs.second=posOnImage.y;
		    boundingBox obstacleShape;
		    //check if the obstacle is new observation or not
		    int offsetX = 30;
		    int offsetY = 30;
		    int listObstacleX, listObstacleY;
		    bool insideXRange, insideYRange;
		    bool checkNewObstacle = true;
		    if(objectsDetectedPixel.size()>0)
		    {
			for(int i=0; i < objectsDetectedPixel.size(); ++i)
			{
			    listObstacleX = objectsDetectedPixel.at(i).first;
			    listObstacleY = objectsDetectedPixel.at(i).second;
			    insideXRange = pixelObs.first < (listObstacleX + offsetX) && pixelObs.first > (listObstacleX - offsetX);
			    insideYRange = pixelObs.second < (listObstacleY + offsetY) && pixelObs.second > (listObstacleY - offsetY);
			    //if the observation is not a new obstacle
			    if((insideXRange && insideYRange))
			    {
				 checkNewObstacle = false;
			    }
			}
			//if the observation is a new obstacle
			if(checkNewObstacle)
			{
			    //update list of max_loc obstacle
			    objectsDetectedPixel.push_back(pixelObs);
			    //update list of BBox obstacle shape
			    obstacleShape = getObstacleBBox(this->user.obstacleMask, max_loc, this->user.mapResolution, this->user.mapOrigin,robot_state);
			    obstacles.push_back(obstacleShape);
			    obstacleFounded = true;
			}
		    }
		    //case of first observation, add to vector
		    else{
			//update list of max_loc obstacle
			objectsDetectedPixel.push_back(pixelObs);
			//update list of BBox obstacle shape
			obstacleShape = getObstacleBBox(this->user.obstacleMask, max_loc, this->user.mapResolution, this->user.mapOrigin,robot_state);
			obstacles.push_back(obstacleShape); 
			obstacleFounded = true;
		    }
		    //convert pixel obstacle to world position obstacle
		    cv::Point posOnMap = sim.pixelToPoint(max_loc.x, max_loc.y, user.mapResolution, user.mapOrigin);
		    pair<float,float> pointObs;
		    pointObs.first=posOnMap.x;
		    pointObs.second=posOnMap.y;
		    objectsDetectedPos.push_back(pointObs);
		    }
		}
		//cv::imshow("filteredObs", filteredObs);
	  }
       k=0;  
       }
       cv::Vec3b pixelColor(0,0,255);
       user.mapDraw.setTo(pixelColor, this->user.obstacleMask == 255);
       cv::imshow("Map_visualizer", user.mapDraw);
       cv::waitKey(10);
       clock_t end = clock();
       double time = (double) (end-start) / CLOCKS_PER_SEC * 1000.0;
       //std::cout << time << std::endl;
  }
  
void LaserDetection::initEnvironment(int erosion_size)
{   
    this->sim.loadMapFromMapServer();
    this->user=this->sim.getUserdata();
    cv::namedWindow("Map_visualizer");
    cv::imshow("Map_visualizer", this->user.map);
    cv::waitKey(10);
    this->user.defaultmap = this->user.map.clone();
    this->user.mapAStarAS = this->user.map.clone();
    this->user.mapCubicAS = this->user.map.clone();
    this->user.mapBezierAS = this->user.map.clone();
    this->user.mapReplanTopolGraphAS = this->user.map.clone();
    //init groundtruth mask
    cv::Mat GroundTruthmask =  this->user.map == 0;
    cv::cvtColor(GroundTruthmask, GroundTruthmask, CV_BGR2GRAY); 
    cv::Mat element = getStructuringElement(cv::MORPH_ELLIPSE,
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
    cv::namedWindow("AStarPath");
    
    //cv::namedWindow("GroundTruthMask");
    this->user.mapMask=GroundTruthmask;
    //init laserScan mask
    this->user.mapMask.copyTo(this->user.laserScanMap);
    this->user.laserScanMap.setTo(cv::Scalar(0));
 //   cv::namedWindow("LaserScan");
    //init obstacle mask
    this->user.mapMask.copyTo(this->user.obstacleMask);
    this->user.obstacleMask.setTo(cv::Scalar(0));
 //   cv::namedWindow("ObstacleMask");
}  

boundingBox LaserDetection::getObstacleBBox(cv::Mat obstacleMap, cv::Point max_loc, float mapRes, geometry_msgs::Pose mapOrig, std::vector <float> robot_state)
{
    boundingBox result;
    int rowROI, ColROI;
    int offsetSearch=20;
    rowROI =  2*offsetSearch; 
    ColROI =  2*offsetSearch;
    std::vector<cv::Point> pointsObstacle;
    cv::Point temp;
    for(int rows=0; rows < rowROI; ++rows)
    {
	for(int cols=0; cols < ColROI; ++cols)
	{
	    temp.x = (max_loc.x - offsetSearch) + cols;
	    temp.y = (max_loc.y - offsetSearch) + rows;
	    if(obstacleMap.at<uchar>(temp) == 255){
		pointsObstacle.push_back(temp);
	    }
	}
    }
    int minX, maxX, minY, maxY;
    Simulator sim;
    cv::Point2f robotPos=sim.pointToPixel(cv::Point2f(robot_state.at(0),robot_state.at(1)),mapRes,mapOrig);
    for(int i = 0; i < pointsObstacle.size(); ++i)
    {
	//std::cout << "LISTA SHAPE POINTS: " << pointsObstacle.at(i).x << " , " << pointsObstacle.at(i).y << std::endl;
    }
    return result;
}