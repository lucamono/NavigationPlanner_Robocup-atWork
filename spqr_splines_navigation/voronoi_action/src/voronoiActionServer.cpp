#include <actionlib/server/simple_action_server.h>
#include <voronoi_action/voronoiAction.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <list>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <laser_obstacle_detection.h>
#include <graph.h>
#include <astar.h>
#include <draw.h>
#include <cubic_spline.h>
#include <brezier_curve.h>
#include <chrono>
#include <ctime>

class voronoiAction
{
protected:

  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange error may occur.


  actionlib::SimpleActionServer<voronoi_action::voronoiAction> as_;
  std::string action_name_;
  // create messages that are used to published feedback/result
  voronoi_action::voronoiFeedback feedback_;
  voronoi_action::voronoiResult result_;
  ros::Subscriber amcl_pose_subscriber;

  // mutex
  boost::mutex pose_mtx;
  geometry_msgs::PoseWithCovarianceStampedConstPtr amclPose;

  LaserDetection laserDetectionOnline;
  Graph gra;
  Draw d;

  std::string amcl_pose_topic;
  std::string command_vel_master_controller_topic;
  // master cmd_vel command vel publisher
  geometry_msgs::Twist twist_update;
  ros::Publisher cmd_vel_master_controller_publisher;
  float stop_vel = 1.0;

  //####### controller components ########

  //feedforward part
  Eigen::Vector3f velocityBezier;

  //error feed-back part
  Eigen::Vector3f odom;
  Eigen::Vector3f positionBezier;
  float gain_tx = 2.0;
  float gain_ty = 2.0;
  float gain_theta = 2.5;
  Eigen::Matrix< float, 3, 3> diag_gain;
  Eigen::Vector3f velocityControllerWorld;
  Eigen::Vector3f velocityController;

public:

  voronoiAction(std::string name) :
    as_(nh_, name, boost::bind(&voronoiAction::executeCB, this, _1), false),
    action_name_(name)
    {
	as_.start();
	 //the topic of all movements from the state machine

	nh_.param("amcl_pose_topic", amcl_pose_topic, std::string("/amcl_pose"));
	amcl_pose_subscriber = nh_.subscribe<geometry_msgs::PoseWithCovarianceStampedConstPtr> (amcl_pose_topic, 1, &voronoiAction::amcl_pose_topic_Callback, this);

	nh_.param("command_vel_master_controller_topic", command_vel_master_controller_topic, std::string("/desired_cmd_vel"));
	cmd_vel_master_controller_publisher =  nh_.advertise<geometry_msgs::Twist>(command_vel_master_controller_topic, 1);
    }
  ~voronoiAction(void)
  {
  }

  float toEulerianAngle(geometry_msgs::Quaternion q){
	float ysqr = q.y * q.y;
	// yaw (z-axis rotation)
	float t3 = +2.0 * (q.w * q.z + q.x * q.y);
	float t4 = +1.0 - 2.0 * (ysqr + q.z * q.z);
	float yaw = std::atan2(t3, t4);
	return yaw;
    }
  
  Eigen::Matrix2d rot2D(double alpha){
    Eigen::Matrix2d out;
    out << cos(alpha), -sin(alpha), sin(alpha), cos(alpha);
    return out;
  }
  
  double boxPlus(double alpha1, double alpha2){
    Eigen::Matrix2d r1 = rot2D(alpha1);
    Eigen::Matrix2d r2 = rot2D(alpha2);
    Eigen::Matrix2d res = r1*r2;
    return std::atan2(res(1,0), res(1,1));
  }
  
  double boxMinus(double alpha1, double alpha2){
    Eigen::Matrix2d r1 = rot2D(alpha1);
    Eigen::Matrix2d r2 = rot2D(-alpha2);
    Eigen::Matrix2d res = r1*r2;
    return std::atan2(res(1,0), res(1,1));
  }

  void amcl_pose_topic_Callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr amclPose_in){
    pose_mtx.lock();
    amclPose = amclPose_in;
    pose_mtx.unlock();
  }

  void executeCB(const voronoi_action::voronoiGoalConstPtr &goal)
  {
    std::cout << "Action Server started" << std::endl;
    // helper variables
    feedback_.goal = 1.0;

    //init controller
    diag_gain << gain_tx, 0, 0, 0, gain_ty, 0, 0, 0, gain_theta;

    std::string startLabel =goal->start_goal[0];
    std::string goalLabel =goal->start_goal[1];
    std::cout << "START: " << startLabel << " | END: " << goalLabel << std::endl;
    //take the topological graph from file
    pGraph impGraph = gra.importGraph();
    //pass the structure graph into the draw simulator
    laserDetectionOnline.sim.g = gra;
    //take start and goal  and compute Astar
    int id_start = gra.getIdFromLabel(startLabel);
    int id_goal = gra.getIdFromLabel(goalLabel);

    pGraph nodeList = gra.getNodeList();
    //wv compute path from start to goal
    Astar myAstar(nodeList);
    std::vector<int> AStarPath = myAstar.compute(id_start,id_goal);
    if(AStarPath.size() == 0)
    {
	std::cout << "ERROR: ASTAR COMPUTATION FAILED! " << std::endl;
	as_.setAborted(result_);
    }
    else
    {
	  //print the shortest path
	  std::cout << "ASTAR Shortest path: " << std::endl;
	  for(int i = 0; i< AStarPath.size(); i++)
	  {
	      std::cout << AStarPath[i] <<" | ";
	  }
	  std::cout << std::endl;

	  //set AStarPath into drawSimulator
	  laserDetectionOnline.sim.AStarPath = AStarPath;

	  //init cubic-bezier vector points
	  std::vector<Vector2> points;
	  for(int i=0; i<AStarPath.size();i++){
	      graphNode node = gra.getNode(AStarPath[i]);
	      points.push_back(Vector2(node.pos_x, node.pos_y));
	  }

	  //generate Cubic Spline
	  Row2 vi,vf;
	  vi << 0.01, 0.0;
	  vf << 0.0, 0.0;
	  CubicSpline* cubic_spline = new CubicSpline(points,vi,vf);
	  cubic_spline->setTimeInterval(0.001);
	  cubic_spline->compute();

	  std::vector<Vector2> cubic_spline_pos = cubic_spline->getPositions();
	  std::vector<Vector2> cubic_spline_vel = cubic_spline->getVelocities();
	  std::vector<Vector2> cubic_spline_acc = cubic_spline->getAccelerations();

	  //generate Brezier Spline
	  Vector2 b_vi,b_vf;
	  b_vi = vi.transpose();
	  b_vf = vf.transpose();
	  //0.8 is the max vel (m/s) of youbot
	  float v_max_youbot = 0.4;
	  BrezierCurve* brezier = new BrezierCurve(points, cubic_spline_pos, cubic_spline_acc, b_vi, b_vf, v_max_youbot , 0.001);
	  brezier->compute();

	  //draw images by setting into drawSimulator
	  std::vector<Vector2> brezier_pos_draw = brezier->getPositions();
	  laserDetectionOnline.sim.cubicSplinePos = cubic_spline_pos;
	  laserDetectionOnline.sim.brezierPos = brezier_pos_draw;

	  std::vector<Vector2> brezier_velocities = brezier->getVelocities();


    std::vector<double> bezier_vel_norm = brezier->getVelocitiesNorm();
    /*for(int i=0; i< bezier_vel_norm.size();++i){
        std::cout << "velocity norm: " << bezier_vel_norm.at(i) << std::endl;
    }*/



    double max = brezier_velocities.at(0).norm();
	  for(int i=0; i< brezier_velocities.size();++i)
	  {
	      if(brezier_velocities.at(i).norm() > max) max = brezier_velocities.at(i).norm();
	  }

	  //time factor
	  double ratio_vel = v_max_youbot/max;

	  bool reachedGoal= false;
	  bool reachedRotation = false;
	  bool NoError=true;

	  float lastOrientation;

	  //start the clock for the timing law
	  std::chrono::time_point<std::chrono::system_clock> startTime, endTime;
	  startTime = std::chrono::system_clock::now();

	  //init incremental time temp1
	  int actual_time = 0;
	  int delta_time;
	  double ac_time = 0.0;
	  double dt_time;
	  double u_dot = v_max_youbot;
	  //std::cout<<"u_dot1: " << u_dot << std::endl;
	  double s = 0.0;
	  double s_1;
	  //std::cout << "PROVA MAX WILSON GAY" << std::endl;
	  double dt = 0.0;
	  
	  //giuseppe
	  double theta = 0.0;
	  Eigen::Matrix3d R_transpose = Eigen::MatrixXd::Zero(3,3);
      Eigen::Vector3d transVeloContrWorld;
      transVeloContrWorld = Eigen::VectorXd::Zero(3);
      Eigen::Vector3d robot_position;
      robot_position = Eigen::VectorXd::Zero(3);
      Eigen::Vector3d angular_velocity;
      angular_velocity = Eigen::VectorXd::Zero(3);
      R_transpose(2,2) = 1.0;
      
	  //



	  //insert in && the condition not empty about the list of traject inside first while

	  while(NoError){
	      //navigate on plan until goal reached or not obstacles found
	      while(!laserDetectionOnline.obstacleFounded && (!reachedGoal || !reachedRotation)){

			endTime = std::chrono::system_clock::now();
			//take current time for evaluate the timing law
			delta_time = std::chrono::duration_cast<std::chrono::milliseconds>
								(endTime-startTime).count();
			if(delta_time > actual_time)
			{
		
				dt_time= (double)delta_time/(1000);
				ac_time = (double)actual_time/(1000);
				dt = dt_time-ac_time;
				s += dt*u_dot;
		
				if(!reachedGoal)
					u_dot = brezier->getVelocityNormAtTime(s);
				if(u_dot > 5.0){
					u_dot = v_max_youbot;
				}
				s_1 = s + dt*u_dot;
			
				pose_mtx.lock();
				odom(0) = amclPose->pose.pose.position.x;
				odom(1) = amclPose->pose.pose.position.y;
				odom(2) = toEulerianAngle(amclPose->pose.pose.orientation);
				pose_mtx.unlock();
				//take velocity components from Bezier
				Vector2 brezier_vel = brezier->getVelocityAtTime(s);
				velocityBezier(0) = brezier_vel.x()*u_dot;
				velocityBezier(1) = brezier_vel.y()*u_dot;
				velocityBezier(2) = 0.0;

				//take position components from Bezier
				Vector2 brezier_pos = brezier->getPositionAtTime(s_1);
				positionBezier(0) = brezier_pos.x();
				positionBezier(1) = brezier_pos.y();
				//if goal reached during navigation
				float goalPosX = nodeList.at(id_goal).pos_x;
				float goalPosY = nodeList.at(id_goal).pos_y;
				float errorGoalX = goalPosX - odom(0);
				float errorGoalY = goalPosY - odom(1);
				float offsetPlus = 0.002;
				float offsetMinus = - 0.002;
				bool checkPositionToGoalX = (errorGoalX < offsetPlus && errorGoalX > offsetMinus);
				bool checkPositionToGoalY = (errorGoalY < offsetPlus && errorGoalY > offsetMinus);
				//if(checkPositionToGoalX && checkPositionToGoalY || (brezier->trajectoryFinished(s)))
				/*if((brezier->trajectoryFinished(s))) {
					std::cout << "ENTRATO NEL GOL" << std::endl;
					reachedGoal = true;
					std::cout << "entro? " << abs(positionBezier(2)-odom(2)) << std::endl;
					if(abs(positionBezier(2)-odom(2)) < 0.005) {
							reachedRotation = true;
							std::cout << "si!" << std::endl;
					} 
				}*/
				//else {
					//check safe Atan2
					float offsetAtanPlus = 0.1;
					float offsetAtanMinus = - 0.1;
					bool checkSafeAtan2GoalX = (velocityBezier(0) < offsetAtanPlus && velocityBezier(0) > offsetAtanMinus);
					bool checkSafeAtan2GoalY = (velocityBezier(1) < offsetAtanPlus && velocityBezier(1) > offsetAtanMinus);


					//positionBezier(2) = atan2((brezier_pos.y()-brezier->getPositionAtTime(s-10*dt*u_dot).y()),(brezier_pos.x()-brezier->getPositionAtTime(s-10*dt*u_dot).x()));
					positionBezier(2) =  brezier->getOrientationAtTime(s);
					// velocityController w.r.t. world
					velocityControllerWorld = velocityBezier + diag_gain*(positionBezier-odom);
					velocityControllerWorld(2) = gain_theta*boxMinus(positionBezier(2), odom(2));
					std::cout << "dovrei   " << positionBezier(2) << " ma   " << odom(2) << std::endl;
					transVeloContrWorld(0)=velocityControllerWorld(0);
					transVeloContrWorld(1)=velocityControllerWorld(1);
					
					//convert World frame controller to Robot frame controller
					float cos_r = cos(odom(2));
					float sin_r = sin(odom(2));
					R_transpose(0,0)=cos_r;
					R_transpose(0,1)=sin_r;
					R_transpose(1,0)=-sin_r;
					R_transpose(1,1)=cos_r;

					//giuseppe
					robot_position(0) = odom(0);
					robot_position(1) = odom(1);
					robot_position(0) = 0.0;
					angular_velocity(2) = (odom(2)-theta)/dt;
					transVeloContrWorld = angular_velocity.cross(R_transpose*robot_position)+(R_transpose*transVeloContrWorld);
					
					velocityController(0) = transVeloContrWorld(0);
					velocityController(1) = transVeloContrWorld(1);
					velocityController(2) = velocityControllerWorld(2);

					//controller publisher
					twist_update.linear.x = velocityController(0)*stop_vel;
					twist_update.linear.y = velocityController(1)*stop_vel;
					twist_update.angular.z = velocityController(2)*stop_vel;

                    if((brezier->trajectoryFinished(s))) {
						std::cout << "ENTRATO NEL GOL" << std::endl;
						reachedGoal = true;
						twist_update.linear.x = 0.0;
						twist_update.linear.y = 0.0;
						twist_update.angular.z = 2*(atan2(brezier_pos_draw.at(brezier_pos_draw.size() -1 ).y() - brezier_pos_draw.at(brezier_pos_draw.size()-5).y(),brezier_pos_draw.at(brezier_pos_draw.size() -1 ).x() - brezier_pos_draw.at(brezier_pos_draw.size()-5).x()) - odom(2));
						std::cout << "entro? " << abs(positionBezier(2)-odom(2)) << std::endl;
						if(abs(positionBezier(2)-odom(2)) < 0.05) {
								reachedRotation = true;
								std::cout << "si!" << std::endl;
								twist_update.angular.z = 0.0;
						} 
					}


					cmd_vel_master_controller_publisher.publish(twist_update);
					actual_time = delta_time;
					theta = odom(2);
		      //}
		    }
	      }

			std::cout << "fuori dal while" << std::endl;		  
	      //if the robot is in the goal
	      if(reachedGoal && reachedRotation)
	      {
			  //check_angle
			  
			twist_update.linear.x = 0.0;
			twist_update.linear.y = 0.0;
			twist_update.angular.z = 0.0;

			//cmd_vel_master_controller_publisher.publish(twist_update);
			result_.goal = feedback_.goal;
			as_.setSucceeded(result_);
		  	return;
	      }
	      //case of obstacle founded
	      else if(laserDetectionOnline.obstacleFounded)
	      {
		  //STOP the robot with cmd_vel=0
		  stop_vel = 0.0;
		  //controller publisher
		  twist_update.linear.x = velocityController(0)*stop_vel;
		  twist_update.linear.y = velocityController(1)*stop_vel;
		  twist_update.angular.z = velocityController(2)*stop_vel;

		  cmd_vel_master_controller_publisher.publish(twist_update);

		  //new obstacle is founded, print list of obstacles
		  std::vector<std::pair<int,int> > obstaclesInMap = this->laserDetectionOnline.objectsDetectedPixel;
		  for(int j = 0; j < obstaclesInMap.size(); ++j)
		  {
		      std::cout << obstaclesInMap.at(j).first << " | " << obstaclesInMap.at(j).second << std::endl;
		  }
		  std::cout << std::endl << std::endl;
		  std::cout << "UPDATE TOPOLOGICAL GRAPH!" << std::endl;

		  //update the navigation graph with the obstacle inside
		  impGraph = gra.replanTopologicalGraph(impGraph);

		  //reset laser detection module
		  laserDetectionOnline.obstacleFounded = false;
		  stop_vel = 1.0;

		  //recompute ASTAR navigation
		  std::cout << "ASTAR RECOMPUTATION!" << std::endl;
	/*	  AStar *grep = new AStar(gra.getNodeList().size());
		  AStarPath = grep->computeShortestPath(gra.getNodeList(), AStarStart, AStarGoal);
		  if(AStarPath.size() == 0)
		  {
		      std::cout << "ERROR: ASTAR COMPUTATION FAILED! " << std::endl;
		      as_.setAborted(result_);
		      NoError = false;
		  }
	*/
	      }
	  }
      }
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "voronoi");
  voronoiAction navigation(ros::this_node::getName());
  ros::Rate r(100);
  while(1){
    r.sleep();
    ros::spinOnce();
  }
  return 0;
}
