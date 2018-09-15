#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <voronoi_action/voronoiAction.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_voronoi_action_server");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<voronoi_action::voronoiAction> ac("voronoi", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action server
  voronoi_action::voronoiGoal goal;
  goal.start_goal[0] = "WPS";
  //last goal WS03
  goal.start_goal[1] = "WP04";
  ac.sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(180.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
    ac.cancelAllGoals();
  }
  else
    ROS_INFO("Action did not finish before the time out.");
  ac.cancelAllGoals();
  //exit
  return 0;
}
