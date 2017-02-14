#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <rover_actions/DriveToAction.h>
#include <rover_actions/PathFollowerAction.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Transform.h>
#include <tf/transform_datatypes.h>
#include <pcl/point_cloud.h>
#include <stdlib.h>
using namespace Eigen;

int main (int argc, char **argv)
{
  ros::init(argc, argv, "action_test_simple");
  if(argc != 4)
  {
    ROS_INFO("Too few arguments, please provid X, Y and Yaw(Deg)");
    return 0;
  }

  //ROS_INFO_STREAM("argc   " << argc <<"      argv   " << argv[1]);

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<rover_actions::DriveToAction> ac(ros::this_node::getNamespace()+"/DriveTo", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  rover_actions::DriveToGoal goal;
  goal.goal_pose.position.x = atof(argv[1]);
  goal.goal_pose.position.y = atof(argv[2]);
  goal.goal_pose.position.z = -100.0;
  goal.goal_pose.orientation = tf::createQuaternionMsgFromYaw(atof(argv[3])*3.1415/180);


  ac.sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  //exit
  return 0;
}
