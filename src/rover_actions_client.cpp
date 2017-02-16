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

geometry_msgs::Pose goal_pose;
bool new_goal = false;
void goal_cb(const geometry_msgs::PoseStamped::ConstPtr msg)
{
   goal_pose = msg->pose;
   ROS_INFO("New Goal received");
   new_goal = true;
}


int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_dtiveto");
  ros::NodeHandle n;
  ros::Subscriber g_sub = n.subscribe("/my_goal",1,goal_cb);

  actionlib::SimpleActionClient<rover_actions::DriveToAction> ac(ros::this_node::getNamespace()+"/DriveTo", true);
  rover_actions::DriveToGoal goal;

  ac.waitForServer();
  ROS_INFO("Actoin Server Found");
  ros::Rate r_(10);
  while(ros::ok())
  {
    if (new_goal)
    {
      goal.goal_pose = goal_pose;
      goal.goal_pose.position.z = -100.0;
      actionlib::SimpleClientGoalState ac_state =
         ac.sendGoalAndWait(goal,ros::Duration(20));
      ROS_WARN_STREAM("actoin state:   " << ac_state.toString());
      new_goal = false;
    }


    ros::spinOnce();
    r_.sleep();
  }
  return 0;
}
