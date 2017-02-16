#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <rover_actions/DriveToOAAction.h>
geometry_msgs::PoseStamped goal_pose;
bool new_goal = false;
void goal_cb(const geometry_msgs::PoseStamped::ConstPtr msg)
{
  goal_pose = *msg;
  ROS_INFO("Goal received Going there :) Hope to not Fuck up :D!");
  new_goal = true;
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_DriveToOA");
  ros::NodeHandle nh;
  ros::Subscriber goal_sub = nh.subscribe("/my_goal",1,goal_cb);
  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<rover_actions::DriveToOAAction> ac("DriveToOA", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started");
  // send a goal to the action
  rover_actions::DriveToOAGoal goal;
  ros::Rate r(10);
  while(ros::ok())
  {
    if (new_goal) {
      new_goal = false;
      goal.goal_pose = goal_pose.pose;
      ROS_WARN("SENDING GOAL");
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
    }
    r.sleep();
    ros::spinOnce();
  }

}
