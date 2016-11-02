#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <rover_actions/DriveToAction.h>
#include <rover_actions/PathFollowerAction.h>
#include <nav_msgs/Path.h>


nav_msgs::Path path;
bool start_path = false;
void path_cb(const nav_msgs::Path::ConstPtr& msg)
{
    path = *msg;
    start_path = true;
}

float poses_dist(geometry_msgs::Pose P1,geometry_msgs::Pose P2)
{
    float distant;
    double dx = P1.position.x - P2.position.x;
    double dy = P1.position.y - P2.position.y;
    distant = sqrt(pow(dx,2)+pow(dy,2));
    return distant;
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_path");
  ros::NodeHandle n;

  //actionlib::SimpleActionClient<rover_actions::PathFollowerAction> ac("PathFollower", true);
  actionlib::SimpleActionClient<rover_actions::DriveToAction> ac("DriveTo", true);
  // create the action client
  // true causes the client to spin its own thread


  ROS_INFO("Waiting for path follower action server to start.");
  ac.waitForServer();
  ROS_INFO("Action server started, subscribing to the path topic");
  ros::Subscriber sub = n.subscribe("/move_base/TrajectoryPlannerROS/global_plan", 3, path_cb);

  //rover_actions::PathFollowerGoal goal;
  rover_actions::DriveToGoal goal;
  goal.goal_pose.position.x = 0.0;
  goal.goal_pose.position.y = 0.0;
  goal.goal_pose.position.z = 0.0;

  while(n.ok())
  {
    if(start_path)
    {

      for(int i=0; i < path.poses.size();i++)
      {
          geometry_msgs::PoseStamped temp = path.poses[i];
          if(poses_dist(temp.pose,goal.goal_pose) < 0.5)
          {
             continue;
          }

          goal.goal_pose = temp.pose;
          ROS_WARN("sub goal x:%f   y:%f  ",goal.goal_pose.position.x,goal.goal_pose.position.y);
          ac.sendGoal(goal);
          //ROS_WARN("goal sent");
          //ROS_INFO_STREAM(path);

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
    }

    ros::spinOnce();
    //exit
  }

}
