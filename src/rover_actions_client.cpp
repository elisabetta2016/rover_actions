#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <rover_actions/DriveToAction.h>
#include <rover_actions/PathFollowerAction.h>
#include <nav_msgs/Path.h>
#include <tf/LinearMath/Transform.h>
#include <tf/transform_datatypes.h>
#include <pcl/point_cloud.h>
using namespace Eigen;

nav_msgs::Path path;
bool first_path = true;
bool start_path = false;
float b_ = 0.4;
float b_thr_ = 0.2;
geometry_msgs::Pose Curr_pose;

void CheckBodyError(geometry_msgs::Pose Current_pose, geometry_msgs::Pose Goal,bool last_sub_goal ,int8_t& command)
{
    // Command : 1 = ignor, 2 = Catch, 3 = Catch and Turn
    float Tx = Current_pose.position.x;
    float Ty = Current_pose.position.y;
    float yaw = tf::getYaw(Current_pose.orientation);
    Matrix2f Rot2x2;
    Rot2x2(0,0) = cos(yaw);  Rot2x2(0,1) =  sin(yaw);
    Rot2x2(1,0) =-sin(yaw);  Rot2x2(1,1) =  cos(yaw);

    //ROS_WARN("Tx:%f    Ty:%f    Yaw:%f",Tx,Ty,yaw*180/M_PI);

    Vector2f G_AT; //AfterTransform
    G_AT(0) = Goal.position.x - Tx;
    G_AT(1) = Goal.position.y - Ty;
    //ROS_WARN("goal x:%f  y:%f",Goal.position.x,Goal.position.y);

    Vector2f G_ATR; //After Transform and Rotation
    G_ATR = Rot2x2*G_AT;

    /*
    float y = yaw*180/M_PI;
    ROS_WARN_STREAM("G_AT: \n"<<G_AT);
    ROS_INFO_STREAM("Rot2x2: \n"<<
                    " cos "<<y<<"     sin "<<y<<"  \n"<<
                    "-sin "<<y<<"     cos "<<y<<"  \n");

    ROS_ERROR_STREAM("G_ATR: \n"<<G_ATR);
    */
    // -------Generate the command:
    // --- Convert from Body frame to b frame
    G_ATR(0) -= b_;
    // --- Decision
    if (G_ATR(0) < b_thr_)
      command = 1;
    else
      command = 2;
    if (last_sub_goal) // -100.00 is last point flag
      command = 3;

}

geometry_msgs::Pose last_pose_shift(geometry_msgs::Pose Goal,float b)
{
  float yaw = tf::getYaw(Curr_pose.orientation);
  geometry_msgs::Pose last_pose = Goal;
  last_pose.position.x += b*cos(yaw);
  last_pose.position.y += b*sin(yaw);
  last_pose.position.z = -100.00;  //Trun in place signal


  return last_pose;
}

bool pose_compare(geometry_msgs::PoseStamped p1, geometry_msgs::PoseStamped p2)
{
  bool out = true;
  if (abs(p1.pose.position.x - p2.pose.position.x)>0.01)
  {
    out = false;
    return out;
  }
  if (abs(p1.pose.position.y - p2.pose.position.y)>0.01)
  {
    out = false;
    return out;
  }
  if (abs(p1.pose.orientation.w - p2.pose.orientation.w)>0.01)
  {
    out = false;
    return out;
  }
  return out;
}

bool is_a_newpath(nav_msgs::Path pold, nav_msgs::Path pnew)
{
    bool output = false;
    int pold_s = pold.poses.size();
    int pnew_s = pnew.poses.size();
    /*
    if(pold_s != pnew_s)
    {
      output = true;
    }
    else
    */
    if(true)
    {
      for(int i = 0; i< std::min(pnew_s,pold_s);i++)
      {
        if(!pose_compare(pold.poses[i],pnew.poses[i]))
        {
          output = true;
          break;
        }
      }
    }
    return output;
}

void path_cb(const nav_msgs::Path::ConstPtr& msg)
{
    if(first_path)
    {
       path = *msg;
       start_path = true;
       first_path = false;
    }
    else if(is_a_newpath(path, *msg))
    {
      path = *msg;
      start_path = true;
      first_path = false;
    }
}

void feedback_cb(const rover_actions::DriveToActionFeedback::ConstPtr& msg)
{
  Curr_pose = msg->feedback.current_pose;
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
  ros::Subscriber sub_from_path = n.subscribe("/move_base/TrajectoryPlannerROS/global_plan", 3, path_cb);
  ros::Subscriber sub_from_feedback = n.subscribe("/DriveTo/feedback",3,feedback_cb);
  //rover_actions::PathFollowerGoal goal;
  rover_actions::DriveToGoal goal;
  goal.goal_pose.position.x = 0.0;
  goal.goal_pose.position.y = 0.0;
  goal.goal_pose.position.z = 0.0;

  while(n.ok())
  {
    if(start_path)
    {
      ROS_INFO_STREAM("Path size:   "<< path.poses.size());
      for(int i=0; i < path.poses.size();i++)
      {
          geometry_msgs::PoseStamped temp_goal = path.poses[i];
          /*
          if(poses_dist(temp.pose,goal.goal_pose) < 0.5 && i!= (path.poses.size()-1) )
          {
             ROS_WARN("Index %d ingnored",i);
             continue;
          }*/
          int8_t command;
          bool last_sub_goal = false;
          if(i==(path.poses.size()-1))
            last_sub_goal = true;
          //rover_actions::DriveToFeedback feedback_ = ac.ActionFeedback;

          CheckBodyError(Curr_pose, temp_goal.pose, last_sub_goal ,command);
          switch (command) {// Command : 1 = ignor, 2 = Catch, 3 = Catch and Turn
          case 1:
          {
            ROS_WARN("Index %d ingnored",i);
            continue;
          }
            break;
          case 2:
          {
            goal.goal_pose = temp_goal.pose;
            ROS_WARN("index %d  sub goal x:%f   y:%f  ",i,goal.goal_pose.position.x,goal.goal_pose.position.y);
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
            break;
          case 3:
          {
            goal.goal_pose = temp_goal.pose;
            //goal.goal_pose.position.z = -100.00;
            ROS_WARN("index %d  sub goal x:%f   y:%f  ",i,goal.goal_pose.position.x,goal.goal_pose.position.y);
            ROS_INFO("Last Goal");
            ac.sendGoal(goal);

            //wait for the action to return
            bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

            if (finished_before_timeout)
            {
                actionlib::SimpleClientGoalState state = ac.getState();
                ROS_INFO("Action finished: %s",state.toString().c_str());
                goal.goal_pose = last_pose_shift(goal.goal_pose,b_);
                ROS_INFO("Final phase");
                ac.sendGoal(goal);
                if(ac.waitForResult(ros::Duration(30.0)))
                  ROS_INFO("PATH ACHIEVED SUCCESSFULLY");

            }
            else
                ROS_INFO("Action did not finish before the time out.");
          }
          default:
            ROS_ERROR("unexpected command, bad implementation correct the code!!!");
            break;
          }

    }
       start_path = false;
    }

    ros::spinOnce();
    //exit
  }

}
