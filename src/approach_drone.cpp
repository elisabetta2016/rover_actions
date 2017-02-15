#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <rover_actions/DriveToAction.h>
#include <actionlib/client/simple_client_goal_state.h>
#include<geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include<geometry_msgs/TransformStamped.h>

enum state
{
  def,drone_detected,move_back,approach_again,mission_complete
};
state S = def;
geometry_msgs::PoseStamped drone_pose;

bool drone_pose_exist = false;
void dronepose_cb(const geometry_msgs::PoseStamped::ConstPtr msg)
{
  drone_pose = *msg;
  ROS_INFO("APPROACH DRONE: drone msg received!");
  if(!drone_pose_exist) S = drone_detected;
  drone_pose_exist = true;
}
void PRINT_V3(tf::Vector3 V,std::string S,std::string typ)
{
  if (typ.compare("info") == 0)
    ROS_INFO_STREAM(S<<":  x:"<<V.getX()<<"  y:"<< V.getY()<<"  z:"<<V.getZ());
  if (typ.compare("warn") == 0)
    ROS_WARN_STREAM(S<<":  x:"<<V.getX()<<"  y:"<< V.getY()<<"  z:"<<V.getZ());
  if (typ.compare("error") == 0)
    ROS_ERROR_STREAM(S<<":  x:"<<V.getX()<<"  y:"<< V.getY()<<"  z:"<<V.getZ());
}
geometry_msgs::Pose PoseFromTransform(tf::StampedTransform trans)
{
  geometry_msgs::TransformStamped temp;
  tf::transformStampedTFToMsg(trans,temp);
  geometry_msgs::Pose msg;
  msg.position.x = temp.transform.translation.x;
  msg.position.y = temp.transform.translation.y;
  msg.position.z = temp.transform.translation.z;
  msg.orientation = temp.transform.rotation;
  return msg;
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "approach_drone");
  ros::NodeHandle nh;
  ros::Subscriber drone_pose_sub = nh.subscribe("/move_base_simple/goal",1,dronepose_cb);

  ros::Rate rate(10); // 10 hz
  tf::TransformListener rover_listener;
  tf::StampedTransform rover_transform;
  geometry_msgs::Pose rover_pose;

  double drone_yaw;
  double x_offset = 1.50; //To be determined accurately
  tf::Vector3 drone_rov;
  tf::Vector3 map_drone;
  tf::Vector3 approach_goal;
  tf::Vector3 rov;

  actionlib::SimpleActionClient<rover_actions::DriveToAction> ac("DriveTo", true);
  actionlib::SimpleClientGoalState ac_state = ac.getState();
  ROS_INFO("APPROACH DRONE:Waiting for action server to start.");
  ac.waitForServer();
  ROS_INFO("APPROACH DRONE:Action server started, sending goal.");
  nh.setParam(ros::this_node::getNamespace()+"/controler/distance_b",0.4);
  rover_actions::DriveToGoal goal;
  while (nh.ok()) {
    try{
      rover_listener.lookupTransform("/map", "/base_link", ros::Time(0), rover_transform);
      rover_pose = PoseFromTransform(rover_transform);
      rov.setValue(rover_pose.position.x,rover_pose.position.y,rover_pose.position.z);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    switch (S) {
    case def:
      ROS_WARN_ONCE("APPROACH DRONE:Wating for /drone msg to be received");
      ros::spinOnce();
      break;
    case drone_detected:
      ROS_INFO("APPROACH DRONE:Drone Detected");
      drone_pose_sub.shutdown();
      S = move_back;
      break;
    case move_back:
      ROS_INFO("APPROACH DRONE:Moving Back");
      rov = rov +tf::Vector3(0.9,0,0).rotate(tf::Vector3(0,0,1),tf::getYaw(rover_pose.orientation));

      goal.goal_pose.position.x = rov.getX();
      goal.goal_pose.position.y = rov.getY();
      goal.goal_pose.orientation.w = 1.0;

      ac_state = ac.sendGoalAndWait(goal,ros::Duration(20));
      //ROS_INFO("Goal state %s",);
      if (ac_state.toString().compare("SUCCEEDED")==0)
      {
        S = approach_again;
        //return 0;
      }
      else
      {
        ROS_ERROR("APPROACH DRONE:Fuck something went wrong, moving back failed");
        return 0;
      }
      break;
    case approach_again:
      drone_yaw = tf::getYaw(drone_pose.pose.orientation);
      //drone_rov.setValue(-x_offset*cos(drone_yaw),-x_offset*sin(drone_yaw),0.0);
      drone_rov.setValue(-x_offset,0,0);
      drone_rov = drone_rov.rotate(tf::Vector3(0,0,1),drone_yaw);
      map_drone.setValue(drone_pose.pose.position.x,drone_pose.pose.position.y,0.0);
      nh.setParam(ros::this_node::getNamespace()+"/controler/distance_b",-0.4); // here
      approach_goal = map_drone + drone_rov;
      PRINT_V3(map_drone,"map_drone","info");
      PRINT_V3(drone_rov,"drone_rov","warn");
      PRINT_V3(approach_goal,"goal","error");
      ac.cancelAllGoals();
      goal.goal_pose.position.x = approach_goal.getX();
      goal.goal_pose.position.y = approach_goal.getY();
      goal.goal_pose.position.z = -100.000;
      goal.goal_pose.orientation = tf::createQuaternionMsgFromYaw(drone_yaw+3.14);
      ac_state = ac.sendGoalAndWait(goal,ros::Duration(20));
      //ac.sendGoal(goal);
      //ac.waitForResult(ros::Duration(100));
      //ac_state = ac.getState();
      if (ac_state.toString().compare("SUCCEEDED")==0)
      {
        ROS_INFO("APPROACH DRONE:approach again successful");
        S = mission_complete;
      }
      else
      {
        ROS_ERROR("APPROACH DRONE: Fuck something went wrong, second approach failed");
        ROS_ERROR("APPROACH DRONE: mission failed!");
        ac.cancelAllGoals();
        return 0;
      }
      break;
    case mission_complete:
      nh.setParam(ros::this_node::getNamespace()+"/controler/distance_b",0.4);
      ROS_INFO("APPROACH DRONE:Mission complete Ready to pickup the drone");
      ac.cancelAllGoals();
      return 0;
      break;
    default:
      ROS_FATAL("APPROACH DRONE:bad implementation");
      S = def;
      break;
    }

    rate.sleep();
  }
}
