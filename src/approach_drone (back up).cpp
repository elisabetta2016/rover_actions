#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <rover_actions/DriveToAction.h>
#include <rover_actions/DriveToOAAction.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <iostream>
#include <ros/package.h>
enum state
{
  def,drone_detected,move_back,approach_again,final_move,mission_complete, just_appraoch
};
enum GoalState
{ NN, WAIT, TIMEOUT,DONE};
state S = def;
geometry_msgs::PoseStamped drone_pose;
char* str2char( std::string str ) {
  char *c = new char[ str.length()+1 ];
  strcpy( c, str.c_str());
  return c;
}
bool drone_pose_exist = false;
void dronepose_cb(const geometry_msgs::PoseStamped::ConstPtr msg)
{
  drone_pose = *msg;
  double yaw = tf::getYaw(drone_pose.pose.orientation);
  drone_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw+M_PI);
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
  ros::Subscriber drone_pose_sub = nh.subscribe("/uav_pose",1,dronepose_cb);//move_base_simple/goal

  ros::Rate rate(10); // 10 hz
  tf::TransformListener rover_listener;
  tf::StampedTransform rover_transform;
  geometry_msgs::Pose rover_pose;
  ROS_INFO("LOADING PARAMS");

  std::string system_arg = "rosparam load " +
      ros::package::getPath("rover_actions") + "/config/DriveTo.yaml /DriveTo";
  system(str2char(system_arg));
  system_arg = "rosparam load " +
        ros::package::getPath("rover_actions") + "/config/DriveToOA.yaml /DriveToOA";
  system(str2char(system_arg));
  ROS_ERROR_STREAM(system_arg);
  double drone_yaw;
  double x_offset = 1.00; //To be determined accurately
  tf::Vector3 drone_rov;
  tf::Vector3 map_drone;
  tf::Vector3 approach_goal;
  tf::Vector3 rov,rov_b;


  bool just_appraoch_mode = false;

  actionlib::SimpleActionClient<rover_actions::DriveToOAAction> acOA("DriveToOA", true);
  actionlib::SimpleActionClient<rover_actions::DriveToAction> ac("DriveTo", true);
  actionlib::SimpleClientGoalState ac_state = acOA.getState();
  //GoalState gstatel = NN;

  ROS_INFO("APPROACH DRONE:Waiting for action server to start.");
  ac.waitForServer();
  acOA.waitForServer();
  ROS_INFO("APPROACH DRONE:Action server started, sending goal.");
  double b_ = 0.4;
  nh.setParam(ros::this_node::getNamespace()+"/controler/distance_b",b_);
  rover_actions::DriveToGoal goal;
  rover_actions::DriveToOAGoal goalOA;
  if (argc == 2) just_appraoch_mode = true;
  double K_Move = 1.0;
  while (nh.ok()) {
    try{
      rover_listener.lookupTransform("/map", "/base_link", ros::Time(0), rover_transform);
      rover_pose = PoseFromTransform(rover_transform);
      rov.setValue(rover_pose.position.x,rover_pose.position.y,rover_pose.position.z);
      rov_b = tf::Vector3(b_,0,0).rotate(tf::Vector3(0,0,1),tf::getYaw(rover_pose.orientation));
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
      if (just_appraoch_mode) S=just_appraoch;
      break;
    case move_back:
      ROS_INFO("APPROACH DRONE:Moving Back");
      approach_goal = rov + rov_b+ tf::Vector3(0.2,0,0).rotate(tf::Vector3(0,0,1),tf::getYaw(rover_pose.orientation));

      goal.goal_pose.position.x = approach_goal.getX();
      goal.goal_pose.position.y = approach_goal.getY();
      goal.goal_pose.orientation.w = 1.0;

      ac_state = ac.sendGoalAndWait(goal,ros::Duration(350));
      //ROS_INFO("Goal state %s",);
      if (ac.getState().toString().compare("SUCCEEDED")==0)
      {
        //return 0;
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
      //nh.setParam(ros::this_node::getNamespace()+"/controler/distance_b",-0.4); // here
      if(ros::param::get(ros::this_node::getNamespace()+"/controler/Controller_Gain",K_Move))
          ros::param::set(ros::this_node::getNamespace()+"/controler/Controller_Gain",K_Move*0.5);
      approach_goal = map_drone + drone_rov;
      //approach_goal += tf::Vector3(b_,0,0).rotate(tf::Vector3(0,0,1),approach_goal.angle(tf::Vector3(1,0,0)));
      //PRINT_V3(map_drone,"map_drone","info");
      //PRINT_V3(drone_rov,"drone_rov","warn");
      //PRINT_V3(approach_goal,"goal","error");
      ac.cancelAllGoals();
      goalOA.goal_pose.position.x = approach_goal.getX();
      goalOA.goal_pose.position.y = approach_goal.getY();
      goalOA.goal_pose.position.z = 0.0;//-100.000;
      goalOA.goal_pose.orientation.w = 1.0;
      ac_state = acOA.sendGoalAndWait(goalOA,ros::Duration(350));
      if (ac_state.toString().compare("SUCCEEDED")==0)
      {
        ROS_INFO("APPROACH DRONE:approach again successful");
        S = final_move;
      }
      else
      {
        ROS_ERROR("APPROACH DRONE: Fuck something went wrong, second approach failed");
        ROS_ERROR("APPROACH DRONE: mission failed!");
        acOA.cancelAllGoals();
        //return 0;
        S = def;
      }
      break;
    case final_move:
      approach_goal = rov + rov_b + rov_b;
      goal.goal_pose.position.x = approach_goal.getX();
      goal.goal_pose.position.y = approach_goal.getY();
      goal.goal_pose.position.z = -100.000;
      goal.goal_pose.orientation = tf::createQuaternionMsgFromYaw(drone_yaw+3.14);
      ac_state = ac.sendGoalAndWait(goal,ros::Duration(150));
      if (ac_state.toString().compare("SUCCEEDED")==0)
      {
        ROS_INFO("APPROACH DRONE:Final Move successful");
        S = mission_complete;
      }
      else
      {
        ROS_ERROR("APPROACH DRONE: Fuck something went wrong, final move failed");
        ROS_ERROR("APPROACH DRONE: mission failed!");
        ac.cancelAllGoals();
        //return 0;
        S = def;
      }
      break;
    case mission_complete:
      nh.setParam(ros::this_node::getNamespace()+"/controler/distance_b",0.4);
      ROS_INFO("APPROACH DRONE:Mission complete Ready to pickup the drone");
      ac.cancelAllGoals();
      acOA.cancelAllGoals();
      //return 0;
      S = def;
      break;
    default:
      ROS_FATAL("APPROACH DRONE:bad implementation");
      S = def;
      break;
    }

    rate.sleep();
  }
}
