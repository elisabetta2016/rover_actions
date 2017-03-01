#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <rover_actions/DriveToAction.h>
#include <rover_actions/DriveToOAAction.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <donkey_rover/Speed_control.h>
#include <iostream>
#include <ros/package.h>
#include <geometry_msgs/Vector3.h>
enum state
{
  def,drone_detected,move_back,approach_again,b_offset,final_approach,final_turn,mission_complete, mission_failed
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
bool subscriber_shutdown = false;
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

tf::StampedTransform get_wasp_relative_dist(bool& success,unsigned int Filter_sample)
{
  using namespace tf;
  TransformListener listener;
  StampedTransform transform;
  StampedTransform TRANS;
  unsigned int sample = 0;
  int attempt = 0;
  std::string arm_base = "sherpa_arm_base";
  std::string wasp_frame = "Wasp0";
  if(listener.waitForTransform(arm_base,"Wasp0",ros::Time(0),ros::Duration(2)))
    wasp_frame = "Wasp0";
  else if(listener.waitForTransform(arm_base,"Wasp1",ros::Time(0),ros::Duration(2)))
    wasp_frame = "Wasp1";
  else
  {
    ROS_ERROR("No wasp found");
    success = false;
    return TRANS;
  }

  while(sample < Filter_sample )
  {
    try{
        listener.lookupTransform(arm_base, wasp_frame, ros::Time(0), transform);
        sample++;
        if (sample == 1)
          TRANS = transform;
        else
        {
          TRANS.setOrigin((transform.getOrigin()+TRANS.getOrigin())/2);
        }
        attempt = 0;
    }
    catch (tf::TransformException ex){
      ROS_ERROR_COND(attempt%50 == 0,"%s",ex.what());
      ros::Duration(0.01).sleep();
      attempt ++;
    }

    if (attempt > 100)
    {
      ROS_FATAL("Transform map to baselink is lost");
      break;
    }
  }
  return TRANS;

}

tf::StampedTransform get_trans_filter(std::string parent_,std::string child_, unsigned int Filter_sample)
{
  using namespace tf;
  TransformListener listener;
  StampedTransform transform;
  StampedTransform TRANS;
  unsigned int sample = 0;
  int attempt = 0;

  while(sample < Filter_sample )
  {
    try{
        listener.lookupTransform(parent_, child_, ros::Time(0), transform);
        sample++;
        if (sample == 1)
          TRANS = transform;
        else
        {
          TRANS.setOrigin((transform.getOrigin()+TRANS.getOrigin())/2);
        }
        attempt = 0;
    }
    catch (tf::TransformException ex){
      ROS_ERROR_COND(attempt%50 == 0,"%s",ex.what());
      ros::Duration(0.01).sleep();
      attempt ++;
    }

    if (attempt > 100)
    {
      ROS_FATAL("TRANSFORM NOT FOUND");
      break;
    }
  }
  return TRANS;

}

bool straight_move(ros::NodeHandle* nh_ptr, float dist, ros::Duration timeout)
{
  ros::Publisher RL_pub    = nh_ptr->advertise<geometry_msgs::Vector3>("/speedfollow",1);
  ros::Publisher speed_ctrl = nh_ptr->advertise<donkey_rover::Speed_control>("/speed_control",1);
  geometry_msgs::Vector3 RL_msg;
  RL_msg.z = 0;
  donkey_rover::Speed_control sctrl_msg;
  sctrl_msg.CMD = false;
  sctrl_msg.RLC = true;
  sctrl_msg.JOY = true;
  tf::StampedTransform T0 = get_trans_filter("map","base_link",3);
  float e = dist;
  float P,I,D,e_acc;
  e_acc = 0;
  P = 1.3;
  I = 0.4;
  ros::Rate r_(10);
  ros::Time time0;
  time0 = ros::Time::now();

  while(fabs(e)>0.05)
  {
    RL_msg.x = P*e ;//+ I*e_acc;
    RL_msg.y = P*e ;
    speed_ctrl.publish(sctrl_msg);
    RL_pub.publish(RL_msg);
    r_.sleep();
    tf::StampedTransform T1 = get_trans_filter("map","base_link",3);
    if (dist >= 0)
    {
      e = dist - T1.getOrigin().distance(T0.getOrigin());
      if (e < 0) dist = e;
    }
    else
    {
      e = dist + T1.getOrigin().distance(T0.getOrigin());
      if (e > 0) dist = e;
    }

    //ROS_INFO("e: %f",e);
    e_acc += e;
    if(ros::Time::now()-time0 > timeout)
    {
      ROS_ERROR("Move straight failed");
      return false;
    }
  }
  ROS_INFO("APPROACH DRONE: Moving straight Successfull");
  return true;
}

inline bool waitForDriveTo(double timeout,actionlib::SimpleActionClient<rover_actions::DriveToAction>* ac)
{

  ros::Time T1 = ros::Time::now();
  while ((ros::Time::now()-T1).toSec()<timeout)
  {
    if (ac->getState().toString().compare("SUCCEEDED") == 0)
      return true;
  }
  return false;
}

inline bool waitForDriveToOA(double timeout,actionlib::SimpleActionClient<rover_actions::DriveToOAAction>* acOA)
{
  ros::Time T1 = ros::Time::now();
  while ((ros::Time::now()-T1).toSec()<timeout)
  {
    if (acOA->getState().toString().compare("SUCCEEDED") == 0)
      return true;
  }
  return false;
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "approach_drone");
  ros::NodeHandle nh;
  ros::Subscriber drone_pose_sub = nh.subscribe("/uav_pose",1,dronepose_cb);//move_base_simple/goal
  //ros::Publisher cmd_vel_pub = nh.advertise <geometry_msgs::Twist>("/cmd_vel",1);
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
  double x_offset = 2.20; //To be determined accurately
  tf::Vector3 drone_rov;
  tf::Vector3 map_drone;
  tf::Vector3 approach_goal;
  tf::Vector3 rov,rov_b;
  ros::Time time_0;


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
      ROS_WARN_ONCE("APPROACH DRONE:Wating for /uav_pose msg to be received");
      ros::spinOnce();
      break;
    case drone_detected:
      ROS_INFO("APPROACH DRONE:Drone Detected");
      //drone_pose_sub.shutdown();
      S = move_back;
      break;
    case move_back:

      ROS_INFO("APPROACH DRONE:Moving Back");
      approach_goal = rov + rov_b+ tf::Vector3(0.2,0,0).rotate(tf::Vector3(0,0,1),tf::getYaw(rover_pose.orientation));

      goal.goal_pose.position.x = approach_goal.getX();
      goal.goal_pose.position.y = approach_goal.getY();
      goal.goal_pose.orientation.w = 1.0;
      time_0 = ros::Time::now();
      ac_state = ac.sendGoalAndWait(goal,ros::Duration(350.0));

      if (ac.getState().toString().compare("SUCCEEDED")==0)
      {
        //return 0;
        S = approach_again;
        //return 0;
      }
      else
      {
        ROS_ERROR("APPROACH DRONE:Fuck something went wrong, moving back failed");
        ROS_INFO("waiting time: %f seconds",(ros::Time::now()-time_0).toSec());
        ac.cancelAllGoals();
        S = mission_failed;
      }
      break;
    case approach_again:
      ROS_INFO("APPROACH DRONE: approaching again");
      drone_yaw = tf::getYaw(drone_pose.pose.orientation);
      //  x_offset = 2.20
      drone_rov.setValue(-x_offset,0,0);
      drone_rov = drone_rov.rotate(tf::Vector3(0,0,1),drone_yaw);
      map_drone.setValue(drone_pose.pose.position.x,drone_pose.pose.position.y,0.0);      
      if(ros::param::get(ros::this_node::getNamespace()+"/controler/Controller_Gain",K_Move))
          ros::param::set(ros::this_node::getNamespace()+"/controler/Controller_Gain",K_Move*0.5);
      approach_goal = map_drone + drone_rov;

      ac.cancelAllGoals();
      goalOA.goal_pose.position.x = approach_goal.getX();
      goalOA.goal_pose.position.y = approach_goal.getY();
      goalOA.goal_pose.position.z = -100.000;
      goalOA.goal_pose.orientation = tf::createQuaternionMsgFromYaw(drone_yaw+3.14);
      time_0 = ros::Time::now();
      acOA.sendGoal(goalOA);
      //ac_state = acOA.sendGoalAndWait(goalOA,ros::Duration(350.0));
      if (waitForDriveToOA(350.0,&acOA))
      {
        ROS_INFO("APPROACH DRONE:approach again successful");
        S = final_turn;
      }
      else
      {
        ROS_ERROR("APPROACH DRONE: Fuck something went wrong, second approach failed");
        ROS_INFO("waiting time: %f seconds",(ros::Time::now()-time_0).toSec());
        acOA.cancelAllGoals();
        //return 0;
        S = mission_failed;
      }
      break;
    case final_turn:
    {
      ROS_INFO("APPROACH DRONE: Final Turn");
      bool success = true;
      tf::StampedTransform arm_wasp = get_wasp_relative_dist(success, 5);
      goal.goal_pose.position.z = -50.0;
      ROS_INFO("angle to wasp: %f",atan2(arm_wasp.getOrigin().getY(),arm_wasp.getOrigin().getX())*180/M_PI);
      goal.goal_pose.orientation = tf::createQuaternionMsgFromYaw(atan2(arm_wasp.getOrigin().getY(),arm_wasp.getOrigin().getX()));
//      if(success)
//      {
//        ac.sendGoal(goal);
//        waitForDriveTo(40,&ac);
//        S = final_approach;
//      }
//      else
//        ROS_WARN("No wasp visibale Going back anyways");
      S = final_approach;
    }
    break;
    case final_approach:
    {
      ROS_INFO("APPROACH DRONE: Final approach");
      bool success = true;
      tf::StampedTransform arm_wasp = get_wasp_relative_dist(success, 5);
      if(straight_move(&nh,1-arm_wasp.getOrigin().getX(),ros::Duration(40)))
        S = mission_complete;
      else
        S = mission_failed;
      success = false;
      arm_wasp = get_wasp_relative_dist(success, 5);
      ROS_INFO("angle to wasp: %f",atan2(arm_wasp.getOrigin().getY(),arm_wasp.getOrigin().getX())*180/M_PI);
    }
      break;
    case mission_complete:
      nh.setParam(ros::this_node::getNamespace()+"/controler/distance_b",0.4);
      ROS_INFO("APPROACH DRONE:Mission complete Ready to pickup the drone");
      ac.cancelAllGoals();
      acOA.cancelAllGoals();
      drone_pose_exist = false;
      //return 0;
      S = def;
      break;
    case mission_failed:
      ROS_ERROR("APPROACH DRONE: Mission failed, we fucked up, hope to do better next time :>");
      drone_pose_exist = false;
      S = def;
    default:
      ROS_FATAL("APPROACH DRONE:bad implementation");
      S = def;
      break;
    }

    rate.sleep();
  }
}
