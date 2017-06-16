//Basic
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Transform.h>
//For Eigen
#include <pcl/point_cloud.h>

//Messegas
#include <donkey_rover/Speed_control.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>

//Actions
#include <rover_actions/DriveToAction.h>
#include <rover_actions/DriveToOAAction.h>
#include <move_base_msgs/MoveBaseAction.h>

//Actionlib
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib/server/simple_action_server.h>
#include <donkey_rover/Speed_control.h>
#include <pcl_analyser/pathsolver.h>

enum STAT
{
  wait_,first_, ordinary_,last_,failed,done
};

using namespace Eigen;

class DriveToOAAction
{


public:

  DriveToOAAction(std::string name) :
    as_(nh_, name, boost::bind(&DriveToOAAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();

//    path_pub = nh_.advertise<nav_msgs::Path>("/path_b",1);
    body_error_pub = nh_.advertise<geometry_msgs::Vector3>(ros::this_node::getNamespace()+"/body_error",1);
    speed_ctrl_pub = nh_.advertise<donkey_rover::Speed_control>(ros::this_node::getNamespace()+"/speed_control",1);
    //path_pub = nh_.advertise<nav_msgs::Path>("/PSO_RES",1);
    ns_ = ros::this_node::getNamespace();
    //Initializers
    // Global params
    b_ = 0.4;
    b_thr_ = 0.2;
    sub_goal_distance = 0.3;
    debug_ = false;

    //Global varriables
    plan_time = ros::Time::now();
    first_path = true;
    start_path = false;
    new_goal = true;

    // pathsolver instant def
    ps_ptr = new pathsolver(&nh_,"global_costmap","elevation_costmap",0.0,3.00,50,"pcl_analyser_node");

  }

  ~DriveToOAAction(void)
  {
  }

  float poses_dist(geometry_msgs::Pose P1,geometry_msgs::Pose P2)
  {
      float distant;
      double dx = P1.position.x - P2.position.x;
      double dy = P1.position.y - P2.position.y;
      distant = sqrt(pow(dx,2)+pow(dy,2));
      return distant;
  }

  geometry_msgs::Pose PoseFromTfTransform(tf::StampedTransform t)
  {
     geometry_msgs::Pose output;
     output.position.x =  t.getOrigin().getX();
     output.position.y =  t.getOrigin().getY();
     output.position.z =  t.getOrigin().getZ();
     tf::quaternionTFToMsg (t.getRotation(),output.orientation);
     return output;
  }


  void update_curr_pose()
  {
//    tf::TransformListener listener;
    tf::StampedTransform transform;
    bool in_process = true;
    int attempt = 0;
    while(in_process)
    {
      try{
          listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
          in_process = false;
          attempt = 0;
      }
      catch (tf::TransformException ex){
          ROS_ERROR("%s",ex.what());
          in_process = true;
          ros::Duration(0.5).sleep();
          attempt ++;
      }
      if (attempt > 100)
      {
        ROS_FATAL("Transform map to baselink is lost");
        break;
      }
    }
    Curr_pose = PoseFromTfTransform(transform);
  }

  void update_curr_pose(tf::StampedTransform& transform)
  {
//    tf::TransformListener listener;
    bool in_process = true;
    int attempt = 0;
    while(in_process)
    {
      try{
          listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
          in_process = false;
          attempt = 0;
      }
      catch (tf::TransformException ex){
          ROS_ERROR_COND(attempt%50 == 0,"%s",ex.what());
          in_process = true;
          ros::Duration(0.01).sleep();
          attempt ++;
      }
      if (attempt > 600)
      {
        ROS_FATAL("Transform map to baselink is lost");
        break;
      }
    }
    Curr_pose = PoseFromTfTransform(transform);
  }

  geometry_msgs::Vector3 BodyErrorMsg(geometry_msgs::Pose Goal)
  {
      tf::StampedTransform transform;
      update_curr_pose(transform);

      // Command = 1 = ignor, 2 = Catch, 3 = Catch and Turn
      float Tx = transform.getOrigin().getX();
      float Ty = transform.getOrigin().getY();
      float yaw = tf::getYaw(transform.getRotation());
      Matrix2f Rot2x2;
      Rot2x2(0,0) = cos(yaw);  Rot2x2(0,1) =  sin(yaw);
      Rot2x2(1,0) =-sin(yaw);  Rot2x2(1,1) =  cos(yaw);

      Vector2f G_AT; //AfterTransform
      G_AT(0) = Goal.position.x - Tx;
      G_AT(1) = Goal.position.y - Ty;

      Vector2f G_ATR; //After Transform and Rotation
      G_ATR = Rot2x2*G_AT;

      G_ATR(0) -= fabs(b_);

      geometry_msgs::Vector3 Output;
      Output.x = G_ATR(0);
      Output.y = G_ATR(1);

      return Output;
  }

  void stop()
  {
    geometry_msgs::Vector3 RLC_msg;
    RLC_msg.x = 0;
    RLC_msg.y = 0;
    donkey_rover::Speed_control d_crl;
    d_crl.header.stamp = ros::Time::now();
    d_crl.CMD = false;
    d_crl.RLC = true;
    d_crl.JOY = true;
    for(int i = 0;i<10;i++)
    {
      speed_ctrl_pub.publish(d_crl);
      body_error_pub.publish(RLC_msg);
      ros::Duration(1/30).sleep();
    }
    d_crl.CMD = false;
    d_crl.RLC = false;
    d_crl.JOY = true;
    speed_ctrl_pub.publish(d_crl);
  }

  void init_param()
  {
    ros::NodeHandle npr("~");
    if(!npr.getParam("b",b_))
    {
      b_ = 0.4;
      ROS_WARN("No value is received for b, it is set to default value of %f", b_);
    }
    if(!npr.getParam("b_threshold",b_thr_))
    {
      b_thr_= 0.2;
      ROS_WARN("No value is received for b_threshold, it is set to default value of %f", b_thr_);
    }
    if(!npr.getParam("subgoal_distance",sub_goal_distance))
    {
      sub_goal_distance= 0.3;
      ROS_WARN("No value is received for sub_goal_distance, it is set to default value of %f", sub_goal_distance);
    }
    if(npr.getParam("debug_",debug_))
    {
      if(debug_)
        ROS_WARN("-----Debug Mode -----");
    }
    if(!npr.getParam("Linear_error",linear_threshold))
    {
      linear_threshold = 0.42;
      ROS_WARN("No value is received for Ainear error, it is set to default value %f",linear_threshold);
    }
  }

  void executeCB(const rover_actions::DriveToOAGoalConstPtr &Goal)  //goal = goal to DriveTo, Goal -> goal to DriveToOV
  {

    //tf::TransformListener listener;

    path_topic_name = "/PSO_RES";


    //invoking DriveTo action instant
    actionlib::SimpleActionClient<rover_actions::DriveToAction> ac_DriveTo(ros::this_node::getNamespace()+"DriveTo", true);

    ROS_INFO("Waiting for DriveTo Action to come up");
    ac_DriveTo.waitForServer();

    rover_actions::DriveToGoal goal;
    goal.goal_pose.position.x = 0.0;
    goal.goal_pose.position.y = 0.0;
    goal.goal_pose.position.z = 0.0;

    bool transform_exists = listener.waitForTransform("/map", "base_link", ros::Time(0), ros::Duration(3));
    if(!transform_exists)
    {
        ROS_FATAL("transform from base_link to map does n*ot exist, rover_actions failed :-( ");
        return;
    }

    init_param();
    // solve here
    geometry_msgs::PoseStamped goal_pose_msg;
    goal_pose_msg.header.frame_id = "map";
    goal_pose_msg.pose = Goal->goal_pose;
    path = ps_ptr->action_solve(goal_pose_msg);
//    ROS_INFO_STREAM("Path header" << path.header);

    ros::Rate r(1);
    wpstate = first_;
    ros::Duration timeout(160);
    int wpid = 0;
    start_path = true;
    while(nh_.ok())
    {

      tf::StampedTransform transform;


      try{
          listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
      }
      catch (tf::TransformException ex){
          ROS_ERROR("%s",ex.what());
          ros::Duration(1.0).sleep();

      }

      Curr_pose = PoseFromTfTransform(transform);

      //Check for the preempt request
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        stop();
        as_.setPreempted();
        break;
      }
      switch(wpstate)
      {
//      case wait_:
//        if (debug_)ROS_WARN_THROTTLE(1,"waiting for a path");
//        ros::spinOnce();

      break;
      case first_:
      {
           update_curr_pose();
           start_path = false;
           //ROS_INFO_STREAM("current pose: \n"<<Curr_pose<<"\n first wp \n"<<path.poses[wpid].pose);
           //ROS_WARN_STREAM(poses_dist(Curr_pose,path.poses[wpid].pose)<< "     " <<   fabs(b_));
           while (poses_dist(Curr_pose,path.poses[wpid].pose) < b_) {
//             ROS_WARN_STREAM("Curr_pose: " << Curr_pose.position << "  way point  : " << path.poses[wpid].pose.position
//                             << " dist  :" << poses_dist(Curr_pose,path.poses[wpid].pose));
             wpid++;
             //sleep(0.2);

           }
           //sleep(4);
           double goal_yaw = atan2(path.poses[wpid].pose.position.y-Curr_pose.position.y,
                                   path.poses[wpid].pose.position.x-Curr_pose.position.x);
           double curr_yaw = tf::getYaw(Curr_pose.orientation);
           goal.goal_pose = Curr_pose;
           goal.goal_pose.position.x += b_*cos(curr_yaw);
           goal.goal_pose.position.y += b_*sin(curr_yaw);
           goal.goal_pose.position.z = 0.0;//-100.0;  // stop the first turn in place which is wrong
           goal.goal_pose.orientation = tf::createQuaternionMsgFromYaw(goal_yaw);
           ROS_INFO("Sending first goal, path size %d and wpid: %d",(int)path.poses.size(),wpid);
           ROS_WARN_STREAM(goal.goal_pose);
           ac_DriveTo.sendGoal(goal);

           if(!ac_DriveTo.waitForResult(timeout))
           {
             ROS_ERROR("catching the fisrt goal didn't finished before the timeout");
             ac_DriveTo.cancelAllGoals();
             wpstate = failed;
             break;
           }
           else
           {
             ROS_INFO("first goal achieved successfully");
             wpstate = ordinary_;
           }
           //return;
      }
      break;
      case ordinary_:
      {
        double K_0;
        ros::param::get(ns_+"/controler/Controller_Gain",K_0);
        ros::param::set(ns_+"/controler/Controller_Gain",1*K_0);
        while(wpid < path.poses.size()-1)
        {
            geometry_msgs::Vector3 RLC_msg = BodyErrorMsg(path.poses[wpid].pose);
            donkey_rover::Speed_control d_crl;
            d_crl.header.stamp = ros::Time::now();
            d_crl.CMD = false;
            d_crl.RLC = true;
            d_crl.JOY = true;
            if (RLC_msg.x < sub_goal_distance && RLC_msg.y < linear_threshold*5 && wpid<path.poses.size()-2)
            {
              ROS_WARN("sending midd way point!");
              ROS_INFO_STREAM(RLC_msg);
              wpid++;

            }
            if(wpid == path.poses.size()-2 && RLC_msg.x < linear_threshold && RLC_msg.y < linear_threshold)
            {
              ROS_INFO("second to the last way point achieved!");
              wpid++;
            }
            speed_ctrl_pub.publish(d_crl);
            body_error_pub.publish(RLC_msg);
            ros::Duration(1/200).sleep();
        }
        //stoping

        stop();
        ros::param::set(ns_+"/controler/Controller_Gain",K_0);
        wpstate = last_;
      }
      break;

      case last_:
      {

        ROS_INFO("Sending Last waypoint");
        //update Curr_pose
        update_curr_pose();
        goal.goal_pose = Goal->goal_pose;
        float yaw = tf::getYaw(Curr_pose.orientation);
        goal.goal_pose.position.x += 1*b_*cos(yaw);
        goal.goal_pose.position.y += 1*b_*sin(yaw);
        goal.goal_pose.position.z = -100.0;
        ac_DriveTo.sendGoal(goal);
        if(!ac_DriveTo.waitForResult(timeout))
        {
          ROS_ERROR("catching ordinary goal didn't finished before the timeout");
          ac_DriveTo.cancelAllGoals();
          wpstate = failed;
          break;
        }
        wpstate = done;
      }
      break;
      case failed:
      {
          ROS_FATAL("Mission Fialed :(");
          as_.setAborted();
          return;
      }
      break;
      case done:
      {
        ROS_INFO("Mission Successful :)");
        as_.setSucceeded(result_);
        start_path = true;
        return;
      }
      break;
      }
      r.sleep();
      //exit
    }

  }

protected:
  tf::TransformListener listener;
  ros::NodeHandle nh_;
  std::string ns_;
  STAT wpstate; //way point state
  actionlib::SimpleActionServer<rover_actions::DriveToOAAction> as_;
  std::string action_name_;
  ros::Publisher path_pub;
  ros::Publisher body_error_pub;
  ros::Publisher speed_ctrl_pub;
  //geometry_msgs::Pose current_pose;
  //geometry_msgs::Pose sub_goal_pose;
  ros::Subscriber sub_from_path;
  rover_actions::DriveToOAFeedback feedback_;
  rover_actions::DriveToOAResult result_;
  double b_;
  double b_thr_;
  std::string map_frame_id;
  double linear_threshold;
  double sub_goal_distance;
  bool debug_;
  std::string path_topic_name;
  geometry_msgs::Pose Curr_pose;
  nav_msgs::Path path;
  nav_msgs::Path path_updated;
  bool first_path;
  bool start_path;
  bool new_goal;
  ros::Time plan_time;
  pathsolver *ps_ptr;
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "DriveToOA");

  DriveToOAAction DriveToOA(ros::this_node::getName());
  while(ros::ok())
  {
    ros::Duration(0.5).sleep();
    ros::spinOnce();
  }
  //ros::spin();

  return 0;
}
