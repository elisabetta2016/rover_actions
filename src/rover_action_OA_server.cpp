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
    sub_from_path = nh_.subscribe("/move_base/TrajectoryPlannerROS/global_plan", 3, &DriveToOAAction::path_cb, this);
    path_pub = nh_.advertise<nav_msgs::Path>("/path_b",1);
    body_error_pub = nh_.advertise<geometry_msgs::Vector3>(ros::this_node::getNamespace()+"/body_error",1);
    speed_ctrl_pub = nh_.advertise<donkey_rover::Speed_control>(ros::this_node::getNamespace()+"/speed_control",1);

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
  }

  ~DriveToOAAction(void)
  {
  }

  bool is_far_enough(geometry_msgs::Pose Goal)
  {
    float Tx = Curr_pose.position.x;
    float Ty = Curr_pose.position.y;
    float yaw = tf::getYaw(Curr_pose.orientation);
    Matrix2f Rot2x2;
    Rot2x2(0,0) = cos(yaw);  Rot2x2(0,1) =  sin(yaw);
    Rot2x2(1,0) =-sin(yaw);  Rot2x2(1,1) =  cos(yaw);


    Vector2f G_AT; //AfterTransform
    G_AT(0) = Goal.position.x - Tx;
    G_AT(1) = Goal.position.y - Ty;

    Vector2f G_ATR; //After Transform and Rotation
    G_ATR = Rot2x2*G_AT;
    G_ATR(0) -= b_;
    if (G_ATR.norm() < sub_goal_distance)
      return false;
    return true;
  }

  void CheckBodyError(geometry_msgs::Pose Current_pose, geometry_msgs::Pose Goal,bool is_last_sub_goal ,bool is_first_sub_goal,int8_t& command)
  {
      if(is_first_sub_goal && new_goal) {
        command = 4;
        //if(debug_) ROS_INFO("Yaw: %f",yaw);
        wpstate = first_;
        return;
      }
      // Command : 1 = ignor, 2 = Catch, 3 = Catch and Turn, 4 = only Turn
      float Tx = Current_pose.position.x;
      float Ty = Current_pose.position.y;
      float yaw = tf::getYaw(Current_pose.orientation);
      Matrix2f Rot2x2;
      Rot2x2(0,0) = cos(yaw);  Rot2x2(0,1) =  sin(yaw);
      Rot2x2(1,0) =-sin(yaw);  Rot2x2(1,1) =  cos(yaw);

      //if(debug_) ROS_WARN("Tx:%f    Ty:%f    Yaw:%f",Tx,Ty,yaw*180/M_PI);

      Vector2f G_AT; //AfterTransform
      G_AT(0) = Goal.position.x - Tx;
      G_AT(1) = Goal.position.y - Ty;
      //if(debug_) ROS_WARN("goal x:%f  y:%f",Goal.position.x,Goal.position.y);

      Vector2f G_ATR; //After Transform and Rotation
      //if(debug_) ROS_INFO_STREAM(Rot2x2);
      if(debug_) ROS_WARN_STREAM(G_AT);
      G_ATR = Rot2x2*G_AT;
      if(debug_) ROS_INFO_STREAM("vector is   " << G_ATR << "   its norm: "<<G_ATR.norm());
      G_ATR(0) -= b_;

      // --- Decision
      if(debug_) ROS_INFO_STREAM("vector is   " << G_ATR << "   its norm: "<<G_ATR.norm());
      if (G_ATR(0) < b_thr_ || G_ATR.norm() < sub_goal_distance)
      {
        command = 1;
        //wpstate = tooclose;
      }
      else
      {
        command = 2;
        wpstate = ordinary_;
      }
      if (is_last_sub_goal) // -100.00 is last point flag
      {
        command = 3;
        wpstate = last_;
      }

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
    bool out = true; // True is inputs are equal
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
    if (abs(tf::getYaw(p1.pose.orientation) - tf::getYaw(p2.pose.orientation))>0.05)
    {
      out = false;
      return out;
    }
    return out;
  }

  bool pose_compare(geometry_msgs::PoseStamped p1, geometry_msgs::PoseStamped p2, float lin_err, float ang_err)
  {
    bool out = true; // True is inputs are equal
    if (abs(p1.pose.position.x - p2.pose.position.x)>lin_err)
    {
      out = false;
      return out;
    }
    if (abs(p1.pose.position.y - p2.pose.position.y)>lin_err)
    {
      out = false;
      return out;
    }
    if (abs(tf::getYaw(p1.pose.orientation) - tf::getYaw(p2.pose.orientation))>ang_err)
    {
      out = false;
      return out;
    }
    return out;
  }

  bool pose_compare(geometry_msgs::Pose p1, geometry_msgs::Pose p2, float lin_err, float ang_err)
  {
    bool out = true; // True is inputs are equal
    if (abs(p1.position.x - p2.position.x)>lin_err)
    {
      out = false;
      return out;
    }
    if (abs(p1.position.y - p2.position.y)>lin_err)
    {
      out = false;
      return out;
    }
    if (abs(tf::getYaw(p1.orientation) - tf::getYaw(p2.orientation))>ang_err)
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
      if(pold_s != pnew_s)
      {
        output = true;
      }
      else
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

      if(pose_compare(pold.poses[pold.poses.size()-1],pnew.poses[pnew.poses.size()-1]))
      {
        //if(debug_) ROS_INFO("New Path to the Old Goal");
        new_goal = false; // in case the new path has the same tail
      }
      else
      {
        if(debug_) ROS_INFO("---------- New Path and New Goal ------------");
        new_goal = true;
      }
      return output;
  }
  void path_cb(const nav_msgs::Path::ConstPtr& msg)
  {
    if (!start_path) return;
    if ((ros::Time::now()-plan_time).toSec()>1.5) return;
    ROS_INFO("Path Received");
    wpstate = first_;
    path = *msg;
    path.header.frame_id = "map";
    path_pub.publish(path);
    start_path = false;
  }
/*
  void path_cb(const nav_msgs::Path::ConstPtr& msg)
  {
      if(first_path)
      {
         path = *msg;
         new_goal = true;
         start_path = true;
         first_path = false;
         ROS_INFO("first path received :)");
         wpstate = first_;
      }
      else if(is_a_newpath(path, *msg))
      {
        geometry_msgs::PoseStamped temp_curr_pose;
        temp_curr_pose.pose = Curr_pose;
        if(pose_compare(msg->poses[msg->poses.size()-1], temp_curr_pose, 0.3, 0.2))
          return;
        path = *msg;
        start_path = true;
        first_path = false;
      }
  }
  */

  void feedback_cb(const rover_actions::DriveToActionFeedback::ConstPtr& msg)
  {
    Curr_pose = msg->feedback.current_pose;
    if(debug_) ROS_ERROR_STREAM(Curr_pose);
  }

  float poses_dist(geometry_msgs::Pose P1,geometry_msgs::Pose P2)
  {
      float distant;
      double dx = P1.position.x - P2.position.x;
      double dy = P1.position.y - P2.position.y;
      distant = sqrt(pow(dx,2)+pow(dy,2));
      return distant;
  }

  geometry_msgs::Quaternion getYawFromPoses(geometry_msgs::PoseStamped P1,geometry_msgs::PoseStamped P2)
  {
     geometry_msgs::Quaternion out;
     double yaw;
     double dx = P2.pose.position.x - P1.pose.position.x;
     double dy = P2.pose.position.y - P1.pose.position.y;
     yaw = atan2(dy,dx);
     out = tf::createQuaternionMsgFromYaw(yaw);
     return out;
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
    tf::TransformListener listener;
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
    tf::TransformListener listener;
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

  void find_path_b()
  {
     double yaw = tf::getYaw(path.poses[0].pose.orientation);
     for(int i = 0;i<path.poses.size();i++)
     {
       if(i<path.poses.size()-1)
       yaw = atan2(path.poses[i+1].pose.position.x - path.poses[i].pose.position.x,
                   path.poses[i+1].pose.position.y - path.poses[i].pose.position.y);

       path.poses[i].pose.position.x += b_ * cos(yaw);
       path.poses[i].pose.position.y += b_ * sin(yaw);

     }
     path_pub.publish(path);
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

  void executeCB(const rover_actions::DriveToOAGoalConstPtr &Goal)  //goal = goal to DriveTo, Goal -> goal to DriveToOV
  {

    //ros::init(argc, argv, "test_path");

    tf::TransformListener listener;

    //invoking DriveTo action instant

    actionlib::SimpleActionClient<rover_actions::DriveToAction> ac_DriveTo("DriveTo", true);
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac_MoveBase("move_base", true);



    ROS_INFO("Waiting for DriveTo Action to come up");
    ac_DriveTo.waitForServer();
    ROS_INFO("Waiting for Movebase Action to come up");
    ac_MoveBase.waitForServer();
    ROS_INFO("Actions are available, Proceeding with Mission");

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

    //Sending goal to move_base for the path
    move_base_msgs::MoveBaseGoal goal_MoveBase;
    goal_MoveBase.target_pose.header.frame_id = "map";
    goal_MoveBase.target_pose.header.stamp = ros::Time::now();
    goal_MoveBase.target_pose.pose = Goal->goal_pose;
    ac_MoveBase.sendGoal(goal_MoveBase);
    ros::Duration(1.0).sleep();
    plan_time = ros::Time::now();

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
    ros::Rate r(1);
    wpstate = wait_;
    ros::Duration timeout(20);
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

      //Check for the success
      /*
      if(pose_compare(Curr_pose,Goal->goal_pose,0.1,0.05))
      {
          result_.result_pose = Curr_pose;
          ROS_INFO("Too close waypoint, Rover will not move");
          as_.setSucceeded(result_);
          return;
      }*/

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
      case wait_:
        if (debug_)ROS_WARN_THROTTLE(1,"waiting for a path");
        ros::spinOnce();

      break;
      case first_:
      {
           update_curr_pose();
           start_path = false;
           //ROS_INFO_STREAM("current pose: \n"<<Curr_pose<<"\n first wp \n"<<path.poses[wpid].pose);
           //ROS_WARN_STREAM(poses_dist(Curr_pose,path.poses[wpid].pose)<< "     " <<   fabs(b_));
           while (poses_dist(Curr_pose,path.poses[wpid].pose) < b_) {
             wpid++;

           }
           double goal_yaw = atan2(path.poses[wpid].pose.position.y-Curr_pose.position.y,
                                   path.poses[wpid].pose.position.x-Curr_pose.position.x);
           double curr_yaw = tf::getYaw(Curr_pose.orientation);
           goal.goal_pose = Curr_pose;
           goal.goal_pose.position.x += b_*cos(curr_yaw);
           goal.goal_pose.position.y += b_*sin(curr_yaw);
           goal.goal_pose.position.z = -100.0;
           goal.goal_pose.orientation = tf::createQuaternionMsgFromYaw(goal_yaw);
           ROS_INFO("Sending first goal, path size %d and wpid: %d",(int)path.poses.size(),wpid);
           ROS_WARN_STREAM(goal.goal_pose);
           //goal.goal_pose = Curr_pose;


           //goal.goal_pose.position.x -= b_*cos(yaw);
           //goal.goal_pose.position.y -= b_*sin(yaw);
           //goal.goal_pose.position.z = -100.0;
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
        /*
        int wpid0 = wpid-1;
        double th0 = atan2(path.poses[wpid].pose.position.y - path.poses[wpid0].pose.position.y,
                           path.poses[wpid].pose.position.x - path.poses[wpid0].pose.position.x);
        double th = th0;
        while (abs(th-th0)*M_PI/180 < 1.0)
        {
          wpid++;
          th = atan2(path.poses[wpid].pose.position.y - path.poses[wpid0].pose.position.y,
                     path.poses[wpid].pose.position.x - path.poses[wpid0].pose.position.x);
          if (wpid == (int)path.poses.size()-2)
          {
            break;
          }
        }
        ROS_WARN("index %d to %d are ignored",wpid0,wpid);

        geometry_msgs::Pose tmp_pose = path.poses[wpid].pose;

        if(is_far_enough(tmp_pose))
        {
          ROS_WARN("Sending Middle waypoint");


          goal.goal_pose = tmp_pose;
          ac_DriveTo.sendGoal(goal);
          if(!ac_DriveTo.waitForResult(timeout))
          {
            ROS_ERROR("catching ordinary goal didn't finished before the timeout");
            ac_DriveTo.cancelAllGoals();
            wpstate = failed;
            break;
          }
        }
        if (wpid < path.poses.size())
          wpid++;
        else
          wpstate = last_;*/
        while(wpid < path.poses.size()-1)
        {
            geometry_msgs::Vector3 RLC_msg = BodyErrorMsg(path.poses[wpid].pose);
            donkey_rover::Speed_control d_crl;
            d_crl.header.stamp = ros::Time::now();
            d_crl.CMD = false;
            d_crl.RLC = true;
            d_crl.JOY = true;
            if (RLC_msg.x < linear_threshold*3 && RLC_msg.y < linear_threshold*3 && wpid<path.poses.size()-2)
            {
              ROS_WARN("sending midd way point!");
              wpid++;

            }
            if(wpid == path.poses.size()-2 && RLC_msg.x < linear_threshold && RLC_msg.y < linear_threshold)
            {
              ROS_INFO("second to the last way point achieved!");
              wpid++;
            }
            speed_ctrl_pub.publish(d_crl);
            body_error_pub.publish(RLC_msg);
            ros::Duration(1/50).sleep();
        }
        //stoping
        stop();
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

  ros::NodeHandle nh_;
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

  geometry_msgs::Pose Curr_pose;
  nav_msgs::Path path;
  bool first_path;
  bool start_path;
  bool new_goal;
  ros::Time plan_time;
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
