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
    //Initializers
    // Global params
    b_ = 0.4;
    b_thr_ = 0.2;
    sub_goal_distance = 0.3;
    debug_ = true;

    //Global varriables

    first_path = true;
    start_path = false;
    new_goal = true;
  }

  ~DriveToOAAction(void)
  {
  }

  void CheckBodyError(geometry_msgs::Pose Current_pose, geometry_msgs::Pose Goal,bool is_last_sub_goal ,bool is_first_sub_goal,int8_t& command)
  {
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
        command = 1;
      else
        command = 2;
      if (is_last_sub_goal) // -100.00 is last point flag
        command = 3;
      if(is_first_sub_goal && new_goal) {
        command = 4;
        if(debug_) ROS_INFO("Yaw: %f",yaw);
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
        if(debug_) ROS_INFO("New Path to the Old Goal");
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
      if(first_path)
      {
         path = *msg;
         new_goal = true;
         start_path = true;
         first_path = false;
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
        ROS_FATAL("transform from base_link to map does not exist, rover_actions failed :-( ");
        return;
    }

    //Sending goal to move_base for the path
    move_base_msgs::MoveBaseGoal goal_MoveBase;
    goal_MoveBase.target_pose.header.frame_id = "base_link";
    goal_MoveBase.target_pose.header.stamp = ros::Time::now();
    goal_MoveBase.target_pose.pose = Goal->goal_pose;
    ac_MoveBase.sendGoal(goal_MoveBase);


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
    ros::Rate r(10);
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

      if(pose_compare(Curr_pose,Goal->goal_pose,0.1,0.05))
      {
          result_.result_pose = Curr_pose;
          ROS_INFO("Too close waypoint, Rover will not move");
          as_.setSucceeded(result_);
          return;
      }

      //Check for the preempt request
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        as_.setPreempted();
        break;
      }

      if(start_path)
      {
        if(debug_) ROS_INFO_STREAM("Path size:   "<< path.poses.size());
        for(int i=0; i < path.poses.size();i++)
        {
            //Check for the preempt request
            if (as_.isPreemptRequested() || !ros::ok())
            {
               ROS_INFO("%s: Preempted", action_name_.c_str());
               as_.setPreempted();
               break;
            }

            geometry_msgs::PoseStamped temp_goal = path.poses[i];

            int8_t command;
            bool is_first_sub_goal = false;
            if(i == 0)
              is_first_sub_goal = true;

            bool is_last_sub_goal = false;
            if(i==(path.poses.size()-1))
              is_last_sub_goal = true;
            //rover_actions::DriveToFeedback feedback_ = ac.ActionFeedback;

            CheckBodyError(Curr_pose, temp_goal.pose, is_last_sub_goal ,is_first_sub_goal,command);
            switch (command) {// Command : 1 = ignor, 2 = Catch, 3 = Catch and Turn
            case 1:
            {
              if(debug_) ROS_WARN("Index %d ingnored",i);
              continue;
            }
              break;
            case 2:
            {
              goal.goal_pose = temp_goal.pose;
              if(debug_) ROS_WARN("index %d  sub goal x:%f   y:%f  ",i,goal.goal_pose.position.x,goal.goal_pose.position.y);
              ac_DriveTo.sendGoal(goal);

              //wait for the action to return
              bool finished_before_timeout = ac_DriveTo.waitForResult(ros::Duration(30.0));

              if (finished_before_timeout)
              {
                  actionlib::SimpleClientGoalState state = ac_DriveTo.getState();
                  if(debug_) ROS_INFO("Action finished: %s",state.toString().c_str());
              }
              else
                  if(debug_) ROS_INFO("Action did not finish before the time out.");

            }
              break;
            case 3:
            {
              goal.goal_pose = temp_goal.pose;
              //goal.goal_pose.position.z = -100.00;
              if(debug_) ROS_WARN("index %d  sub goal x:%f   y:%f  ",i,goal.goal_pose.position.x,goal.goal_pose.position.y);
              if(debug_) ROS_INFO("Last Goal");
              ac_DriveTo.sendGoal(goal);

              //wait for the action to return
              bool finished_before_timeout = ac_DriveTo.waitForResult(ros::Duration(30.0));

              if (finished_before_timeout)
              {
                  actionlib::SimpleClientGoalState state = ac_DriveTo.getState();
                  if(debug_) ROS_INFO("Action finished: %s",state.toString().c_str());
                  goal.goal_pose = last_pose_shift(goal.goal_pose,b_);
                  if(debug_) ROS_INFO("Final phase");
                  ac_DriveTo.sendGoal(goal);
                  if(ac_DriveTo.waitForResult(ros::Duration(30.0)))
                  {
                    if(debug_) ROS_INFO("PATH ACHIEVED SUCCESSFULLY");
                    result_.result_pose = Curr_pose;
                    ROS_INFO("Goal Achieved");
                    as_.setSucceeded(result_);
                    return;
                  }

              }
              else
                  if(debug_) ROS_INFO("Action did not finish before the time out.");
            }
              break;
            case 4:
            {
              if(debug_) ROS_INFO("Executing the first sub Goal with yaw %f   ", (float) tf::getYaw(temp_goal.pose.orientation));
              goal.goal_pose.position = Curr_pose.position;
              goal.goal_pose.position.z = -100.00;  //Turn in place signal
              //goal.goal_pose.orientation = temp_goal.pose.orientation; //should be corrected
              goal.goal_pose.orientation = getYawFromPoses(path.poses[0],path.poses[1]);
              ac_DriveTo.sendGoal(goal);

              //wait for the action to return
              bool finished_before_timeout = ac_DriveTo.waitForResult(ros::Duration(30.0));

              if (finished_before_timeout)
              {
                  actionlib::SimpleClientGoalState state = ac_DriveTo.getState();
                  if(debug_) ROS_INFO("Action finished: %s",state.toString().c_str());
              }
              else
                  if(debug_) ROS_INFO("Action did not finish before the time out.");
            }
              break;

            default:
              if(debug_) ROS_ERROR("unexpected command, bad implementation correct the code!!!");
              break;
            }

      }
         start_path = false;
      }

      ros::spinOnce();
      r.sleep();
      //exit
    }

  }

protected:

  ros::NodeHandle nh_;

  actionlib::SimpleActionServer<rover_actions::DriveToOAAction> as_;
  std::string action_name_;

  //geometry_msgs::Pose current_pose;
  //geometry_msgs::Pose sub_goal_pose;
  ros::Subscriber sub_from_path;
  rover_actions::DriveToOAFeedback feedback_;
  rover_actions::DriveToOAResult result_;
  float b_;
  float b_thr_;
  std::string map_frame_id;

  double sub_goal_distance;
  bool debug_;

  geometry_msgs::Pose Curr_pose;
  nav_msgs::Path path;
  bool first_path;
  bool start_path;
  bool new_goal;
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "DriveToOA");

  DriveToOAAction DriveToOA(ros::this_node::getName());

  ros::spin();

  return 0;
}
