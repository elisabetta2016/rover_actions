#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
//#include <tf/Matrix3x3.h>
#include <rover_actions/PathFollowerAction.h>
#include <pcl/point_cloud.h> 
#include <donkey_rover/Speed_control.h>
#include <geometry_msgs/Twist.h>
#include <rover_actions/DriveToAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
using namespace Eigen;

class PathFollowerAction
{


public:

  PathFollowerAction(std::string name) :
    as_(nh_, name, boost::bind(&PathFollowerAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
    b_ = 0.4;
    b_thr_ = 0.2;
    map_frame_id ="map";
  }

  ~PathFollowerAction(void)
  {
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

  void CheckBodyError(tf::StampedTransform transform, geometry_msgs::Pose Goal,bool last_sub_goal ,int8_t& command)
  {
      // Command : 1 = ignor, 2 = Catch, 3 = Catch and Turn
      float Tx = transform.getOrigin().getX();
      float Ty = transform.getOrigin().getY();
      float yaw = tf::getYaw(transform.getRotation());
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

  void executeCB(const rover_actions::PathFollowerGoalConstPtr &goal)
  {

    ros::Rate r(10);
    bool success = false;
    bool transform_exists = false;
    tf::TransformListener listener;


    transform_exists = listener.waitForTransform("/map", "base_link", ros::Time(0), ros::Duration(3));
    if(!transform_exists)
    {
        ROS_FATAL("transform from base_link to map does not exist, rover_actions failed :-( ");
        return;
    }
    // start executing the action
    tf::StampedTransform transform;

    int goal_path_size = goal->goal_path.poses.size();
    geometry_msgs::Pose final_pose = goal->goal_path.poses[goal_path_size-1].pose;

    actionlib::SimpleActionClient<rover_actions::DriveToAction> ac("DriveTo", true);

    int it = 0;
    bool last_sub_goal = false;
    ROS_INFO("it:%d   and   vector_size:%d",it,goal_path_size);
    while(nh_.ok() && it < goal_path_size)
    {
        
        try{
            listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            
        }


        ROS_INFO_ONCE("Waiting for action server to start.");
        ac.waitForServer();
        ROS_INFO("Action server started, sending goal.");


        current_pose = PoseFromTfTransform(transform);
        sub_goal_pose = goal->goal_path.poses[it].pose;
        ROS_WARN_STREAM(sub_goal_pose);
        int8_t command;

        if(it == goal_path_size) last_sub_goal = true;
        CheckBodyError(transform,sub_goal_pose,last_sub_goal,command);

        switch(command)
        {
          case 1: //ignor the subgoal
          {
              it++;
              continue;
          }

          case 2:  //catch the subgoal
          {
             rover_actions::DriveToGoal goal;
             goal.goal_pose = sub_goal_pose;
             ac.sendGoal(goal);
             //wait for the action to return
             bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

             if (finished_before_timeout)
             {
               //actionlib::SimpleClientGoalState state = ac.getState();
               ROS_INFO("sub goal number %d achieved",it);
               geometry_msgs::PoseStamped temp;
               temp.header.stamp = ros::Time::now();
               temp.header.frame_id = map_frame_id;
               temp.pose = sub_goal_pose;
               feedback_.feedback_path.poses.push_back(temp);
               it++; //go to next subgoal
             }
             else
               ROS_WARN("Subgoal was not achieved before the timeout");
           }
           case 3:
           {
              rover_actions::DriveToGoal goal;
              goal.goal_pose = sub_goal_pose;
              ac.sendGoal(goal);
              //wait for the action to return
              bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

              if (finished_before_timeout)
              {
                //actionlib::SimpleClientGoalState state = ac.getState();
                ROS_INFO("sub goal number %d achieved",it);
                geometry_msgs::PoseStamped temp;
                temp.header.stamp = ros::Time::now();
                temp.header.frame_id = map_frame_id;
                temp.pose = sub_goal_pose;
                feedback_.feedback_path.poses.push_back(temp);
                goal.goal_pose.position.z = -100.0; //Turn in place signal
                ac.sendGoal(goal);
                bool turn_in_place_success = ac.waitForResult(ros::Duration(30.0));
                result_.result_pose = current_pose;
                as_.setSucceeded(result_);
                it++; //Exit while loop
                if(turn_in_place_success)
                {
                  ROS_INFO("path_achieved!!!");
                }
                else
                {
                  ROS_WARN("Path achieved although goal pose did not achieved");

                }
              }
              else
                ROS_WARN("Subgoal was not achieved before the timeout");
            }
        }



        // publish the feedback
        as_.publishFeedback(feedback_);


        if (as_.isPreemptRequested())
        {
            ROS_WARN("%s: Preempted", action_name_.c_str());
            // set the action state to preempted
            as_.setPreempted();
            success = false;
            break;
        }
        ros::spinOnce();
        r.sleep();

    }
    if (!nh_.ok())
    {
        ROS_WARN("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
    }

  }

protected:

  ros::NodeHandle nh_;

  actionlib::SimpleActionServer<rover_actions::PathFollowerAction> as_;
  std::string action_name_;
  geometry_msgs::Pose current_pose;
  geometry_msgs::Pose sub_goal_pose;

  rover_actions::PathFollowerFeedback feedback_;
  rover_actions::PathFollowerResult result_;
  float b_;
  float b_thr_;
  std::string map_frame_id;

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "PathFollower");

  PathFollowerAction PathFollower(ros::this_node::getName());

  ros::spin();

  return 0;
}
