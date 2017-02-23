#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
//#include <tf/Matrix3x3.h>
#include <rover_actions/DriveToAction.h>
#include <pcl/point_cloud.h> 
#include <donkey_rover/Speed_control.h>
#include <geometry_msgs/Twist.h>

int SGN(double x)
{
  if (x>=0) return 1;
  return -1;
}
double SAT(double x,double X)
{
  X = fabs(X);
  if (x >= 0)
     return std::min(x,X*SGN(x));
  return std::max(x,X*SGN(x));
}

enum state
{MOVE, TURN, DONE};
using namespace Eigen;

class DriveToAction
{


public:

  DriveToAction(std::string name) :
    as_(nh_, name, boost::bind(&DriveToAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
    body_error_pub = nh_.advertise<geometry_msgs::Vector3>(ros::this_node::getNamespace()+"/body_error",1);
    speed_ctrl_pub = nh_.advertise<donkey_rover::Speed_control>(ros::this_node::getNamespace()+"/speed_control",1);
    cmd_vel_pub    = nh_.advertise<geometry_msgs::Twist>(ros::this_node::getNamespace()+"/cmd_vel",1);
    //initializers
    Status = 3;
    vicinity = false;
    //b_ = 0.4;
    b_thr_ = 0.2;
    rate = 10;
    omega_sgn = 1;
  }

  ~DriveToAction(void)
  {
    ROS_WARN("DriveTO destructed");
  }

  bool PoseCompare2D(geometry_msgs::Pose Current, geometry_msgs::Pose Goal)
  {
      bool out = false;
      //ROS_INFO_STREAM("current pose   \n"<<Current);
      //ROS_WARN_STREAM("Goal pose      \n"<<Goal);

      if (fabs(Current.position.x - Goal.position.x)<linear_threshold
      &&  fabs(Current.position.y - Goal.position.y)<linear_threshold)
      {

            ROS_INFO("linear reached");
            double yaw_G,yaw_C;
            tf::Quaternion tf_q(Goal.orientation.x,Goal.orientation.y,Goal.orientation.z,Goal.orientation.w);
            yaw_G = tf::getYaw(tf_q);
            tf::Quaternion tf_q2(Current.orientation.x,Current.orientation.y,Current.orientation.z,Current.orientation.w);
            yaw_C = tf::getYaw(tf_q2);
            ROS_WARN_STREAM("current yaw:  "<<yaw_C<<"     goal yaw:   "<<yaw_G);
            if(fabs(yaw_C-yaw_G) < angular_threshold)
            {
              out = true;
              STAT = DONE;
            }
            else
            {
                 vicinity = true;
                 //Status = 2;
                 if(round(Goal.position.z) == -100.00)
                   Status = TURN;
                 else
                   out = true;
            }
            return out;
      }

      return out;

  }

  bool angle_reached(geometry_msgs::Pose Current, geometry_msgs::Pose Goal)
  {
      double yaw_G,yaw_C;
      tf::Quaternion tf_q(Goal.orientation.x,Goal.orientation.y,Goal.orientation.z,Goal.orientation.w);
      yaw_G = tf::getYaw(tf_q);
      tf::Quaternion tf_q2(Current.orientation.x,Current.orientation.y,Current.orientation.z,Current.orientation.w);
      yaw_C = tf::getYaw(tf_q2);

      if(fabs(yaw_C-yaw_G) < angular_threshold)
      {
          return true;
      }
      else return false;
  }

  bool Must_turn(geometry_msgs::Pose p)
  {
    if (round(p.position.z + 100) == 0) return true;
    return false;
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

  geometry_msgs::Vector3 BodyErrorMsg(tf::StampedTransform transform, geometry_msgs::Pose Goal, int8_t& command)
  {
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
      //ros::param::get("/controler/distance_b",b_);
      G_ATR(0) -= fabs(b_);
      if (b_ < 0) G_ATR(0) -= fabs(b_);
      //ROS_ERROR("dist b is fucking    %f",b_);
      // --- Decision
      if (G_ATR(0) > linear_threshold) // < b_thr
        command = 1;
      else
        command = 2;
      if (round(Goal.position.z) == -100.00) // -100.00 is last point flag
        command = 3;

      geometry_msgs::Vector3 Output;
      Output.x = G_ATR(0);
      Output.y = G_ATR(1);
      //Defining controller Gain
      if(adaptive_gain)
        Output.z = 1/sqrt(pow(G_ATR(0),2)+pow(G_ATR(1),2));
      else
        Output.z = 0.0;

      return Output;
  }

  void executeCB(const rover_actions::DriveToGoalConstPtr &goal)
  {
    Status = 1; //Default Status - Move
    STAT = MOVE;
    ros::Rate r(rate);
    bool success = false;
    bool transform_exists = false;
    tf::TransformListener listener;

    double yaw_G,yaw_C;
    tf::Quaternion tf_q(goal->goal_pose.orientation.x,goal->goal_pose.orientation.y,goal->goal_pose.orientation.z,goal->goal_pose.orientation.w);
    yaw_G = tf::getYaw(tf_q);
    geometry_msgs::Vector3 Stop;
    Stop.x = 0.0;
    Stop.y = 0.0;
    Stop.z = 0.0;

    tf_is_valid = true;

    //Reading the paameters
    ros::NodeHandle npr("~");
    if(!npr.getParam("Linear_error",linear_threshold))
    {
      linear_threshold = 0.42;
      ROS_WARN("No value is received for Ainear error, it is set to default value %f",linear_threshold);
    }
    if(!npr.getParam("Angular_error",angular_threshold))
    {
      angular_threshold = 0.1;
      ROS_WARN("No value is received for Angular error, it is set to default value %f",angular_threshold);
    }
    if(!npr.getParam("Trun_in_place_speed",omega))
    {
      omega = 0.5;
      ROS_WARN("No value is received for Turn in place speed, it is set to default value %f",omega);
    }
    if(!npr.getParam("Adaptive_controller_gain",adaptive_gain))
    {
      adaptive_gain = false;
      ROS_WARN("No value is received for Adaptive controller gain, it is set to default value %d",adaptive_gain);
    }
    double P_ey = 0.3;
    double I_ey = 0.01;
    double D_ey = -0.1;
    double K_Move = 0.6;
    ROS_WARN_COND(!npr.getParam("Turn_ctlr_P",P_ey),"No value is received for Turn_ctlr_P default:%f",P_ey);
    ROS_WARN_COND(!npr.getParam("Turn_ctlr_I",I_ey),"No value is received for Turn_ctlr_I default:%f",I_ey);
    ROS_WARN_COND(!npr.getParam("Turn_ctlr_P",D_ey),"No value is received for Turn_ctlr_D default:%f",D_ey);
    ROS_WARN_COND(!npr.getParam("Move_ctlr_K",K_Move),"No value is received for Move_ctlr_K default:%f",K_Move);
    ros::param::set(ros::this_node::getNamespace()+"/controler/Controller_Gain",K_Move);
    ros::param::set(ros::this_node::getNamespace()+"/controler/Tracking_precision",linear_threshold*0.01);
    b_ = 0.4;
    ros::param::get("/controler/distance_b",b_);
    ROS_INFO("DriverTo: b : %f ",b_);
    omega_orig = omega;
    donkey_rover::Speed_control d_crl;

    feedback_.current_pose.position.x = 0.0;
    feedback_.current_pose.position.y = 0.0;
    feedback_.current_pose.position.z = 0.0;
    feedback_.current_pose.orientation.w = 1.0;
    feedback_.current_pose.orientation.x = 0.0;
    feedback_.current_pose.orientation.y = 0.0;
    feedback_.current_pose.orientation.z = 0.0;

    transform_exists = listener.waitForTransform("/map", "base_link", ros::Time(0), ros::Duration(3));
    if(!transform_exists)
    {
        ROS_FATAL("transform from base_link to map does not exist, rover_actions failed :-( ");
        return;
    }
    // start executing the action
    double e_yaw,e_yaw_0;
    e_yaw_0 = 0;

    double e_yaw_acc = 0;
    while(nh_.ok())
    {
        tf::StampedTransform transform;
        
        try{
            listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);

        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            tf_is_valid = false;
        }

        
        feedback_.current_pose = PoseFromTfTransform(transform);
        //Case Far from the Goal - Move
        switch (STAT)
        {
            case MOVE:   //MOVE TO CATCH THE GOAL
            {
                ROS_WARN("Move");
                int8_t command;
                geometry_msgs::Vector3 RLC_msg = BodyErrorMsg(transform,goal->goal_pose,command); 
                d_crl.header.stamp = ros::Time::now();
                d_crl.CMD = false;
                d_crl.RLC = true;
                d_crl.JOY = true;
                speed_ctrl_pub.publish(d_crl);
                body_error_pub.publish(RLC_msg);
                // decision
                if (fabs(RLC_msg.x) < linear_threshold && fabs(RLC_msg.y) < linear_threshold)
                {
                  if (Must_turn(goal->goal_pose)) STAT = TURN;
                  else STAT = DONE;
                }
            }
            break;

            case TURN:    //TURN IN PLACE TO SATISFY FINAL POSE
            {
               ROS_WARN("Turn");
               yaw_C = tf::getYaw(transform.getRotation()) + M_PI;
               tf::Quaternion tf_q2(goal->goal_pose.orientation.x,goal->goal_pose.orientation.y,goal->goal_pose.orientation.z,goal->goal_pose.orientation.w);
               yaw_G = tf::getYaw(tf_q2) + M_PI;
               e_yaw = yaw_G - yaw_C;
               e_yaw_acc += e_yaw;
               //Slowing down
               float norm_dyaw = fabs(yaw_C - yaw_G)/(2*M_PI);
               if (norm_dyaw < 0.5 && fabs(omega) > fabs(omega_orig*0.6))
               {
                 omega = 0.5 * omega_orig;
                 //ROS_INFO("omega: %f norm delta yaw: %f",omega,norm_dyaw);
               }
               geometry_msgs::Twist CMD_msg;
               CMD_msg.linear.x = 0;CMD_msg.linear.y = 0;CMD_msg.linear.z = 0;
               CMD_msg.angular.x = 0;CMD_msg.angular.y = 0;


               if((yaw_G < yaw_C || yaw_G > yaw_C + M_PI) && tf_is_valid)
               {
                  omega_sgn = -1;
                  tf_is_valid = false;
               }
               else if(tf_is_valid)
               {
                  omega_sgn = 1;
                  tf_is_valid = false;
               }
               //CMD_msg.angular.z = omega_sgn*omega;
               CMD_msg.angular.z = SAT(P_ey*e_yaw + I_ey*e_yaw_acc + D_ey*(e_yaw-e_yaw_0),omega);
               d_crl.header.stamp = ros::Time::now();
               d_crl.CMD = true;
               d_crl.RLC = false;
               d_crl.JOY = true;
               speed_ctrl_pub.publish(d_crl);
               cmd_vel_pub.publish(CMD_msg);
               e_yaw_0 = e_yaw;
               //decision
               if (angle_reached(feedback_.current_pose,goal->goal_pose))
                 STAT = DONE;
            }
            break;

            case DONE:
            {
               ROS_INFO("Goal Achieved");
               for(int i=0;i<rate;i++)
               {
                 d_crl.CMD = false;
                 d_crl.RLC = true;
                 d_crl.JOY = true;
                 speed_ctrl_pub.publish(d_crl);
                 body_error_pub.publish(Stop);
                 r.sleep();
               }

               d_crl.header.stamp = ros::Time::now();
               d_crl.CMD = false;
               d_crl.RLC = false;
               d_crl.JOY = true;
               speed_ctrl_pub.publish(d_crl);

            }
            success = true;
            break;

            default:
              ROS_ERROR("UNKOWN STATUS");
            break;
        }

        // check for success
        if(success)
        {
            body_error_pub.publish(Stop);
            ROS_INFO("Goal Achieved!!!!!!!!!!!!");
            result_.result_pose = feedback_.current_pose;
            as_.setSucceeded(result_);
            tf_is_valid = true;
            break;

        }

        as_.publishFeedback(feedback_);


        if (as_.isPreemptRequested())
        {
            ROS_INFO("%s: Preempted", action_name_.c_str());
            // set the action state to preempted
            as_.setPreempted();
            success = false;
            for(int i = 0;i< 10;i++)
            {
              body_error_pub.publish(Stop);
              r.sleep();
            }
            break;
        }
        ros::spinOnce();
        r.sleep();
    }
    if (!nh_.ok())
    {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
        for(int i = 0;i< 10;i++)
        {
          body_error_pub.publish(Stop);
          r.sleep();
        }
    }

  }

protected:
  int rate;
  ros::NodeHandle nh_;
  ros::Publisher body_error_pub;
  ros::Publisher speed_ctrl_pub;
  ros::Publisher cmd_vel_pub;

  actionlib::SimpleActionServer<rover_actions::DriveToAction> as_; 
  std::string action_name_;

  rover_actions::DriveToFeedback feedback_;
  rover_actions::DriveToResult result_;
  double b_;
  float b_thr_;

private:
  int sgn(float x)
  {
    int out = 1;
    if(x < 0) out = -1;
    return out;
  }
  bool vicinity;
  int Status;   // 1- Move    2- Trun in place    3- Reached
  state STAT;
  double linear_threshold;
  double angular_threshold;
  double omega; //Trun in place
  double omega_orig;
  bool adaptive_gain;
  bool tf_is_valid;
  int omega_sgn;

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "DriveTo");

  DriveToAction DriveTo(ros::this_node::getName());//getName
  ros::spin();

  return 0;
}
