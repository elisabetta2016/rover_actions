#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
//#include <tf/Matrix3x3.h>
#include <rover_actions/DriveToAction.h>
#include <pcl/point_cloud.h> 
#include <donkey_rover/Speed_control.h>
#include <geometry_msgs/Twist.h>


using namespace Eigen;

class DriveToAction
{


public:

  DriveToAction(std::string name) :
    as_(nh_, name, boost::bind(&DriveToAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
    body_error_pub = nh_.advertise<geometry_msgs::Vector3>("/body_error",1);
    speed_ctrl_pub = nh_.advertise<donkey_rover::Speed_control>("/speed_control",1);
    cmd_vel_pub    = nh_.advertise<geometry_msgs::Twist>("/cmd_vel",1);
    Status = 3;
    vicinity = false;
  }

  ~DriveToAction(void)
  {
  }

  bool PoseCompare2D(geometry_msgs::Pose Current, geometry_msgs::Pose Gole)
  {
      bool out = false;
      float linear_threshold = 0.8;
      float angular_threshold = 0.1;
      //ROS_INFO_STREAM("current pose   \n"<<Current);
      //ROS_WARN_STREAM("gole pose      \n"<<Gole);

      if (fabs(Current.position.x - Gole.position.x)<linear_threshold
      &&  fabs(Current.position.y - Gole.position.y)<linear_threshold)
      {

            ROS_INFO("linear reached");
            double roll, pitch, yaw_G,yaw_C;
            tf::Quaternion tf_q(Gole.orientation.x,Gole.orientation.y,Gole.orientation.z,Gole.orientation.w);
            yaw_G = tf::getYaw(tf_q);
            //tf::Matrix3x3(tf_q).getRPY(roll, pitch, yaw_G);
            tf::Quaternion tf_q2(Current.orientation.x,Current.orientation.y,Current.orientation.z,Current.orientation.w);
            yaw_C = tf::getYaw(tf_q2);
            //tf::Matrix3x3(tf_q2).getRPY(roll, pitch, yaw_C);
            ROS_WARN_STREAM("current yaw:  "<<yaw_C<<"     goal yaw:   "<<yaw_G);
            if(fabs(yaw_C-yaw_G) < angular_threshold)
            {
              out = true;
              Status = 3;
            }
            else
            {
                 vicinity = true;
                 Status = 2;
                 /*
                 tf::TransformListener listener;
                 tf::StampedTransform transform;



                 ROS_INFO("Trun in Place");
                 ros::Rate r(10);
                 int curr_sgn = sgn(yaw_C-yaw_G);
                 int last_sgn = curr_sgn;
                 float omega = 0.5;
                 while(fabs(yaw_C-yaw_G) > angular_threshold && nh_.ok() )
                 {
                     try{
                         listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
                     }
                     catch (tf::TransformException ex){
                         ROS_ERROR("%s",ex.what());
                         ros::Duration(1.0).sleep();

                     }
                     yaw_C = tf::getYaw(transform.getRotation());
                     geometry_msgs::Twist msg;
                     msg.linear.x = 0;msg.linear.y = 0;msg.linear.z = 0;
                     msg.angular.x = 0;msg.angular.y = 0;
                     msg.angular.z = omega;

                     curr_sgn = sgn(yaw_C-yaw_G);
                     if (curr_sgn != last_sgn)
                     {
                       msg.angular.z = 0.0;
                       cmd_vel_pub.publish(msg);
                       ROS_WARN("Too fast turn!!!");
                       break;
                     }
                     donkey_rover::Speed_control d_crl;
                     d_crl.CMD = true;
                     d_crl.RLC = false;
                     d_crl.JOY = true;
                     speed_ctrl_pub.publish(d_crl);
                     cmd_vel_pub.publish(msg);
                     last_sgn = curr_sgn;
                     r.sleep();
                 }*/

            }
            return out;
      }

      //Case linear Threshold has been achieved:

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

  geometry_msgs::Vector3 BodyErrorMsg(float Tx, float Ty,float yaw, geometry_msgs::Pose Goal)
  {

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
      geometry_msgs::Vector3 Output;
      Output.x = G_ATR(0);
      Output.y = G_ATR(1);
      Output.z = 0.0;
      //ROS_INFO_STREAM(Output);
      //ROS_INFO_STREAM("X:"<<Result.getX()<<"    Y:  "<<Result.getY());



      return Output;
  }

  void executeCB(const rover_actions::DriveToGoalConstPtr &goal)
  {
    Status = 1; //Default Status - Move
    ros::Rate r(10);
    bool success = false;
    bool transform_exists = false;
    tf::TransformListener listener;
    float omega = 0.5;  //Turn in place speed
    double yaw_G,yaw_C;
    tf::Quaternion tf_q(goal->goal_pose.orientation.x,goal->goal_pose.orientation.y,goal->goal_pose.orientation.z,goal->goal_pose.orientation.w);
    yaw_G = tf::getYaw(tf_q);
    geometry_msgs::Vector3 Stop;
    Stop.x = 0.0;
    Stop.y = 0.0;
    Stop.z = 0.0;    

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
        
        
        feedback_.current_pose = PoseFromTfTransform(transform);
        //Case Far from the Goal - Move
        switch (Status)
        {
            case 1:
            {
                ROS_WARN("Move");
                geometry_msgs::Vector3 RLC_msg = BodyErrorMsg(transform.getOrigin().getX(),   transform.getOrigin().getY() ,
                                                  tf::getYaw(transform.getRotation()),goal->goal_pose);
                d_crl.CMD = false;
                d_crl.RLC = true;
                d_crl.JOY = true;
                speed_ctrl_pub.publish(d_crl);
                body_error_pub.publish(RLC_msg);
            }
            break;

            case 2:
            {
               ROS_WARN("Turn");
               yaw_C = tf::getYaw(transform.getRotation());
               geometry_msgs::Twist CMD_msg;
               CMD_msg.linear.x = 0;CMD_msg.linear.y = 0;CMD_msg.linear.z = 0;
               CMD_msg.angular.x = 0;CMD_msg.angular.y = 0;
               CMD_msg.angular.z = omega;
               d_crl.CMD = true;
               d_crl.RLC = false;
               d_crl.JOY = true;
               speed_ctrl_pub.publish(d_crl);
               cmd_vel_pub.publish(CMD_msg);
            }
            break;

            case 3:
            {
               ROS_INFO("Goal Achieved");
               d_crl.CMD = false;
               d_crl.RLC = false;
               d_crl.JOY = true;
               speed_ctrl_pub.publish(d_crl);
            }
            break;

            default:
              ROS_ERROR("UNKOWN STATUS");
            break;
        }


        // check for success
        if(PoseCompare2D(feedback_.current_pose,goal->goal_pose))
        {
            success = true;
            body_error_pub.publish(Stop);
            ROS_INFO("Goal Achieved");

        }
        //Case linear threshold has been achieved, angular not yet


        // publish the feedback
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

  ros::NodeHandle nh_;
  ros::Publisher body_error_pub;
  ros::Publisher speed_ctrl_pub;
  ros::Publisher cmd_vel_pub;

  actionlib::SimpleActionServer<rover_actions::DriveToAction> as_; 
  std::string action_name_;

  rover_actions::DriveToFeedback feedback_;
  rover_actions::DriveToResult result_;

private:
  int sgn(float x)
  {
    int out = 1;
    if(x < 0) out = -1;
    return out;
  }
  bool vicinity;
  int Status;   // 1- Move    2- Trun in place    3- Reached

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "DriveTo");

  DriveToAction DriveTo(ros::this_node::getName());
  ros::spin();

  return 0;
}
