#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
//#include <tf/Matrix3x3.h>
#include <rover_actions/DriveToAction.h>
#include <pcl/point_cloud.h> 
#include <donkey_rover/Speed_control.h>

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
  }

  ~DriveToAction(void)
  {
  }

  bool PoseCompare2D(geometry_msgs::Pose Current, geometry_msgs::Pose Gole)
  {
      bool out = false;
      float linear_threshold = 0.1;
      float angular_threshold = 0.1;
      if (fabs(Current.position.x - Gole.position.x)<linear_threshold
      &&  fabs(Current.position.y - Gole.position.y)<linear_threshold)
      {
            double roll, pitch, yaw_G,yaw_C;
            tf::Quaternion tf_q(Gole.orientation.x,Gole.orientation.y,Gole.orientation.z,Gole.orientation.w);
            tf::Matrix3x3(tf_q).getRPY(roll, pitch, yaw_G);
            tf::Quaternion tf_q2(Current.orientation.x,Current.orientation.y,Current.orientation.z,Current.orientation.w);
            tf::Matrix3x3(tf_q2).getRPY(roll, pitch, yaw_C);
            if(fabs(yaw_C-yaw_G) < angular_threshold) out = true;
      }
      return out;

  }

  geometry_msgs::Pose PoseFromTfTransform(tf::StampedTransform t)
  {
     geometry_msgs::Pose output;
     output.position.x =  t.getOrigin().getX();
     output.position.y =  t.getOrigin().getY();
     output.position.z =  t.getOrigin().getZ();
     tf::quaternionTFToMsg (t.getRotation(),output.orientation);
  }

  geometry_msgs::Vector3 BodyErrorMsg(float Tx, float Ty,float yaw, geometry_msgs::Pose Goal,tf::Transform T)
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


      donkey_rover::Speed_control d_crl;
      d_crl.CMD = false;
      d_crl.RLC = true;
      d_crl.JOY = true;
      speed_ctrl_pub.publish(d_crl);
      return Output;
  }

  void executeCB(const rover_actions::DriveToGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(10);
    bool success = false;
    bool transform_exists = false;
    tf::TransformListener listener;
    geometry_msgs::Vector3 Stop;
    Stop.x = 0.0;
    Stop.y = 0.0;
    Stop.z = 0.0;    


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
        //Move
        geometry_msgs::Vector3 msg = BodyErrorMsg(transform.getOrigin().getX(),   transform.getOrigin().getY() ,
                                                  tf::getYaw(transform.getRotation()),goal->goal_pose,transform);
        body_error_pub.publish(msg);
        //ROS_INFO_STREAM(msg);
        // check that preempt has not been requested by the client
        if (as_.isPreemptRequested() || !ros::ok())
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
        // check for success
        if(PoseCompare2D(goal->goal_pose,feedback_.current_pose)) 
        {
            success = true;
            body_error_pub.publish(Stop);
            ROS_INFO("Goal Achieved");
        }
        // publish the feedback
        as_.publishFeedback(feedback_);

        // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
        r.sleep();
    }
    if (!ros::ok())
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
  // The topic to be published to make the rover move
  // Trun in place situation has to be think of
  ros::Publisher body_error_pub;
  ros::Publisher speed_ctrl_pub;
  // NodeHandle instance must be created before this line. Otherwise strange error may occur.
  actionlib::SimpleActionServer<rover_actions::DriveToAction> as_; 
  std::string action_name_;
  // create messages that are used to published feedback/result
  rover_actions::DriveToFeedback feedback_;
  rover_actions::DriveToResult result_;


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "DriveTo");

  DriveToAction DriveTo(ros::this_node::getName());
  ros::spin();

  return 0;
}
