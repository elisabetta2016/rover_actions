#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
//#include <tf/Matrix3x3.h>
#include <rover_actions/DriveToAction.h>
#include <pcl/point_cloud.h> 

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

  geometry_msgs::Vector3 BodyErrorMsg(float Tx, float Ty,float yaw, geometry_msgs::Pose Goal)
  {
      Matrix3f Trans;
      Trans(0,0) = cos(yaw);  Trans(0,1) = -sin(yaw); Trans(0,2) = -Tx;
      Trans(1,0) = sin(yaw);  Trans(1,1) =  cos(yaw); Trans(1,2) = -Ty;
      Trans(2,0) = 0.0;       Trans(2,1) =  0.0;      Trans(3,3) = 1.0;

      Vector3f G_IF;
      G_IF(0) = Goal.position.x;
      G_IF(1) = Goal.position.y;
      G_IF(2) = 1.0;

      Vector3f G_BF;
      G_BF = Trans*G_IF;
      geometry_msgs::Vector3 Output;
      Output.x = G_BF(0);
      Output.y = G_BF(1);
      Output.y = G_BF(0);
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
        body_error_pub.publish(BodyErrorMsg(transform.getOrigin().getX(),   transform.getOrigin().getY() ,
                                            tf::getYaw(transform.getRotation()),       goal->goal_pose));
        // check that preempt has not been requested by the client
        if (as_.isPreemptRequested() || !ros::ok())
        {
            ROS_INFO("%s: Preempted", action_name_.c_str());
            // set the action state to preempted
            as_.setPreempted();
            success = false;
            body_error_pub.publish(Stop);
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

  }

protected:

  ros::NodeHandle nh_;
  // The topic to be published to make the rover move
  // Trun in place situation has to be think of
  ros::Publisher body_error_pub;
  // NodeHandle instance must be created before this line. Otherwise strange error may occur.
  actionlib::SimpleActionServer<rover_actions::DriveToAction> as_; 
  std::string action_name_;
  // create messages that are used to published feedback/result
  rover_actions::DriveToFeedback feedback_;
  rover_actions::DriveToResult result_;


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "DriveToAct");

  DriveToAction DriveToAct(ros::this_node::getName());
  ros::spin();

  return 0;
}