#include "ros/ros.h"
#include <sensor_msgs/Joy.h>
#include <tf/transform_broadcaster.h>

#include <planning_msgs/publishPointsInBBX.h>
#include <repelling_velocity_msgs/repelling_velocity.h>

#include <asctec_hl_comm/mav_ctrl.h>

class SubscribeAndPublish
{
public:

  ros::Rate rate_;

  SubscribeAndPublish(ros::NodeHandle& nh)
    : rate_(10.0), // [Hz]
      world_frame_("world"),
      base_frame_("fcu"),
      lin_velocity_(0.05), // [m/s]
      z_gain_(0.1),        // [1/s]
      ang_velocity_(0.06), // [rad/s]
      first_(true)
	{
    last_repelling_velocity_.x = 0.0;
    last_repelling_velocity_.y = 0.0;
    last_repelling_velocity_.z = 0.0;

    // Parameters
    ros::NodeHandle private_nh("~");
    double rate = 1.0/rate_.expectedCycleTime().toSec(); //default value
    private_nh.param("rate",rate,rate);
    rate_ = ros::Rate(rate);
    private_nh.param("world_frame",world_frame_,world_frame_);
    private_nh.param("base_frame",base_frame_,base_frame_);
    private_nh.param("lin_velocity",lin_velocity_,lin_velocity_);
    private_nh.param("z_gain",z_gain_,z_gain_);
    private_nh.param("ang_velocity",ang_velocity_,ang_velocity_);

    // Scale velocities with loop rate
    lin_velocity_ /= rate;
    z_gain_ /= rate;
    ang_velocity_ /= rate;

    // Advertise
    ctrl_pub_ = nh.advertise<asctec_hl_comm::mav_ctrl>("fcu/control", 10);

    // Subscribe
    joy_sub_ = nh.subscribe("joy", 1, &SubscribeAndPublish::callback, this);
    rep_vel_sub_ = nh.subscribe("repelling_velocity", 1, &SubscribeAndPublish::rep_vel_callback, this);

    // Service Caller
    client_ = nh.serviceClient<planning_msgs::publishPointsInBBX>("publishPointsInBBX");
	}

	void callback(const sensor_msgs::Joy::ConstPtr& msg)
	{
//    if (last_message_.axes.size() == 3){
//      if (last_message_.axes[2]!=msg->axes[2])
//        last_message_ = *msg;
//      else{
//        last_message_.header =msg->header;
//        last_message_.axes[0]=msg->axes[0];
//        last_message_.axes[1]=msg->axes[1];
//        last_message_.buttons=msg->buttons;
//      }
//    }
//    else
      last_message_ = *msg;
  }

  void rep_vel_callback(const repelling_velocity_msgs::repelling_velocity::ConstPtr& rep_vel){
    last_repelling_velocity_ = *rep_vel;
  }

	void integratePosition()
	{
    ros::Time now = ros::Time::now();
    static float pos[4] = {0};
    pos[0] += last_repelling_velocity_.x;
    pos[1] += last_repelling_velocity_.y;
    pos[2] += last_repelling_velocity_.z;
    if (last_message_.axes.size() >= 3) // wait for first joy message
		{
      pos[0] += last_message_.axes[0]*lin_velocity_;
      pos[1] += last_message_.axes[1]*lin_velocity_;
      pos[2] = pos[2] + z_gain_*((last_message_.axes[2] + 1) - pos[2]); // 0m < z position < 2m
//      last_message_.axes[2] = pos[2] - 1 + last_repelling_velocity_.z;
      pos[3] += (last_message_.buttons[3] - last_message_.buttons[4])*ang_velocity_;  //yaw angle
		}

    // broadcast tf transformation
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(pos[0], pos[1], pos[2]));
    transform.setRotation(tf::Quaternion(tf::Vector3(0.0, 0.0, 1.0), pos[3]));
    br.sendTransform(tf::StampedTransform(transform, now, world_frame_, base_frame_));
    if (first_ || (transform.getOrigin()-last_transform_.getOrigin()).length2()>=0.1*0.1){
      planning_msgs::publishPointsInBBX::Request req;
      planning_msgs::publishPointsInBBX::Response res;
      req.header.frame_id = base_frame_;
      req.header.stamp = now;
      client_.call(req,res);
      last_transform_=transform;
    }

    asctec_hl_comm::mav_ctrl ctrl;

    ctrl.header.frame_id = world_frame_;
    ctrl.header.stamp = now;
    ctrl.type = asctec_hl_comm::mav_ctrl::position;
    ctrl.v_max_xy = -1;
    ctrl.v_max_z = -1;
    ctrl.x = pos[0];
    ctrl.y = pos[1];
    ctrl.z = pos[2];
    ctrl.yaw = fmod(pos[3], 2 * M_PI);
    ctrl.yaw = ctrl.yaw > M_PI ? ctrl.yaw - 2 * M_PI : ctrl.yaw;
    ctrl.yaw = ctrl.yaw < -M_PI ? ctrl.yaw + 2 * M_PI : ctrl.yaw;

    ctrl_pub_.publish(ctrl);
  }

private:
//  ros::NodeHandle n_;

  std::string world_frame_,base_frame_;
  double lin_velocity_;
  double ang_velocity_;
  double z_gain_;

  ros::Publisher ctrl_pub_;
  ros::Subscriber joy_sub_, rep_vel_sub_;

  ros::ServiceClient client_;

  tf::Transform last_transform_;

  sensor_msgs::Joy last_message_;
  repelling_velocity_msgs::repelling_velocity last_repelling_velocity_;
  bool first_;

}; //End of class SubscribeAndPublish


int main(int argc, char **argv)
{

	ros::init(argc, argv, "joy_listener");

  ros::NodeHandle nh;

  double rate = 10.0; //[Hz]

	//Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject(nh);

	while (ros::ok())
	{
		ros::spinOnce();

		SAPObject.integratePosition();

    SAPObject.rate_.sleep();
	}

	return 0;
}

