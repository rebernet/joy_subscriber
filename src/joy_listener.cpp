#include "ros/ros.h"
#include <sensor_msgs/Joy.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#define __NO_OCTOMAP__

#ifndef __NO_OCTOMAP__
#include <repelling_velocity_msgs/repelling_velocity.h>
#endif

#include <asctec_hl_comm/mav_ctrl.h>

class SubscribeAndPublish
{
public:

  ros::Rate rate_;

  SubscribeAndPublish(ros::NodeHandle& nh)
    : rate_(10.0), // [Hz]
      world_frame_("world"),
      base_frame_("vicon/hummy_student/hummy_student"),
      set_point_viz_frame_("fcu"),
      lin_velocity_(0.5), // [m/s]
      max_vel_(1.5),
      z_gain_(0.1),        // [1/s]
      ang_velocity_(0.6), // [rad/s]
      first_(true),
      full_repelling_velocity_(false),
      xy_repelling_velocity_(false),
      z_vel_offset_(0.0),
      send_commands_(false)
	{
#ifndef __NO_OCTOMAP__
    last_repelling_velocity_.x = 0.0;
    last_repelling_velocity_.y = 0.0;
    last_repelling_velocity_.z = 0.0;
#endif

    // Parameters
    ros::NodeHandle private_nh("~");
    double rate = 1.0/rate_.expectedCycleTime().toSec(); //default value
    private_nh.param("rate",rate,rate);
    rate_ = ros::Rate(rate);
    private_nh.param("world_frame",world_frame_,world_frame_);
    private_nh.param("base_frame",base_frame_,base_frame_);
    private_nh.param("set_point_viz_frame",set_point_viz_frame_,set_point_viz_frame_);
    private_nh.param("lin_velocity",lin_velocity_,lin_velocity_);
    private_nh.param("max_velocity",max_vel_,max_vel_);
    private_nh.param("z_gain",z_gain_,z_gain_);
    private_nh.param("ang_velocity",ang_velocity_,ang_velocity_);
    
    simulation_ = set_point_viz_frame_ == base_frame_;
    if (simulation_)
      ROS_INFO("set_point_viz_frame_ == base_frame_, not sending commands but visualizing repelling velocity");

    tf::StampedTransform UAVToWorldTf;
    try {
      tf_listener_.waitForTransform(world_frame_,base_frame_,ros::Time::now(), ros::Duration(3.0));
      tf_listener_.lookupTransform(world_frame_,base_frame_,ros::Time(0),UAVToWorldTf);
      send_commands_ = !simulation_;
    } catch(tf::TransformException& ex){
      ROS_ERROR_STREAM( "Transform error of sensor data: " << ex.what() << ", not sending commands!");
    }
    last_transform_.setOrigin(UAVToWorldTf.getOrigin());
    last_transform_.setRotation(UAVToWorldTf.getRotation());

    // Advertise
    ctrl_pub_ = nh.advertise<asctec_hl_comm::mav_ctrl>("hummy/fcu/control", 10);

    // Subscribe
    joy_sub_ = nh.subscribe("joy", 1, &SubscribeAndPublish::callback, this);
#ifndef __NO_OCTOMAP__
    rep_vel_sub_ = nh.subscribe("repelling_velocity", 1, &SubscribeAndPublish::rep_vel_callback, this);
#endif
	}

	void callback(const sensor_msgs::Joy::ConstPtr& msg)
	{
    if (msg->axes.size()==6 && last_message_.buttons.size()==12)// Logitech Controller && last_message_ is not empty!
    {
#ifndef __NO_OCTOMAP__
      if (msg->buttons[3] && !last_message_.buttons[3])
      {
        if (z_vel_offset_)
          z_vel_offset_ = 0.0;
        else
          z_vel_offset_ = -last_repelling_velocity_.z;
      }
#endif
      if (msg->buttons[8] && !last_message_.buttons[8])
        xy_repelling_velocity_ = !xy_repelling_velocity_;
      if (msg->buttons[9] && !last_message_.buttons[9])
        full_repelling_velocity_ = !full_repelling_velocity_;
      if (msg->buttons[6] && msg->buttons[7])// Reset!
      {
        ROS_WARN("Reset! Deactivated repelling velocity!");
        first_ = true;
        xy_repelling_velocity_ = false;
        full_repelling_velocity_ = false;
        tf::StampedTransform UAVToWorldTf;
        try {
          tf_listener_.waitForTransform(world_frame_,base_frame_,ros::Time::now(), ros::Duration(3.0));
          tf_listener_.lookupTransform(world_frame_,base_frame_,ros::Time(0),UAVToWorldTf);
        } catch(tf::TransformException& ex){
          ROS_ERROR_STREAM( "Transform error of sensor data: " << ex.what() << ", not sending commands!");
          send_commands_ = false;
        }
        last_transform_.setOrigin(UAVToWorldTf.getOrigin());
        last_transform_.setRotation(UAVToWorldTf.getRotation());
      }
    }

    last_message_ = *msg;
  }

#ifndef __NO_OCTOMAP__
  void rep_vel_callback(const repelling_velocity_msgs::repelling_velocity::ConstPtr& rep_vel){
    last_repelling_velocity_ = *rep_vel;
  }
#endif

	void integratePosition()
	{
    static ros::Time past = ros::Time(0);
    ros::Time now = ros::Time::now();
    double dt;
    if(past != ros::Time(0))
      dt = now.toSec() - past.toSec();
    else
      dt = 0.0;
    past = now;

    static float pos[4] = {0.0, 0.0, 0.0, 0.0};
    if ((first_ && send_commands_) || simulation_)// if send_commands_ is off last_transform is empty
    {
      pos[0] = last_transform_.getOrigin().x();
      pos[1] = last_transform_.getOrigin().y();
      pos[2] = last_transform_.getOrigin().z();
      pos[3] = tf::getYaw(last_transform_.getRotation());
#ifdef __NO_OCTOMAP__
      first_ = false;
#endif
    }

    float vel[4] = {0.0, 0.0, 0.0, 0.0};
#ifndef __NO_OCTOMAP__
    if (xy_repelling_velocity_||full_repelling_velocity_)
    {
      vel[0] += last_repelling_velocity_.x;
      vel[1] += last_repelling_velocity_.y;
    }
    if (full_repelling_velocity_)
      vel[2] += last_repelling_velocity_.z+z_vel_offset_;
#endif

    if (last_message_.axes.size() == 3) // Joystick
		{
      vel[0] += last_message_.axes[1]*lin_velocity_;
      vel[1] += last_message_.axes[0]*lin_velocity_;
      vel[2] =  z_gain_*((last_message_.axes[2] + 1) - pos[2]); // 0m < z position < 2m
//      last_message_.axes[2] = pos[2] - 1 + last_repelling_velocity_.z;
      vel[3] += (last_message_.buttons[3] - last_message_.buttons[4])*ang_velocity_;  //yaw angle
		}
    else if (last_message_.axes.size() == 6) // Logitech Controller
    {
      vel[0] += (last_message_.axes[3]*cos(pos[3])-last_message_.axes[2]*sin(pos[3]))*lin_velocity_;
      vel[1] += (last_message_.axes[3]*sin(pos[3])+last_message_.axes[2]*cos(pos[3]))*lin_velocity_;
      vel[2] += last_message_.axes[1]*lin_velocity_;
      vel[3] += last_message_.axes[0]*ang_velocity_;
    }

    float vel_length = vel[0]*vel[0] + vel[1]*vel[1] + vel[2]*vel[2];
    if (vel_length > max_vel_)
    {
      vel[0] *= max_vel_/vel_length;
      vel[1] *= max_vel_/vel_length;
      vel[2] *= max_vel_/vel_length;
    }
    pos[0] += dt*vel[0];
    pos[1] += dt*vel[1];
    pos[2] += dt*vel[2];
    pos[3] += dt*vel[3];

    // broadcast tf transformation
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(pos[0], pos[1], pos[2]));
    transform.setRotation(tf::Quaternion(tf::Vector3(0.0, 0.0, 1.0), pos[3]));
    if (!simulation_)
		br.sendTransform(tf::StampedTransform(transform, now, world_frame_, set_point_viz_frame_));
#ifndef __NO_OCTOMAP__
    if (first_  || simulation_ || (transform.getOrigin()-last_transform_.getOrigin()).length2()>=0.1*0.1){//TODO make resolution parameter
//      planning_msgs::publishPointsInBBX::Request req;
//      planning_msgs::publishPointsInBBX::Response res;
//      req.header.frame_id = set_point_viz_frame_;
//      req.header.stamp = ros::Time(0);//now;
//      client_.call(req,res);
      if (!simulation_)
        last_transform_=transform;
      else
      {
        tf::StampedTransform UAVToWorldTf;
        try {
          tf_listener_.waitForTransform(world_frame_,base_frame_,ros::Time::now(), ros::Duration(3.0));
          tf_listener_.lookupTransform(world_frame_,base_frame_,ros::Time(0),UAVToWorldTf);
        } catch(tf::TransformException& ex){
          ROS_ERROR_STREAM( "Transform error of sensor data: " << ex.what() << ", not sending commands!");
        }
        last_transform_.setOrigin(UAVToWorldTf.getOrigin());
        last_transform_.setRotation(UAVToWorldTf.getRotation());
      }
      first_ = false;
    }
#endif

    if (send_commands_){
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
  }

private:
//  ros::NodeHandle n_;

  std::string world_frame_,base_frame_,set_point_viz_frame_;
  double lin_velocity_;
  double max_vel_;
  double ang_velocity_;
  double z_gain_;
  bool full_repelling_velocity_, xy_repelling_velocity_, constant_height_, send_commands_, simulation_;
  float z_vel_offset_;

  ros::Publisher ctrl_pub_;
  ros::Subscriber joy_sub_, rep_vel_sub_;
  tf::TransformListener tf_listener_;

  tf::Transform last_transform_;

  sensor_msgs::Joy last_message_;
#ifndef __NO_OCTOMAP__
  repelling_velocity_msgs::repelling_velocity last_repelling_velocity_;
#endif
  bool first_;

}; //End of class SubscribeAndPublish


int main(int argc, char **argv)
{

	ros::init(argc, argv, "joy_listener");

  ros::NodeHandle nh;

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

