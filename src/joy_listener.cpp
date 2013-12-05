#include "ros/ros.h"
#include <sensor_msgs/Joy.h>
#include <tf/transform_broadcaster.h>


class SubscribeAndPublish
{
public:
	sensor_msgs::Joy lastmessage;
	sensor_msgs::Joy output;

	SubscribeAndPublish()
	{
		lin_velocity_ = 0.05; // [cm/s]
		ang_velocity_ = 0.06; // [rad/s]
		//Topic you want to publish
		pub_ = n_.advertise<sensor_msgs::Joy>("position", 1);

		//Topic you want to subscribe
		sub_ = n_.subscribe("joy", 1, &SubscribeAndPublish::callback, this);
	}

	void callback(const sensor_msgs::Joy::ConstPtr& msg)
	{
		lastmessage = *msg;
	}

	void integratePosition()
	{

		if (lastmessage.axes.size() == 3) // wait for first joy message
		{
			static float pos[4] = {0};
			pos[0] += lastmessage.axes[0]*lin_velocity_;
			pos[1] += lastmessage.axes[1]*lin_velocity_;
			pos[2]  = (lastmessage.axes[2] + 1); // 0m < z position < 2m
			pos[3] += (lastmessage.buttons[3] - lastmessage.buttons[4])*ang_velocity_;  //yaw angle

			output.axes.resize(4);
			output.axes[0] = pos[0];
			output.axes[1] = pos[1];
			output.axes[2] = pos[2];
			output.axes[3] = pos[3];

			// broadcast tf transformation
			static tf::TransformBroadcaster br;
			tf::Transform transform;
			transform.setOrigin(tf::Vector3(output.axes[0], output.axes[1], output.axes[2]));
			transform.setRotation(tf::Quaternion(0, 0, output.axes[3]));
			br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "UAV"));
		}
		output.header.stamp = ros::Time::now();
		pub_.publish(output);
	}

private:
	float lin_velocity_;
	float ang_velocity_;
	ros::NodeHandle n_;
	ros::Publisher pub_;
	ros::Subscriber sub_;

}; //End of class SubscribeAndPublish


int main(int argc, char **argv)
{

	ros::init(argc, argv, "joy_listener");

	//Create an object of class SubscribeAndPublish that will take care of everything
	SubscribeAndPublish SAPObject;

	ros::Rate r(10); // 10 hz

	while (ros::ok())
	{
		ros::spinOnce();

		SAPObject.integratePosition();

		r.sleep();
	}

	return 0;
}

