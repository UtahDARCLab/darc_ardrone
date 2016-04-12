/* 

	Daman Bareiss
	DARC Lab
	
	This node taks in joystick commands and sends them to the ardrone_atuonomy as new_u for testing
*/

// Includes
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>

// Variable Init
double joy_x_,joy_y_,joy_z_,joy_yaw_;
double joy_x,joy_y,joy_z,joy_yaw;
sensor_msgs::Joy joy_msg_in;
geometry_msgs::Vector3 u_out;
geometry_msgs::Vector3 curr_pos, hold_pos;
geometry_msgs::Vector3 curr_vel;
std_msgs::Float32 yaw_out;
double curr_yaw, hold_yaw, curr_yawVel;

double Kp = 0.14;
double Kpx = Kp, Kpy = Kp, Kpz = 2.0*Kp, Kpyaw = 1.5*Kp;

double Kd = 0.06;
double Kdx = Kd, Kdy = Kd, Kdz = 1.1*Kd, Kdyaw = Kd;

// Read joystick postions
void joy_callback(const sensor_msgs::Joy& joy_msg_in)
{
	//Take in xbox controller
	joy_z_   =  joy_msg_in.axes[1]; //left stick up-down
	joy_yaw_ =  joy_msg_in.axes[0]; //left stick left-right
	joy_y_   =  joy_msg_in.axes[4]; //right stick up-down
	joy_x_   = -joy_msg_in.axes[3]; //right stick left/right
}

// Read mocap position
void pos_callback(const geometry_msgs::Vector3& pos_msg_in)
{
	curr_pos.x = pos_msg_in.x;
	curr_pos.y = pos_msg_in.y;
	curr_pos.z = pos_msg_in.z;
}

// Read mocap yaw angle
void yaw_callback(const std_msgs::Float32& yaw_msg_in)
{
	curr_yaw = yaw_msg_in.data;
}

// Read mocap yaw angular velocity
void yawVel_callback(const geometry_msgs::Vector3& rdot_msg_in)
{
	curr_yawVel = rdot_msg_in.z;
}

// Read velocity
void vel_callback(const geometry_msgs::Vector3& vel_msg_in)
{
	curr_vel.x = vel_msg_in.x;
	curr_vel.y = vel_msg_in.y;
	curr_vel.z = vel_msg_in.z;
}

// Combine joystick variables
void merge_new_msgs(void)
{
	joy_x=joy_x_;
	joy_y=joy_y_;
	joy_z=joy_z_;
	joy_yaw = joy_yaw_;
}

int main(int argc, char** argv)
{
	ros::init(argc,argv,"flyfromjoy");
	ros::NodeHandle node;
	ros::Rate loop_rate(50);

	ros::Publisher u_pub,yaw_pub;
	u_pub = node.advertise<geometry_msgs::Vector3>("desired_u",1);
	yaw_pub = node.advertise<std_msgs::Float32>("desired_yaw",1);
	ros::Subscriber joy_sub;
	joy_sub = node.subscribe("joy",1,joy_callback);
	ros::Subscriber pos_sub;
	pos_sub = node.subscribe("current_position",1,pos_callback);
	ros::Subscriber yaw_sub;
	yaw_sub = node.subscribe("current_yaw",1,yaw_callback);
	ros::Subscriber vel_sub;
	vel_sub = node.subscribe("current_velocity",1,vel_callback);
	ros::Subscriber yawVel_sub;
	yawVel_sub = node.subscribe("current_rdot",1,yawVel_callback);

	double dead_zone = 0.15;
	while(ros::ok())
	{
		merge_new_msgs();
		u_out.x = joy_x;
		u_out.y = joy_y;
		u_out.z = joy_z;
		yaw_out.data = joy_yaw;

		if(u_out.x < dead_zone && u_out.x > -dead_zone) {
			u_out.x = 0.0;
		} else if (u_out.x < 0.0) {
			u_out.x = (u_out.x + dead_zone) / (1.0 - dead_zone);
		} else {
			u_out.x = (u_out.x - dead_zone) / (1.0 - dead_zone);
		}
		
		if(u_out.y < dead_zone && u_out.y > -dead_zone) {
			u_out.y = 0.0;
		} else if (u_out.y < 0.0) {
			u_out.y = (u_out.y + dead_zone) / (1.0 - dead_zone);
		} else {
			u_out.y = (u_out.y - dead_zone) / (1.0 - dead_zone);
		}

		if(u_out.z < dead_zone && u_out.z > -dead_zone) {
			u_out.z = 0.0;
		} else if (u_out.z < 0.0) {
			u_out.z = (u_out.z + dead_zone) / (1.0 - dead_zone);
		} else {
			u_out.z = (u_out.z - dead_zone) / (1.0 - dead_zone);
		}

		
		if(joy_yaw < 1.1*dead_zone && 1.1*joy_yaw > -dead_zone) {
			yaw_out.data = 0.0;
		} else if (yaw_out.data < 0.0) {
			yaw_out.data = (yaw_out.data + dead_zone) / (1.0 - dead_zone);
		} else {
			yaw_out.data = (yaw_out.data - dead_zone) / (1.0 - dead_zone);
		}
		
		u_pub.publish(u_out);
		yaw_pub.publish(yaw_out);
		ros::spinOnce();
		loop_rate.sleep();
	}
}

	
	
	
