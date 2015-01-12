/*
Daman Bareiss
DARC Lab @ University of Utah

This node takes in the /new_u control input and outputs it to the vrep simulator as /vrep/input

*/

// Includes
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>

geometry_msgs::Point point_out;

// Read in new_u
void u_callback(const geometry_msgs::Vector3& u_in)
{
    point_out.x = u_in.x;
    point_out.y = u_in.y;
    point_out.z = u_in.z;
}

// Main Loop
int main(int argc, char** argv)
{
    // ROS initialization
	ros::init(argc, argv,"sim_control");
    ros::NodeHandle node;
    ros::Rate loop_rate(50);

    // ROS publishers
	ros::Publisher point_pub;
	point_pub = node.advertise<geometry_msgs::Point>("/vrep/input",1);
	
    // ROS subscribers
    ros::Subscriber u_sub;
    u_sub = node.subscribe("/new_u", 1, u_callback);

    // Starting main loop
 	while (ros::ok()) 
    {   
        point_pub.publish(point_out);
	    ros::spinOnce();
	    loop_rate.sleep();
    }//ros::ok
}//main
