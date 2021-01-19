#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <unistd.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <boost/thread/thread.hpp>
#include <nav_converter/speed_wheel.h>
#include <nav_converter/nav_converter.h>


/*global variable */
clock_t start;
double timeoutMs = 1; //sec
static cmd_vel cmdVel;
static speedWheel W;
static navi *naviConvert;
static uint8_t rate = 20; // Hz

std::string driver_left, driver_right;

/* Create function Call back*/
void cmd_velCallback(const geometry_msgs::Twist &msg);
/* Create Function Thread */
void navgationDoStuff(ros::NodeHandlePtr nh, uint8_t *publish_rate);
void checkCallback();
bool loadParam(std::string node_name);

int main(int argc, char **argv)
{
    /* Khoi tao Node */
    ros::init(argc, argv, "nav_converter");
    ros::NodeHandlePtr nh = boost::make_shared<ros::NodeHandle>();
    ROS_INFO("nav_converter.cpp::42 -> is ready !!");

    std::string node_name = ros::this_node::getName();
	ROS_INFO("nav_converter.cpp-node_name: %s", node_name.c_str());
	if(loadParam(node_name)){
		ROS_INFO("nav_converter.cpp-Load parameter successfull");
	}else{
		ROS_ERROR("nav_converter.cpp-Error when load parameter");
	}

    /*Create thread*/
	boost::thread nav_converterThread(navgationDoStuff,nh, &rate);
	nav_converterThread.join();
    ros::spin();
    delete(naviConvert);
    return 0;
}
/**********************************************************
                    Function detail
**********************************************************/
void cmd_velCallback(const geometry_msgs::Twist &msg)
{
    start = clock();
    cmdVel.linear.x = msg.linear.x;
    cmdVel.angular.z = msg.angular.z;
    // ROS_INFO("Navigation_control.cpp-83-  cmdVel.linear.x: %f   cmdVel.angular.z: %f", cmdVel.linear.x, cmdVel.angular.z);
} //cmd_velCallback

void navgationDoStuff(ros::NodeHandlePtr nh, uint8_t *publish_rate)
{
    start = clock();
    ros::Rate loop_rate(*publish_rate);
    naviConvert = new navi(nh);
    /* Publisher */
    ros::Publisher wheel_left_pub = nh->advertise<nav_converter::speed_wheel>(driver_left+"/control_wheel", 20);
    ros::Publisher wheel_right_pub = nh->advertise<nav_converter::speed_wheel>(driver_right+"/control_wheel", 20);
    nav_converter::speed_wheel motor;
    /* Subscriber */
    ros::Subscriber velCallback = nh->subscribe("cmd_vel", 20, cmd_velCallback);
    ROS_INFO("nav_converter.cpp:59 -> navgationDoStuff is ready!!");
    while (ros::ok())
    {
        W = naviConvert->navigationConverter(cmdVel);
        motor.wheel_letf = W.letf;
        motor.wheel_right = W.right;
        // ROS_INFO("nav_converter.cpp-81-wheel_letf: %d", motor.wheel_letf);
        // ROS_INFO("nav_converter.cpp-82-wheel_right: %d", motor.wheel_right);

        wheel_left_pub.publish(motor);
        wheel_right_pub.publish(motor);
        loop_rate.sleep();
        //checkConnect();
        ros::spinOnce();
    }
}

void checkCallback(){
    if((double)(clock() - start)/CLOCKS_PER_SEC >= timeoutMs) 
    {
        W.letf = W.right= 0;
        start = clock();
    }
}

bool loadParam(std::string node_name){
	ROS_INFO("nav_converter.cpp-loadParam() -> node_name: %s", node_name.c_str());
	
	if(!ros::param::get(node_name + "/driver_name_left", driver_left)){
		return false;
	}
	ROS_INFO("nav_converter.cpp- driver_name_left: %s", driver_left.c_str());
	if(!ros::param::get(node_name + "/driver_name_right", driver_right)){
		return false;
	}
	ROS_INFO("nav_converter.cpp- driver_name_right: %s", driver_right.c_str());

	return true;
}