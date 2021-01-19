#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <linelibrary/agvline.h>
#include <geometry_msgs/Twist.h>
#include <boost/thread/thread.hpp>
#include <agvlinepkg/MLS_Measurement.h>
#include <agv_define/agv_action.h>
#include <actionlib/server/simple_action_server.h>
#include <agv_define/lineAction.h>  // Note: "Action" is appended
#include <agv_define/agvlib.h>

typedef actionlib::SimpleActionServer<agv_define::lineAction> Server;

/* creat struct*/
Mlse_info Forward_Info;
Mlse_info Backward_Info;
cmd_vel Line_cmd_vel;

/*global variable */
uint8_t rate = 20; // Hz
agvline *agvLine;
uint8_t action_;
bool isEnable = true; 
bool succeed = NULL;
/***Create Node***/
ros::NodeHandlePtr nh;
/* Subscriber position line */
ros::Subscriber mlsForward;
ros::Subscriber mlsBackward;
/* Publisher */
ros::Publisher cmd_vel_pub;
geometry_msgs::Twist cmd_vel;

/**********************************************************************
 *                   		Define Function 
***********************************************************************/
/* Create function Call back*/
void mlsForwardCallback(const agvlinepkg::MLS_Measurement &msg);
void mlsBackwardCallback(const agvlinepkg::MLS_Measurement &msg);
/* Event function */
void agvSucceed();
void areaOne();
void areaTwo();
void areaThree();
/* Create Thread */
void agvRunForward_doStuff(Server* as, uint8_t *publish_rate);
void agvRunBackward_doStuff(Server* as, uint8_t *publish_rate);
/* Action*/
void lineActionCallback(const agv_define::lineGoalConstPtr& goal, Server* as);
void line_backward(Server* as);
void line_forward(Server* as);

/**********************************************************************
 *                   		MAIN 
***********************************************************************/
int main(int argc, char **argv)
{
	/*** ROS initialization ***/
	ros::init(argc, argv, "agvLineAction");
    // ROS_INFO("agvLineAction.cpp- main()");
	/***Create Node***/
    nh = boost::make_shared<ros::NodeHandle>();

    char *command_type = (char *)"sudo ip link set can0 type can";
    system(command_type);
    char *command_baud = (char *)"sudo ip link set can0 up type can bitrate 125000";
    system(command_baud);
    ROS_INFO("agvLineAction.cpp-336- System command config CAN");

  /* Subscriber position line */
	mlsForward = nh->subscribe("mls0", 20, mlsForwardCallback);
	mlsBackward = nh->subscribe("mls1", 20, mlsBackwardCallback);
	/* Publisher */
	cmd_vel_pub = nh->advertise<geometry_msgs::Twist>("cmd_vel", 20);
	/*Action server*/
	Server server(*nh, "line_action", boost::bind(&lineActionCallback, _1, &server), false);
	server.start();

  /* Create Class agvline */
	agvLine = new agvline(nh, (const uint8_t)rate);
	agvLine->areaOne = &areaOne;
	agvLine->areaTwo = &areaTwo;
	agvLine->areaThree = &areaThree;
	 
    ROS_INFO("agvlinectrl.cpp::42 ->AGV line is ready !!");
	ros::spin();
	delete(agvLine);
	return 0;
}

/**********************************************************************
 *                   		Function detail 
***********************************************************************/
/* fix code*/
void mlsForwardCallback(const agvlinepkg::MLS_Measurement &msg)
{
	Forward_Info.status = msg.status;
	Forward_Info.lcp = msg.lcp;
	for (int i = 0; i < 3; i++)
		Forward_Info.position[i] = msg.position[i];
} //mlsForwardCallback

void mlsBackwardCallback(const agvlinepkg::MLS_Measurement &msg)
{
	Backward_Info.status = msg.status;
	Backward_Info.lcp = msg.lcp;
	for (int i = 0; i < 3; i++)
		Backward_Info.position[i] = msg.position[i];
} //mlsForwardCallback

void agvRunForward_doStuff(Server* as, uint8_t *publish_rate)
{
    agvLine->EnventEnable(FOR_WARD);
    ROS_INFO("agvLineAction.cpp :120 -> line_forward is ready!!");
    ros::Rate loop_rate(*publish_rate);
    while (ros::ok() && isEnable==true)
    {
        ROS_INFO("agvLineAction.cpp-125-Forward Line track level is %d", agvLine->agvForwardInfo.status.track_level);
        agvLine->getDataAgvforward(Forward_Info);
        Line_cmd_vel = agvLine->agvLineforward();
        cmd_vel.linear.x = Line_cmd_vel.linear.x;
        cmd_vel.angular.z = Line_cmd_vel.angular.z;
        cmd_vel_pub.publish(cmd_vel);
        if(succeed)
        {
            isEnable = false;
            as->setSucceeded();
        }
        if(agvLine->agvForwardInfo.status.line_good == false)
        {
            isEnable = false;
            agvLine->agvStop();
            ROS_ERROR("agvLineAction.cpp-158 - Line is NOT good");
            as->setAborted(agv_define::lineResult(), "From MLS forward: no track!!");
        } 	
        loop_rate.sleep();
		ros::spinOnce();
    }
    agvLine->EnventDisEnable();
}

void agvRunBackward_doStuff(Server* as, uint8_t *publish_rate)
{
    agvLine->EnventEnable(BACK_WARD);
    ROS_INFO("agvLineAction.cpp :147 ->line_backward is ready!!");
    ros::Rate loop_rate(*publish_rate);
    while (ros::ok() && isEnable==true && !as->isPreemptRequested())
    {
        agvLine->getDataAgvbackward(Backward_Info);
        ROS_INFO("agvLineAction.cpp-155-Backward Line track level is %d", agvLine->agvBackwardInfo.status.track_level);
        Line_cmd_vel = agvLine->agvLinebackward();
        cmd_vel.linear.x = Line_cmd_vel.linear.x;
        cmd_vel.angular.z = Line_cmd_vel.angular.z;
        cmd_vel_pub.publish(cmd_vel);
        if(succeed)
        {
            isEnable = false;
            as->setSucceeded();
        }
        if(agvLine->agvBackwardInfo.status.line_good == false)
        {
            isEnable = false;
            agvLine->agvStop();
            ROS_ERROR("agvLineAction.cpp-158 - Line is NOT good");
            as->setAborted(agv_define::lineResult(), "From MLS forward: no track!!");
        } 	
        loop_rate.sleep();
        ros::spinOnce();
    }     
    agvLine->EnventDisEnable();   
}

void line_forward(Server* as)
{
    agvRunForward_doStuff(as,&rate);
}

void line_backward(Server* as)
{
    agvRunBackward_doStuff(as,&rate);
}

/* action Server*/
void lineActionCallback(const agv_define::lineGoalConstPtr& goal, Server* as)
{
	action_ = ActionState(goal->action);
	ROS_INFO("agvLineAction.cpp-187-lineActionCallback() - action: %d", action_);
	if((action_ == ACTION_LIFT_IN)||(action_ == ACTION_CHARGING_IN)){
        isEnable = true;
        succeed = false;
        line_backward(as);
        cmd_vel.linear.x = 0;
        cmd_vel.angular.z = 0;
        cmd_vel_pub.publish(cmd_vel);
	}else if((action_ == ACTION_LIFT_OUT)||(action_ == ACTION_CHARGING_OUT)){
        isEnable = true;
        succeed = false;
        line_forward(as);
        cmd_vel.linear.x = 0;
        cmd_vel.angular.z = 0;
        cmd_vel_pub.publish(cmd_vel);
	}
}//teleop_keyCallback

void agvSucceed()
{
    // sleep(1);
    succeed = true;
}
/* USER CODE EVENT CALLBACK*/
void areaOne()
{	
  /* Do some thing*/
	//ROS_INFO("RUNNING!!");
}
void areaTwo()
{
	/* Do some thing*/
	//ROS_INFO("DETECTING!!");
}
void areaThree()
{   
	/* Do some thing*/
	//ROS_INFO("STOPPING!!");
    agvLine->agvStop();
    if(agvLine->acceLineBackward.presentReality == 0 || agvLine->acceLineForward.presentReality == 0)
        agvSucceed();
}