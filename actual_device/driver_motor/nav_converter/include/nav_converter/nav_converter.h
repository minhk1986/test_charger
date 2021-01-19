#ifndef NAVI_H
#define NAVI_H
#pragma once

#include <ros/ros.h>
#include <unistd.h>
#include <stdint.h>
#include <libraryparam/agvparam.h>

class navi
{
private:
	void agvGetparam(ros::NodeHandlePtr nodeHandle, agvParam &param);

public:
	agvParam param;
	speedWheel navigationConverter(cmd_vel msg);
	navi(ros::NodeHandlePtr nodeHandle);
	~navi();
};

navi::navi(ros::NodeHandlePtr nodeHandle)
{
	this->agvGetparam(nodeHandle, this->param);
}

navi::~navi() {}

speedWheel navi::navigationConverter(cmd_vel msg)
{
	speedWheel V;
	// ROS_INFO("nav_converter.h-32- L: %lf", this->param.L);
	// ROS_INFO("nav_converter.h-33- R: %lf", this->param.R);
	// ROS_INFO("nav_converter.h-34- K: %lf", this->param.K);
	// ROS_INFO("nav_converter.h-35- RAD_PER_RPM: %lf", RAD_PER_RPM);
	
	// ROS_INFO("nav_converter.h-32-x: %f", msg.linear.x);
	// ROS_INFO("nav_converter.h-33-z: %f", msg.angular.z);
	V.letf = (2 * msg.linear.x - msg.angular.z * this->param.L) / (2 * this->param.R) * this->param.K * RAD_PER_RPM;
	V.right = -(2 * msg.linear.x + msg.angular.z * this->param.L) / (2 * this->param.R) * this->param.K * RAD_PER_RPM;

	// ROS_INFO("nav_converter.h-42-V.letf: %d", V.letf);
	// ROS_INFO("nav_converter.h-43-V.right: %d", V.right);

	// double speedLeft = (V.letf*this->param.R)/(this->param.K*RAD_PER_RPM);
	// double speedRight = (V.right*this->param.R)/(this->param.K*RAD_PER_RPM);
	
	// double vx = (speedLeft - speedRight)/2;
	// double vth = -(speedLeft + speedRight)/(this->param.L);
	// ROS_INFO("nav_converter.cpp-vx: %lf", vx);
	// ROS_INFO("nav_converter.cpp-vth: %lf", vth);

	if (abs(V.letf) > this->param.speedMotor_max)
	{
		if (V.letf > 0)
			V.letf = this->param.speedMotor_max;
		else if (V.letf < 0)
			V.letf = -this->param.speedMotor_max;
	}

	if (abs(V.right) > this->param.speedMotor_max)
	{
		if (V.right > 0)
			V.right = this->param.speedMotor_max;
		else if (V.right < 0)
			V.right = -this->param.speedMotor_max;
	}
	if (abs(V.letf) < this->param.speedMotor_min)
		V.letf = 0;
	if (abs(V.right) < this->param.speedMotor_min)
		V.right = 0;

	return V;
}

void navi::agvGetparam(ros::NodeHandlePtr nodeHandle, agvParam &param)
{
	char paramName[70];
	sprintf(paramName, "/L");
	nodeHandle->getParam(paramName, param.L);
	ROS_INFO("nav_converter.h- %s = %f (Khoang cach 2 banh - m)", paramName, param.L);
	sprintf(paramName, "/R");
	nodeHandle->getParam(paramName, param.R);
	ROS_INFO("nav_converter.h- %s = %f (Ban kinh banh xe - m)", paramName, param.R);
	sprintf(paramName, "/K");
	nodeHandle->getParam(paramName, param.K);
	ROS_INFO("nav_converter.h- %s = %f (ti so truyen dong co)", paramName, param.K);
	sprintf(paramName, "/SpeedMotorMax");
	nodeHandle->getParam(paramName, param.speedMotor_max);
	ROS_INFO("nav_converter.h- %s = %d (Toc do (xung) toi da cua dong co)", paramName, param.speedMotor_max);
	sprintf(paramName, "/SpeedMotorMin");
	nodeHandle->getParam(paramName, param.speedMotor_min);
	ROS_INFO("nav_converter.h- %s = %d (Toc do (xung) toi thieu cua dong co)", paramName, param.speedMotor_min);
}

#endif