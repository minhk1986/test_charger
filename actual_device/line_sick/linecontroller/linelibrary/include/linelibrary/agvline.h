//
// Created by hiep on  11/11/2020.
//

#ifndef AGVLINE_H
#define AGVLINE_H
#pragma once 

#include <stdint.h>
#include <ros/ros.h>
#include <unistd.h>
#include <math.h>
#include <time.h>
#include <boost/thread/thread.hpp>
#include <linelibrary/agvLineparam.h>
#include <libraryparam/agvparam.h>

#define UP_SPEED 				true
#define REDUCE_SPEED 			false
#define RUN 					true
#define NOT_RUN 				false
#define FOR_WARD				1
#define BACK_WARD       -1

/*CLASS*/
class agvline {
private:	

	/*  Acceleration parameter */
	struct acceparam {
		clock_t beginTime;
		double  presentSetting;
		double  presentReality;
		bool acceleration;
	};

	/* parameter of */ 
 	agvLineParam  paramAgvforward;
	agvLineParam  paramAgvbackward;
	boost::thread EventThread;
	int threadRate;
	
	void agvGetparam(ros::NodeHandlePtr nodeHandle,  agvLineParam &param, const char *Nodename);
	void agvgGetlineData(Mlse_info receiveData, agvLineinfo &msg);
	void upSpeed(double &present_speed, const double step);
	void reduceSpeed(double &present_speed, const double step);
	static void EventforwarddoStuff(void *payload);
	static void EventbackwarddoStuff(void *payload);
	cmd_vel agvlinepurePursuite(agvLineParam agvparam, agvLineinfo agvInfo, acceparam acce);

public:

	bool theadEnable;
	agvLineinfo agvForwardInfo;
	agvLineinfo agvBackwardInfo;
	acceparam acceLineForward;
	acceparam acceLineBackward;

  void EnventEnable(int8_t direct);
	void EnventDisEnable(void);
	void acceleration(acceparam &acce, const float_t time);
	void bockAcceleration(acceparam &acce);
	void getDataAgvforward(Mlse_info receiveData);
	void getDataAgvbackward(Mlse_info receiveData);
	void agvStop(void);
	cmd_vel agvLineforward(void);
	cmd_vel	agvLinebackward(void);

	void (*areaOne) (void);
	void (*areaTwo) (void);
	void (*areaThree) (void);

	agvline(ros::NodeHandlePtr nodeHandle, const int publish_rate );
	 ~agvline();
};

#endif