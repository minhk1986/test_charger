#include "linelibrary/agvline.h"

/************************************************************************************
	agvline : Create class agv line and read all parameter
*************************************************************************************
Input :
--------------------------------------------------
nodeHandle  : ros node handle
publish_rate  : rate for while loop
Output :
----------------------------------------------------
no output
************************************************************************************/
agvline::agvline(ros::NodeHandlePtr nodeHandle, const int publish_rate)
{
	this->threadRate = publish_rate;
	this->agvGetparam(nodeHandle, this->paramAgvforward, "conceptForward");
	this->agvGetparam(nodeHandle, this->paramAgvbackward, "conceptBackward");
	this->acceLineForward.beginTime = clock();
	this->acceLineBackward.beginTime = clock();
	this->acceLineForward.presentSetting = 0;
	this->acceLineBackward.presentSetting = 0;
}

/************************************************************************************
	~agvline :  Not used
*************************************************************************************
Input :
--------------------------------------------------
no input
Output :
----------------------------------------------------
no output
************************************************************************************/
agvline::~agvline() {}

/************************************************************************************
EnventEnable :  
*************************************************************************************
Input :
--------------------------------------------------
direct : phuong huong
Output :
----------------------------------------------------
no output
************************************************************************************/
void agvline::EnventEnable(int8_t direct)
{
	this->theadEnable = true;
	if(this->EventThread.joinable() == false)
		if(direct == FOR_WARD)
			this->EventThread = boost::thread(this->EventforwarddoStuff,this);
		else if(direct == BACK_WARD)
			this->EventThread = boost::thread(this->EventbackwarddoStuff,this);
	else ROS_ERROR("Event thread busy !!"); 
}

/************************************************************************************
EnventDisEnable :  
*************************************************************************************
Input :
--------------------------------------------------
no input
Output :
----------------------------------------------------
no output
************************************************************************************/
void agvline::EnventDisEnable()
{
	this->theadEnable = false;
	if(this->EventThread.joinable() == true)
		this->EventThread.join();
}

/************************************************************************************
	agvGetparam :  load parameter concept of agv 
*************************************************************************************
Input :
--------------------------------------------------
nodeHandle : ros node handle
param 			  : address of struct agvLineParam
Nodename    : the name of concept agv in  yaml file 
Output :
----------------------------------------------------
no output
************************************************************************************/
void agvline::agvGetparam(ros::NodeHandlePtr nodeHandle, agvLineParam &param, const char *Nodename)
{

	char paramName[70];

	sprintf(paramName, "/L");
	nodeHandle->getParam(paramName, param.agv.L);
	ROS_INFO("agvLine.cpp- %s = %f (Khoang cach 2 banh - m)", paramName,param.agv.L);
	sprintf(paramName, "/R");
	nodeHandle->getParam(paramName, param.agv.R);
	ROS_INFO("agvLine.cpp- %s = %f (Ban kinh banh xe - m)", paramName,param.agv.R);
	sprintf(paramName, "/K");
	nodeHandle->getParam(paramName, param.agv.K);
	ROS_INFO("agvLine.cpp- %s = %f (ti so truyen dong co)", paramName,param.agv.K);
	sprintf(paramName, "/%s/V", Nodename);
	nodeHandle->getParam(paramName, param.V);
	ROS_INFO("agvLine.cpp- %s = %f", paramName,param.V);
	sprintf(paramName, "/%s/Lm", Nodename);
	nodeHandle->getParam(paramName, param.Lm);
	ROS_INFO("agvLine.cpp- %s = %f", paramName,param.Lm);
	sprintf(paramName, "/%s/PresentRunning", Nodename);
	nodeHandle->getParam(paramName, param.PresentRunning);
	param.PresentRunning /= 100;
	ROS_INFO("agvLine.cpp- %s = %f", paramName,param.PresentRunning);
	sprintf(paramName, "/%s/PresentSlow", Nodename);
	nodeHandle->getParam(paramName, param.PresentSlow);
	param.PresentSlow /= 100;
	ROS_INFO("agvLine.cpp- %s = %f", paramName,param.PresentSlow);
}

void agvline::agvgGetlineData(Mlse_info receiveData, agvLineinfo &msg)
{
	msg.mlse = receiveData; 
	/* 
	* #LCP
	* The following assignment applies (see "Outputof line center points", page 40):
	* 0 => No track found
	* 2 => One track found
	* 3 => Two tracks found: Left diverter
	* 6 => Two tracks found: Left diverter
	* 7 => Three tracks found or 90 °C intersection
	*/
	msg.lcp.lcp_nummber = msg.mlse.lcp & LCP_NUM_BIT;
	/* 
	* Marker 
	* Bit 0 is the introductory character bit
	* Bit 1...4 present code 1...15
	*/
	msg.lcp.marker = msg.mlse.lcp & MAKER_NUM_BIT;
	/* 
	* True is  Sufficiently strong track detected
	* Fasle is  No track or track too weak
	*/
	msg.status.line_good = msg.mlse.status & LINE_IS_GOOD_BIT;
	/*
	* Display of magnetic field strength in accor‐dance
	*/
	msg.status.track_level = msg.mlse.status & TRACK_LEVEL_BIT;
	/*
	* Indicates whether or not the measuring rangehas been inverted
	* False => Negative positions on cable outlet side
	* True => Positive positions on cable outlet side
	*/
	msg.status.sensor_fipped = msg.mlse.status & SENSOR_FLIPPED_BIT;
	/*
	* Indicates whether the upper surface of themagnetic tape is magnetized to the north orsouth pole
	* False => North pole
	* True => South pole
	*/
	msg.status.polarity = msg.mlse.status & POLARITY_BIT;
	/* 
	* False => No code present to read
	* True => Sensor is reading code
	*/
	msg.status.reading_code = msg.mlse.status & READING_CODE_BIT;
	/* Error register */
	msg.mlse.error_register = receiveData.error_register;
	
}

void agvline::getDataAgvforward(Mlse_info receiveData)
{
	this->agvgGetlineData(receiveData, this->agvForwardInfo);
}

void agvline::getDataAgvbackward(Mlse_info receiveData)
{
	this->agvgGetlineData(receiveData, this->agvBackwardInfo);
}

cmd_vel agvline::agvlinepurePursuite(agvLineParam agvparam, agvLineinfo agvInfo, acceparam acce)
{
	double thetaAngle;
	cmd_vel result;
	thetaAngle = atan(agvInfo.mlse.position[1] / agvparam.Lm);
	result.linear.x = agvparam.V * acce.presentReality - 0.55*abs(agvInfo.mlse.position[1]);
	result.angular.z = 0.6*result.linear.x * 2 * sin(thetaAngle) / sqrt(pow(agvInfo.mlse.position[1], 2) + pow(agvparam.Lm, 2));
	return result;
}

cmd_vel agvline::agvLineforward(void)
{
	this->bockAcceleration(this->acceLineBackward);
	this->acceleration(this->acceLineForward, 1);
	return this->agvlinepurePursuite(this->paramAgvforward, this->agvForwardInfo, this->acceLineForward);
}

cmd_vel agvline::agvLinebackward(void)
{
	this->bockAcceleration(this->acceLineForward);
	this->acceleration(this->acceLineBackward, 1);
	return this->agvlinepurePursuite(this->paramAgvbackward, this->agvBackwardInfo, this->acceLineBackward);
}

void agvline::agvStop()
{
	this->acceLineForward.presentSetting = 0;
	this->acceLineBackward.presentSetting = 0;
}

void agvline::acceleration(acceparam &acce, const float_t time)
{
	if (float_t(clock() - acce.beginTime) / CLOCKS_PER_SEC * 1000 >= time)
	{
		if (acce.acceleration == UP_SPEED)
		{
			if (acce.presentReality < acce.presentSetting)
			{
				upSpeed(acce.presentReality, 0.015);
			}
			else if (acce.presentReality > acce.presentSetting)
			{
				acce.presentReality = acce.presentSetting;
			}
		}
		else
		{
			if (acce.presentReality > acce.presentSetting)
			{
				reduceSpeed(acce.presentReality, 0.015);
			}
			else if (acce.presentReality < acce.presentSetting)
			{
				acce.presentReality = acce.presentSetting;
			}
		}
		//ROS_INFO("acceleration = %x acceset = %f accereality = %f",acce.acceleration,acce.presentSetting,acce.presentReality);
	}
}

void agvline::bockAcceleration(acceparam &acce)
{
	acce.presentSetting = 0;
	acce.presentReality = 0;
}

void agvline::upSpeed(double &present_speed, const double step)
{
	present_speed += step;
}

void agvline::reduceSpeed(double &present_speed, const double step)
{
	present_speed -= step;
}

void agvline::EventforwarddoStuff(void *payload)
{
	agvline *as = (agvline *)payload;
	ros::Rate loop_rate(as->threadRate);
	ROS_INFO("Event forward do Stuff is ready !!");
	while (ros::ok() && as->theadEnable == true)
	{
		if (as->agvForwardInfo.mlse.error_register == 0)
		{
			if(as->agvForwardInfo.status.line_good)
			{
				if (as->agvForwardInfo.mlse.position[2] > 0.00 && as->agvForwardInfo.mlse.lcp == 7)
				{
					// This->acceLineForward.presentSetting = 0;
					if (as->areaThree)
						as->areaThree();
				}
				else if (as->agvForwardInfo.mlse.position[2] <= 0.00)
				{
					if (as->agvForwardInfo.mlse.position[0] == 0.00)
					{
						as->acceLineForward.acceleration = UP_SPEED;
						as->acceLineForward.presentSetting = as->paramAgvforward.PresentRunning;
						if (as->areaOne)
							as->areaOne();
					}
					else if (as->agvForwardInfo.mlse.position[0] < 0.00)
					{
						as->acceLineForward.acceleration = REDUCE_SPEED;
						as->acceLineForward.presentSetting = as->paramAgvforward.PresentSlow;
						if (as->areaTwo)
							as->areaTwo();
					}
				}
					if(as->agvForwardInfo.status.track_level <=3 && as->agvForwardInfo.status.track_level >0)
					ROS_WARN("Back line sensor: track too weak!!");
			} 
			else 
			{
				as->acceLineForward.presentSetting = 0;
				ROS_ERROR("Front line sensor : no track!!");
			}	
		}
		loop_rate.sleep();
		ros::spinOnce();
	}
}

void agvline::EventbackwarddoStuff(void *payload)
{
	agvline *as = (agvline *)payload;
	ros::Rate loop_rate(as->threadRate);
	ROS_INFO("Event backward do Stuff is ready !!");
	while (ros::ok() && as->theadEnable == true)
	{
		if (as->agvBackwardInfo.mlse.error_register == 0)
		{
			if(as->agvBackwardInfo.status.line_good)
			{
				if (as->agvBackwardInfo.mlse.position[2] > 0.00 && as->agvBackwardInfo.mlse.lcp == 7)
				{
					//This->acceLineBackward.presentSetting = 0;
					if (as->areaThree)
						as->areaThree();
				}
				else if (as->agvBackwardInfo.mlse.position[2] <= 0.00)
				{
					if (as->agvBackwardInfo.mlse.position[0] == 0.00)
					{
						as->acceLineBackward.acceleration = UP_SPEED;
						as->acceLineBackward.presentSetting = as->paramAgvbackward.PresentRunning;
						if (as->areaOne)
							as->areaOne();
					}
					else if (as->agvBackwardInfo.mlse.position[0] < 0.00)
					{
						as->acceLineBackward.acceleration = REDUCE_SPEED;
						as->acceLineBackward.presentSetting = as->paramAgvbackward.PresentSlow;
						if (as->areaTwo)
							as->areaTwo();
					}
				}
				if(as->agvBackwardInfo.status.track_level <=3 && as->agvBackwardInfo.status.track_level >0)
					ROS_WARN("Back line sensor: track too weak!!");
			}
			else 
			{
				as->acceLineBackward.presentSetting = 0;
				ROS_ERROR("Back line sensor : no track!!");
			}
		}
		loop_rate.sleep();
		ros::spinOnce();
	}
}