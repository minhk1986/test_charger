#include <ros/ros.h>
#include <cstdlib>
#include <unistd.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "sick_lidar_localization/SickLocColaTelegramSrv.h"
#include "sick_lidar_localization/SickLocSetPoseSrv.h"
#include "sick_lidar_localization/SickLocInitializePoseSrv.h"
#include "sick_lidar_localization/SickLocSetOdometryActiveSrv.h"
#include "sick_lidar_localization/SickLocSetOdometryPortSrv.h"


ros::ServiceClient client_SickLocColaTelegram;
ros::ServiceClient client_SickLocSetPose;
ros::ServiceClient client_SickLocInitializePose;
ros::ServiceClient client_LocSetOdometryActive;
ros::ServiceClient client_LocSetOdometryPort;
sick_lidar_localization::SickLocColaTelegramSrv srv;

double getYaw(double x, double y, double z, double w);
double radToDegree (double angle);
bool sickLocColaTelegram(std::string cola_ascii_request, float wait_response_timeout);
bool sickLocSetPose(int32_t posex, int32_t posey, int32_t yaw, int32_t uncertainty);
bool sickLocInitializePose(int32_t posex, int32_t posey, int32_t yaw, uint32_t sigmatranslation);
bool sickLocSetOdometryActive(bool active_);
bool sickLocSetOdometryPort(uint32_t port);

void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
    ROS_INFO("control_lidar_localization.cpp-23-initialPoseCallback()");
    geometry_msgs::PoseWithCovarianceStamped initPose; 
    initPose.pose = msg->pose;
    // ROS_INFO("control_lidar_localization.cpp-1200-initPose.x: %f", initPose.pose.pose.position.x);
    // ROS_INFO("control_lidar_localization.cpp-1201-initPose.y: %f", initPose.pose.pose.position.y);
    // ROS_INFO("control_lidar_localization.cpp-1202-initPose.z: %f", initPose.pose.pose.orientation.z);
    // ROS_INFO("control_lidar_localization.cpp-1203-initPose.w: %f", initPose.pose.pose.orientation.w);

    typedef int32_t my_int;
    my_int init_x = (rintf32)(initPose.pose.pose.position.x * 1000);
    my_int init_y = (rintf32)(initPose.pose.pose.position.y * 1000);
    my_int init_yaw = (rintf32)(getYaw(0, 0, initPose.pose.pose.orientation.z, initPose.pose.pose.orientation.w) * 1000);
    ROS_INFO("control_lidar_localization.cpp-35-init_x: %d", init_x);
    ROS_INFO("control_lidar_localization.cpp-36-init_y: %d", init_y);
    ROS_INFO("control_lidar_localization.cpp-37-init_yaw: %d", init_yaw);

    // std::stringstream ss;
    // ss << "rosservice call /SickLocSetPose {\"posex: " << init_x << ", posey: " << init_y << ", yaw: " << init_yaw << ", uncertainty: 1000\"}";
    // system(ss.str().c_str());
    // system("rosservice call /SickLocSetPose {\"posex: 2000, posey: 1000, yaw: 30000, uncertainty: 1000\"}");
    
    // Set initPose
    // if(!sickLocSetPose(init_x, init_y, init_yaw, 1000)){
    //     ROS_ERROR("control_lidar_localization.cpp- ERROR when set init Pose");
    // }

    // Set InitPose and auto calibration
    if(!sickLocInitializePose(init_x, init_y, init_yaw, 1000)){
        ROS_ERROR("control_lidar_localization.cpp- ERROR when set init Pose");
    }
    ROS_INFO("control_lidar_localization.cpp - Setting the initPose is successfully");
}
double getYaw(double x, double y, double z, double w) {
    ROS_INFO("control_lidar_localization.cpp-954-getYaw");
    // yaw (z-axis rotation)
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);
    yaw = radToDegree(yaw);
    return yaw;
}
double radToDegree (double angle){
    ROS_INFO("control_lidar_localization.cpp-40-radToDegree()");
    return (180*angle)/M_PI;
}

void enableLidarLocalization(){
    if(sickLocColaTelegram("sMN IsSystemReady", 1)){
        if(srv.response.cola_ascii_response == "sAN IsSystemReady 0"){
            ROS_ERROR("control_lidar_localization.cpp- ERROR when check lidar localization system");
        }else if(srv.response.cola_ascii_response == "sAN IsSystemReady 1"){
            if(sickLocColaTelegram("sRN LocState", 1)){
                if(srv.response.cola_ascii_response == "sRA LocState 0"){
                    ROS_ERROR("control_lidar_localization.cpp - ERROR when locState: BOOTING");
                }else if(srv.response.cola_ascii_response == "sRA LocState 1"){
                    ROS_ERROR("control_lidar_localization.cpp - locState mode: IDLE");
                    if(sickLocColaTelegram("sMN LocStartLocalizing", 1)){
                        ROS_INFO("control_lidar_localization.cpp- LocStartLocalizing is successfully");
                    }else{
                        ROS_ERROR("control_lidar_localization.cpp- ERROR when LocStartLocalizing");
                    }
                }else if(srv.response.cola_ascii_response == "sRA LocState 2"){
                    ROS_INFO("control_lidar_localization.cpp - locState mode: LOCALIZING");
                }else if(srv.response.cola_ascii_response == "sRA LocState 3"){
                    ROS_INFO("control_lidar_localization.cpp - locState mode: DEMO_MAPPING");
                }
            }
        }
    }else{
        ROS_ERROR("control_lidar_localization.cpp- ERROR when check lidar localization system");
    }
}

bool sickLocColaTelegram(std::string cola_ascii_request, float wait_response_timeout) {
    ROS_INFO("control_lidar_localization.cpp- sickLocColaTelegram() -> cola_ascii_request: %s", cola_ascii_request.c_str());
    srv.request.cola_ascii_request = cola_ascii_request;
    srv.request.wait_response_timeout = wait_response_timeout;
    if (client_SickLocColaTelegram.call(srv)){
        ROS_INFO("control_lidar_localization.cpp- cola_ascii_response: %s", srv.response.cola_ascii_response.c_str());
        ROS_INFO("control_lidar_localization.cpp- send_timestamp_sec: %d", srv.response.send_timestamp_sec);
        ROS_INFO("control_lidar_localization.cpp- send_timestamp_nsec: %d", srv.response.send_timestamp_nsec);
        ROS_INFO("control_lidar_localization.cpp- receive_timestamp_sec: %d", srv.response.receive_timestamp_sec);
        ROS_INFO("control_lidar_localization.cpp- receive_timestamp_nsec: %d", srv.response.receive_timestamp_nsec);
        return true;
    }else{
        ROS_ERROR("control_lidar_localization.cpp- Failed to call service /SickLocColaTelegram");
        return false;
    }
}
bool sickLocSetPose(int32_t posex, int32_t posey, int32_t yaw, int32_t uncertainty){
    // Chi set Init Pose
    ROS_INFO("control_lidar_localization.cpp- sickLocSetPose() -> posex: %d, posey: %d, yaw: %d", posex, posey, yaw);
    sick_lidar_localization::SickLocSetPoseSrv srv;
    srv.request.posex = posex;
    srv.request.posey = posey;
    srv.request.yaw = yaw;
    srv.request.uncertainty = uncertainty;
    if (client_SickLocSetPose.call(srv)){
        ROS_INFO("control_lidar_localization.cpp- success: %d", srv.response.success);
        return srv.response.success;
    }else{
        ROS_ERROR("control_lidar_localization.cpp- Failed to call service /SickLocSetPose");
        return false;
    }
}
bool sickLocInitializePose(int32_t posex, int32_t posey, int32_t yaw, uint32_t sigmatranslation){
    // Set Init Pose va tu dong hieu chinh vi tri
    ROS_INFO("control_lidar_localization.cpp- sickLocInitializePose() -> posex: %d, posey: %d, yaw: %d", posex, posey, yaw);
    sick_lidar_localization::SickLocInitializePoseSrv srv;
    srv.request.x = posex;
    srv.request.y = posey;
    srv.request.yaw = yaw;
    srv.request.sigmatranslation = sigmatranslation;
    if (client_SickLocInitializePose.call(srv)){
        ROS_INFO("control_lidar_localization.cpp- success: %d", srv.response.success);
        return srv.response.success;
    }else{
        ROS_ERROR("control_lidar_localization.cpp- Failed to call service /SickLocInitializePoseSrv");
        return false;
    }
}
bool sickLocSetOdometryActive(bool active_){
    // Anable/Disable Odometry
    ROS_INFO("control_lidar_localization.cpp- sickLocSetOdometryActive() -> active_: %d", active_);
    sick_lidar_localization::SickLocSetOdometryActiveSrv srv;
    srv.request.active = active_;
    if (client_LocSetOdometryActive.call(srv)){
        ROS_INFO("control_lidar_localization.cpp- sickLocSetOdometryActive -> Result port: %d", srv.response.set);
        ROS_INFO("control_lidar_localization.cpp- sickLocSetOdometryActive -> executed: %d", srv.response.executed);
        if(srv.response.set && srv.response.executed){
            return true;
        }
    }else{
        ROS_ERROR("control_lidar_localization.cpp- Failed to call service /SickLocSetOdometryPortSrv");
        return false;
    }
}
bool sickLocSetOdometryPort(uint32_t port_){
    // Set Odometry Port
    ROS_INFO("control_lidar_localization.cpp- sickLocSetOdometryPort() -> port: %d", port_);
    sick_lidar_localization::SickLocSetOdometryPortSrv srv;
    srv.request.port = port_;
    if (client_LocSetOdometryPort.call(srv)){
        ROS_INFO("control_lidar_localization.cpp- sickLocSetOdometryPort -> Result port: %d", srv.response.set);
        ROS_INFO("control_lidar_localization.cpp- sickLocSetOdometryPort -> executed: %d", srv.response.executed);
        if(srv.response.set && srv.response.executed){
            return true;
        }
    }else{
        ROS_ERROR("control_lidar_localization.cpp- Failed to call service /SickLocSetOdometryPortSrv");
        return false;
    }
}

bool loadParam(std::string node_name){
	ROS_INFO("control_lidar_localization.cpp-loadParam() - node_name: %s", node_name.c_str());
	
    int32_t odometryPort_;
	if(!ros::param::get(node_name + "/sick_lidar_localization/driver/odom_telegrams_udp_port", odometryPort_)){
		return false;
	}ROS_INFO("control_lidar_localization.cpp- odom_telegrams_udp_port: %d", odometryPort_);
    if(!sickLocSetOdometryPort(odometryPort_)){
        ROS_ERROR("control_lidar_localization.cpp- ERROR when set Odometry Port");
    }
    ROS_INFO("control_lidar_localization.cpp - Setting the Odometry Port is successfully");

    bool odom_enable_;
	if(!ros::param::get("odom_enable", odom_enable_)){
		return false;
	}ROS_INFO("blvd20km_controller.cpp- odom_enable: %d", odom_enable_);
    if(!sickLocSetOdometryActive(odom_enable_)){
        ROS_ERROR("control_lidar_localization.cpp- ERROR when set Odometry Active");
    }
    ROS_INFO("control_lidar_localization.cpp - Setting the Odometry Active is successfully");

	return true;
}

int main(int argc, char** argv){
    ROS_INFO("control_lidar_localization.cpp-35-main");

    ros::init(argc, argv, "control_lidar_localization");
    ros::NodeHandle n;

    client_SickLocColaTelegram = n.serviceClient<sick_lidar_localization::SickLocColaTelegramSrv>("SickLocColaTelegram");
    client_SickLocSetPose = n.serviceClient<sick_lidar_localization::SickLocSetPoseSrv>("SickLocSetPose");
    client_SickLocInitializePose = n.serviceClient<sick_lidar_localization::SickLocInitializePoseSrv>("SickLocInitializePose");
    client_LocSetOdometryActive = n.serviceClient<sick_lidar_localization::SickLocSetOdometryActiveSrv>("SickLocSetOdometryActive");
    client_LocSetOdometryPort = n.serviceClient<sick_lidar_localization::SickLocSetOdometryPortSrv>("SickLocSetOdometryPort");

    //Subscribe the topic /initialpose
    ros::Subscriber initial_pose_sub_ = n.subscribe("/initialpose", 10, initialPoseCallback);
    ROS_INFO("control_lidar_localization.cpp-85-Subscriber topic: /initialpose");

    enableLidarLocalization();

    std::string node_name = ros::this_node::getName();
	ROS_INFO("control_lidar_localization.cpp-node_name: %s", node_name.c_str());
	if(loadParam(node_name)){
		ROS_INFO("control_lidar_localization.cpp-Load parameter successfull");
	}else{
		ROS_ERROR("control_lidar_localization.cpp-Error when load parameter");
	}

    ros::spin();
    return(0);
}