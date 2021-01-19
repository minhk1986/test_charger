#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/PoseStamped.h"
#include "sick_lidar_localization/SickLocResultPortTelegramMsg.h"
#include <agv_define/agv_action.h>
#include <actionlib/server/simple_action_server.h>
#include <agv_define/agvAction.h>  // Note: "Action" is appended
#include <agv_define/agvlib.h>


std::string action_charger_name = "charger_action";     // Name of action charger
ros::Publisher cmd_vel_pub, pub_current_pose;
geometry_msgs::Twist cmd_vel;
geometry_msgs::PoseStamped agv_current_pose;
geometry_msgs::Quaternion getQuaternion(double yaw, double pitch, double roll);
void pubCurrentPose();
void pubCmdvel(geometry_msgs::Twist cmd_vel_);

class ChargingAction
{
    protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<agv_define::agvAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
    std::string action_name_;
    // create messages that are used to published feedback/result
    agv_define::agvFeedback feedback_;
    agv_define::agvResult result_;

    public:
    ChargingAction(std::string name) :
        as_(nh_, name, boost::bind(&ChargingAction::executeCB, this, _1), false), action_name_(name)
    {
        as_.start();
    }

    ~ChargingAction(void){}

    void executeCB(const agv_define::agvGoalConstPtr &goal){
        ros::Rate r(1);
        bool success = true;
        uint8_t num = goal->action;
        // start executing the action
        for(int i=1; i<=num; i++){
            // check that preempt has not been requested by the client
            if (as_.isPreemptRequested() || !ros::ok()){  // Check Cancel
                ROS_INFO("charger_server.cpp -> %s: Preempted", action_name_.c_str());
                as_.setPreempted();  // set the action state to preempted
                success = false;
                break;
            }
            float ratio_ = (100*i)/num;
            feedback_.percent_complete = ratio_;
            pubCurrentPose();
            pubCmdvel(cmd_vel);
            as_.publishFeedback(feedback_); // publish the feedback
            ROS_INFO("charger_server.cpp - executing action - publish the feedback -> percent_complete: %f", feedback_.percent_complete);
            // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
            r.sleep();
        }

        if(success){
            result_.status = 3;
            ROS_INFO("charger_server.cpp -> %s: Succeeded", action_name_.c_str());
            // set the action state to succeeded
            as_.setSucceeded(result_);
        }
    }
};

void currentPoseCallback(const sick_lidar_localization::SickLocResultPortTelegramMsg& msg){
    // ROS_INFO("agv_main.cpp- currentPoseCallback()");
    uint8_t quality = msg.telegram_payload.quality;
    if(quality < 80){
        ROS_ERROR_DELAYED_THROTTLE(2, "agv_main.cpp->Quality is LOW: %d", quality); // print every 1 seconds
    }else{
        ROS_INFO_ONCE("agv_main.cpp-Quality is GOOD: %d", quality);
    }

    agv_current_pose.header = msg.header;
    float x = float(msg.telegram_payload.posex)/1000;
    float y = float(msg.telegram_payload.posey)/1000;
    float th_degree = (msg.telegram_payload.poseyaw)/1000;
    float th_rad = (th_degree*M_PI)/180;

    agv_current_pose.pose.position.x = x;  // m
    agv_current_pose.pose.position.y = y;  // m
    agv_current_pose.pose.orientation = getQuaternion(th_rad, 0, 0);  // rad
}
geometry_msgs::Quaternion getQuaternion(double yaw, double pitch, double roll){   // yaw (Z), pitch (Y), roll (X)
    // Abbreviations for the various angular functions
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    geometry_msgs::Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
    return q;
}
void pubCurrentPose(){
    pub_current_pose.publish(agv_current_pose);
    ROS_INFO("agv_main.cpp- Publish agv_current_pose");
}
void pubCmdvel(geometry_msgs::Twist cmd_vel_){
    cmd_vel_pub.publish(cmd_vel_);
    ROS_INFO("agv_main.cpp- Publish cmd_vel");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "charger_server");
    ROS_INFO ("charger_server.cpp - Create node charger_server");
    ChargingAction agv(action_charger_name);

    ros::NodeHandle n;
    ros::Subscriber current_pose_sub = n.subscribe("/sick_lidar_localization/driver/result_telegrams", 10, currentPoseCallback);
    ROS_INFO("charger_server.cpp- Subscriber topic /sick_lidar_localization/driver/result_telegrams");

    pub_current_pose = n.advertise<geometry_msgs::PoseStamped>("/agv_current_pose", 10); 
    ROS_INFO("agv_main.cpp- Publish topic /agv_current_pose");
    cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10); 
    ROS_INFO("charger_server.cpp- Publish topic /cmd_vel");

    ros::spin();
    return 0;
}
