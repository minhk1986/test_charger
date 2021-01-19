#include <math.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/PoseStamped.h"
#include "sick_lidar_localization/SickLocResultPortTelegramMsg.h"

#include "agv_define/agvlib.h"
#include "agv_define/agv_action.h"
#include <actionlib/client/simple_action_client.h>
#include <agv_define/agvAction.h>  // Note: "Action" is appended


// Charger action
std::string action_charger_name = "charger_action";     // Name of action charger
typedef actionlib::SimpleActionClient<agv_define::agvAction> client_charger_action;


void callChargerAction(agv_define::agv_action msg);
void chargerActionDoneCb(const actionlib::SimpleClientGoalState& state, const agv_define::agvResultConstPtr& result);
void chargerActionActiveCb();
void chargerActionFeedbackCb(const agv_define::agvFeedbackConstPtr& feedback);

void serverAgvActionCallback(const agv_define::agv_action& msg){
    ROS_INFO("agv_main.cpp- serverAgvActionCallback()");
    
    uint8_t action_state_ = State(msg.state);
    switch(action_state_){
        case STATE_CANCEL:
            ROS_INFO("agv_main.cpp- STATE_CANCEL");
        break;
        case STATE_EXECUTE:
            uint8_t action_mode_ = ActionState(msg.action);
            switch(action_mode_){
                case ACTION_CHARGING_OUT:
                    ROS_INFO("agv_main.cpp- ACTION_CHARGING_OUT");
                    callChargerAction(msg);
                break;
                case ACTION_CHARGING_IN:
                    ROS_INFO("agv_main.cpp- ACTION_CHARGING_IN");
                    callChargerAction(msg);
                break;
            }
        break;
    }
}


// Charging Action
void callChargerAction(agv_define::agv_action msg){
    ROS_INFO("charger_client.cpp-callChargerAction()");
    client_charger_action client_charger_action_(action_charger_name, true);
    uint8_t time_out_ = 5;  // (s)
    bool wait_server_timeout_ = client_charger_action_.waitForServer(ros::Duration(time_out_));
    if (wait_server_timeout_){
        ROS_INFO("charger_client.cpp-callChargerAction() -> Connected to Charger Server");
    }else{
        ROS_INFO("charger_client.cpp-callChargerAction() -> Connect Charger server is timeout");
    }
    agv_define::agvGoal agvGoal_;
    agvGoal_.action = msg.action;
    client_charger_action_.sendGoal(agvGoal_, &chargerActionDoneCb, &chargerActionActiveCb, &chargerActionFeedbackCb);
    
    bool finished_charger_action_before_timeout_ = client_charger_action_.waitForResult(ros::Duration(300));
    if (!finished_charger_action_before_timeout_){
        ROS_ERROR("charger_client.cpp-callChargerAction() -> Charging action did not finish before the time out.");
    }    
}
void chargerActionDoneCb(const actionlib::SimpleClientGoalState& state, const agv_define::agvResultConstPtr& result){
    ROS_INFO("charger_client.cpp -> chargerActionDoneCb() -> Finished in state [%s]", state.toString().c_str());
    ROS_INFO("charger_client.cpp -> chargerActionDoneCb() -> Answer: %i", result->status);
}
void chargerActionActiveCb(){  // Called once when the goal becomes active
    ROS_INFO("charger_client.cpp -> chargerActionActiveCb() -> Active the Charging action");
}
void chargerActionFeedbackCb(const agv_define::agvFeedbackConstPtr& feedback){  // Called every time feedback is received for the goal
    ROS_INFO("charger_client.cpp -> chargerActionFeedbackCb() -> Got Feedback of percent_complete: %f", feedback->percent_complete);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "charger_client");
    ros::NodeHandle n;
    ROS_INFO("charger_client.cpp");

    // system('sudo rm -rf /home/robotics/.ros/log');
    char *command_type = (char *)"sudo rm -rf /home/robotics/.ros/log";
    system(command_type);

    ros::Subscriber server_agv_action_sub = n.subscribe("/agv_action", 10, serverAgvActionCallback);
    ROS_INFO("charger_client.cpp- Subscriber topic /agv_action");

    ros::spin();
    return 0;
}