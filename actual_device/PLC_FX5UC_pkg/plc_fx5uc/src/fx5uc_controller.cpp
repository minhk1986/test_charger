#include <ros/ros.h>
#include <mbtcp/modbus.h>
#include <plc_fx5uc/fx5uc_IO.h>
#include <fx5u_hardware/fx5u_hardware.h>
#include <boost/thread/thread.hpp>
#include <diagnostic_msgs/KeyValue.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include "agv_define/agv_action.h"
#include "agv_define/lift.h"
#include "agv_define/agvlib.h"

using namespace std;

#define NO_CL  0
#define RED    1
#define GREEN  2
#define YELLOW 3
#define BULE   4
#define PINK   5
#define OCEAN  6
#define WHITE  7

/* create a modbus object */
modbus *fx5uc;
/* create hardware */
FX5U_series device;
/***Create Node***/
ros::NodeHandlePtr nh;
/* Publisher */
ros::Publisher cmd_PLC;
ros::Publisher pub_IO_PLC;

/*global variable */

bool isLiftUp = false;
bool isLiftDown = true;
std::string action_;
uint8_t state_  = 0;
uint8_t status_ = 0;
uint8_t rate = 20; // Hz
static bool bitM_echo[300];
static bool bitM_pub[100];

/**********************************************************************
*                   		Define Function 
***********************************************************************/
uint8_t liftUp();
uint8_t liftDown();
void readPLC();
void writePLC();
/* Service */
bool liftServiceCallback(agv_define::lift::Request  &req, agv_define::lift::Response &res);
/* thread*/
void plcdoStuff(uint8_t *publish_rate);
/**********************************************************************
 *                   		MAIN 
***********************************************************************/
int main(int argc, char **argv)
{
  /* create ros ndoe */
  ros::init(argc, argv, "fx5uc_controller");
  nh= boost::make_shared<ros::NodeHandle>();
  fx5uc= new modbus(nh);
  boost::thread plcThread(plcdoStuff,&rate);
  plcThread.join();
  delete(fx5uc);
  ros::spin();
  return 0;
}  

/**********************************************************************
 *                   		Function detail 
***********************************************************************/
bool liftServiceCallback(agv_define::lift::Request  &req, agv_define::lift::Response &res)
{
	ROS_INFO("fx5uc_controller.cpp-21-liftServiceCallback()");
	action_ = req.action;
	state_ = State(req.state);
  if(action_ == "LIFT_UP"){
    if(state_ == STATE_EXECUTE){
      isLiftUp = true;
      isLiftDown = false;
      res.status = liftUp();
      // ROS_INFO("fx5uc_controller.cpp-34- isLiftUp = true");
    }else if(state_ == STATE_CANCEL){
      isLiftUp = false;
      res.status = 4;
    }
  }else if(action_ == "LIFT_DOWN"){
    if(state_ == STATE_EXECUTE){
      isLiftUp = false;
      isLiftDown = true;
      res.status = liftDown();
      // ROS_INFO("fx5uc_controller.cpp-34- isLiftUp = false");
    }else if(state_ == STATE_CANCEL){
      isLiftDown = false;
      res.status = 4;
    }
  }
  return true;
}
/**********************************************************************************************/
uint8_t liftUp(){
  ROS_INFO("fx5uc_controller.cpp-89-liftUp()");
  bitM_pub[16] = ON;
  ros::Rate loop(20);
  while(ros::ok() && (isLiftUp == true) && (isLiftDown == false)){   
    //ROS_INFO("fx5uc_controller.cpp-49- Start LIFT UP M12 = %d M200 = %d",bitM_echo[12],bitM_echo[200]);     
    readPLC();
    bitM_pub[12] = ON;  // LIFT_UP
    bitM_pub[14] = OFF;  // LIFT_DOWN
    status_ = 0; 
    if (bitM_echo[201] == ON){
      status_ = 3;
      isLiftUp = false;
      ROS_INFO("fx5uc_controller.cpp-89-Lift UP done");
      break;
    } 
    writePLC(); 
    loop.sleep();
    ros::spinOnce();
  }
  return status_;
}
/**********************************************************************************************/
uint8_t liftDown(){
  ROS_INFO("fx5uc_controller.cpp-89-liftDown()");
  bitM_pub[16] = ON;
  ros::Rate loop(20);
  while(ros::ok() && (isLiftUp == false) && (isLiftDown == true)){
    //ROS_INFO("fx5uc_controller.cpp-49- Start LIFT DOWN M14 = %d M201 = %d",bitM_echo[14],bitM_echo[201]);
    readPLC();
    bitM_pub[14] = ON;  // LIFT_DOWN
    bitM_pub[12] = OFF;  // LIFT_DOWN
    status_ = 0;

    if (bitM_echo[200] == ON){
      status_ = 3;
      isLiftDown = false;
      ROS_INFO("fx5uc_controller.cpp-89-Lift DOWN done");
      break;
    } 
    writePLC(); 
    loop.sleep();
    ros::spinOnce();
  }
  return status_;
}
/**********************************************************************************************/
void readPLC(){
  fx5uc->modbus_read_coils(Mbit, 300, bitM_echo);
  fx5uc->modbus_read_coils(PORT0, 7, device.y0);
  fx5uc->modbus_read_coils(PORT1, 7, device.y1);
  fx5uc->modbus_read_input_bits(PORT0, 7, device.x0);
  fx5uc->modbus_read_input_bits(PORT1, 7, device.x1);
}
/**********************************************************************************************/
void writePLC(){
  fx5uc->modbus_write_coils(Mbit, 100, bitM_pub); 
  fx5uc->modbus_write_register(0, device.D[1]);
}
/**********************************************************************************************/
void plcdoStuff(uint8_t *publish_rate)
{
  /* connect with the server */
  fx5uc->modbus_connect();
  ros::Rate loop_rate(*publish_rate);
  /* Publisher */
  cmd_PLC = nh->advertise<diagnostic_msgs::DiagnosticStatus>("PLC_infomation", 20);
  pub_IO_PLC = nh->advertise<plc_fx5uc::fx5uc_IO>("plc_IO",20);
  plc_fx5uc::fx5uc_IO pub_IO;
  /* Service*/
  ros::ServiceServer service = nh->advertiseService("lift_srv", liftServiceCallback);
  ROS_INFO("fx5uc_controller.cpp - Ready to lift_srv.");
  
  // diagnostic_msgs::DiagnosticArray dir_array;
  // diagnostic_msgs::DiagnosticStatus PLC;
  // diagnostic_msgs::KeyValue LED;
  // diagnostic_msgs::KeyValue Dock;
  // diagnostic_msgs::KeyValue Xilanh;
  // PLC.name = "PLC-Fx5UC";
	// PLC.hardware_id = "192.168.1.51:502";
  while(ros::ok())
  {
    readPLC();
    if(bitM_echo[0] == ON)// Nếu M0 on 
    {
      if(bitM_echo[5] == ON){
          device.D[1] = RED;
          bitM_pub[1] = ON;bitM_pub[2] = ON;bitM_pub[3] = OFF;
      } else if(bitM_echo[6] == ON){
          device.D[1] = YELLOW;
          bitM_pub[1] = ON;bitM_pub[2] = ON;bitM_pub[3] = OFF;
      } else if(bitM_echo[7]== ON){
          device.D[1] = YELLOW;
          bitM_pub[1] = ON;bitM_pub[2] = OFF;bitM_pub[3] = OFF;
      } else if(bitM_echo[8]== ON){
          device.D[1] = GREEN;
          bitM_pub[1] = ON;bitM_pub[2] = OFF;bitM_pub[3] = OFF;
      } else if(bitM_echo[9]== ON){
          device.D[1] = OCEAN;
          bitM_pub[1] = ON;bitM_pub[2] = OFF;bitM_pub[3] = OFF;
      }
      
      // if(bitM_echo[20] == ON) //sensor charging AGV
      // {
      //     // bitM_pub[18] = ON;   // send to PLC
      //     // ros::Duration(5).sleep(); 
      //     ROS_INFO("fx5uc_controller.cpp-89- Shutdown the IPC");
      //     //system("sudo shutdown now");
      // }
      writePLC(); 
    } else {
        ros::Duration(5).sleep(); 
        ROS_WARN("fx5uc_controller.cpp-80-not listen"); 
    }
    pub_IO.X04 = device.x0[4];
    pub_IO_PLC.publish(pub_IO);
    loop_rate.sleep();
    ros::spinOnce();
  }
  //close connection and free the memory
  fx5uc->modbus_close();
}