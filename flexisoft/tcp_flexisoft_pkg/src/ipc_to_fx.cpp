#include <client.h>
#include <flexisoft_fx.h>
#include <flexisoft_msg.h>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>

#include <agv_define/agv_flexisoft.h>
#include <agv_define/agv_action.h>
#include <agv_define/agvlib.h>

bool value = 1;
bool data;

TCPClient *flexisoft = new TCPClient("192.168.1.11", 9101);
ros::Publisher fx_ipc_action_status_pub;
ros::Publisher st_io_action_pub;

void ipcFxActionStatusCallback(const agv_define::agv_action &st_io_action)
{
  ROS_INFO("ipc_to_fx.cpp-ipcFxActionStatusCallback()");
  bool check_action = true;
  uint16_t st_io_action_Action = st_io_action.action;
  uint16_t st_io_action_State = st_io_action.state;
  uint16_t st_io_action_Status = st_io_action.status;
  bool st_io_action_Load = st_io_action.load;
  float st_io_limit_trans = st_io_action.max_vel_trans;
  float st_io_limit_theta = st_io_action.max_vel_theta;
  bool *IPC_SAFE_SP_Limit;
  uint16_t st_speed_limit_SAFE_SP_Limit;
  agv_define::agv_action st_io_action_status;
  st_io_action_status = st_io_action;

  flexisoft->TCPClientReadInputGentArray(1);

  if (st_io_action_Action == 0 || st_io_action_Action == 1 || st_io_action_Action == 2 || ((st_io_action_Action == 5||st_io_action_Action==4)&&(st_io_action_Load==false)))
  {
    flexisoft->TCPClientWriteArrayBit16(SAFE_Do_Standstill, true); // THIS
    flexisoft->TCPClientWriteArrayBit16(SAFE_Do_Turn, false);
    flexisoft->TCPClientWriteArrayBit16(SAFE_Do_Forward_L, false);
    flexisoft->TCPClientWriteArrayBit16(SAFE_Do_Forward_R, false);
    flexisoft->TCPClientWriteArrayBit16(SAFE_Do_Charge, false);
    flexisoft->TCPClientWriteArrayBit16(SAFE_Do_Pick, false);
    if (flexisoft->TCPClientReadArrayBit(flexisoft->Fx_Read_Gent, SAFE_Do_Standstill) == 1)
    {
      st_io_action_status.action = st_io_action_Action;
    }
  }
  else if (st_io_action_Action == 3)
  {
    flexisoft->TCPClientWriteArrayBit16(SAFE_Do_Standstill, false);
    flexisoft->TCPClientWriteArrayBit16(SAFE_Do_Turn, true); //THIS
    flexisoft->TCPClientWriteArrayBit16(SAFE_Do_Forward_L, false);
    flexisoft->TCPClientWriteArrayBit16(SAFE_Do_Forward_R, false);
    flexisoft->TCPClientWriteArrayBit16(SAFE_Do_Charge, false);
    flexisoft->TCPClientWriteArrayBit16(SAFE_Do_Pick, false);
    if (flexisoft->TCPClientReadArrayBit(flexisoft->Fx_Read_Gent, SAFE_Do_Turn) == 1)
    {
      st_io_action_status.action = st_io_action_Action;
    }
  }

  else if (st_io_action_Action == 6 || st_io_action_Action == 7)
  {
    flexisoft->TCPClientWriteArrayBit16(SAFE_Do_Standstill, false);
    flexisoft->TCPClientWriteArrayBit16(SAFE_Do_Turn, false);
    flexisoft->TCPClientWriteArrayBit16(SAFE_Do_Forward_L, false);
    flexisoft->TCPClientWriteArrayBit16(SAFE_Do_Forward_R, false);
    flexisoft->TCPClientWriteArrayBit16(SAFE_Do_Charge, true); //THIS
    flexisoft->TCPClientWriteArrayBit16(SAFE_Do_Pick, false);
    if (flexisoft->TCPClientReadArrayBit(flexisoft->Fx_Read_Gent, SAFE_Do_Pick) == 1)
    {
      st_io_action_status.action = st_io_action_Action;
    }
  }
  else if (st_io_action_Action == 8 || st_io_action_Action == 9 || st_io_action_Action == 10 || st_io_action_Action==11||((st_io_action_Action == 5||st_io_action_Action==4)&&(st_io_action_Load==true)))
  {
    flexisoft->TCPClientWriteArrayBit16(SAFE_Do_Standstill, false);
    flexisoft->TCPClientWriteArrayBit16(SAFE_Do_Turn, false);
    flexisoft->TCPClientWriteArrayBit16(SAFE_Do_Forward_L, false);
    flexisoft->TCPClientWriteArrayBit16(SAFE_Do_Forward_R, false);
    flexisoft->TCPClientWriteArrayBit16(SAFE_Do_Charge, false);
    flexisoft->TCPClientWriteArrayBit16(SAFE_Do_Pick, true); //THIS
    if (flexisoft->TCPClientReadArrayBit(flexisoft->Fx_Read_Gent, SAFE_Do_Standstill) == 1)
    {
      st_io_action_status.action = st_io_action_Action;
    }
  }
  else
  {
    flexisoft->TCPClientWriteArrayBit16(SAFE_Do_Standstill, false);
    flexisoft->TCPClientWriteArrayBit16(SAFE_Do_Turn, false);
    flexisoft->TCPClientWriteArrayBit16(SAFE_Do_Forward_L, false);
    flexisoft->TCPClientWriteArrayBit16(SAFE_Do_Forward_R, false);
    flexisoft->TCPClientWriteArrayBit16(SAFE_Do_Charge, false);
    flexisoft->TCPClientWriteArrayBit16(SAFE_Do_Pick, false);
    if (flexisoft->TCPClientReadArrayBit(flexisoft->Fx_Read_Gent, SAFE_Do_Standstill) == 1)
    {
      st_io_action_status.action = st_io_action_Action;
    }
  }

  if (st_io_limit_theta >= st_io_limit_trans)
  {
    st_speed_limit_SAFE_SP_Limit = 100 * st_io_limit_theta;
    IPC_SAFE_SP_Limit = flexisoft->TCPClientDecToBin16(st_speed_limit_SAFE_SP_Limit);
  }
  else
  {
    st_speed_limit_SAFE_SP_Limit = 100 * st_io_limit_trans;
    IPC_SAFE_SP_Limit = flexisoft->TCPClientDecToBin16(st_speed_limit_SAFE_SP_Limit);
  }
  flexisoft->TCPClientWriteArrayBit16(SAFE_SP_Limit_0, IPC_SAFE_SP_Limit[0]);
  flexisoft->TCPClientWriteArrayBit16(SAFE_SP_Limit_1, IPC_SAFE_SP_Limit[1]);
  flexisoft->TCPClientWriteArrayBit16(SAFE_SP_Limit_2, IPC_SAFE_SP_Limit[2]);
  flexisoft->TCPClientWriteArrayBit16(SAFE_SP_Limit_3, IPC_SAFE_SP_Limit[3]);
  flexisoft->TCPClientWriteArrayBit16(SAFE_SP_Limit_4, IPC_SAFE_SP_Limit[4]);
  flexisoft->TCPClientWriteArrayBit16(SAFE_SP_Limit_5, IPC_SAFE_SP_Limit[5]);
  flexisoft->TCPClientWriteArrayBit16(SAFE_SP_Limit_6, IPC_SAFE_SP_Limit[6]);
  flexisoft->TCPClientWriteArrayBit16(SAFE_SP_Limit_7, IPC_SAFE_SP_Limit[7]);

  if (st_io_action_State == 0)
  {
    flexisoft->TCPClientWriteArrayBit16(SAFE_IO_Status_Pending, true);
    flexisoft->TCPClientWriteArrayBit16(SAFE_IO_Status_Active, false);
    flexisoft->TCPClientWriteArrayBit16(SAFE_IO_Status_Succeeded, false);
    flexisoft->TCPClientWriteArrayBit16(SAFE_IO_Status_Rejected, false);
    flexisoft->TCPClientWriteArrayBit16(SAFE_IO_Status_Lost, false);
    flexisoft->TCPClientWriteOutputGentArray(2);
    while (flexisoft->TCPClientReadArrayBit(flexisoft->Fx_Read_Gent, SAFE_Stt_Status_Active))
    {
      st_io_action_status.status = 1;
      break;
    }
  }
  else if (st_io_action_State == 1 || st_io_action_State == 2)
  {
    flexisoft->TCPClientWriteArrayBit16(SAFE_IO_Status_Pending, true);
    flexisoft->TCPClientWriteArrayBit16(SAFE_IO_Status_Active, true);
    flexisoft->TCPClientWriteArrayBit16(SAFE_IO_Status_Succeeded, true);
    flexisoft->TCPClientWriteArrayBit16(SAFE_IO_Status_Rejected, false);
    flexisoft->TCPClientWriteArrayBit16(SAFE_IO_Status_Lost, false);
    flexisoft->TCPClientWriteOutputGentArray(2);
    while (flexisoft->TCPClientReadArrayBit(flexisoft->Fx_Read_Gent, SAFE_IO_Status_Succeeded))
    {
      st_io_action_status.status = 3;
      break;
    }
  }
  st_io_action_pub.publish(st_io_action_status);  
  fx_ipc_action_status_pub.publish(st_io_action_status);
}
void IPCSafetyIoCallback(const agv_define::agv_flexisoft &ipc_st_io)
{
  ROS_INFO("ipc_to_fx.cpp- IPCSafetyIoCallback()");
  bool ipc_st_io_AGV_Run = ipc_st_io.IPC_SAFE_IO_AGV_Run;
  bool ipc_st_io_AGV_Brake = ipc_st_io.IPC_SAFE_IO_AGV_Brake;
  bool ipc_st_io_IPC_Ok = ipc_st_io.IPC_SAFE_IO_IPC_Ok;
  bool ipc_st_io_IPC_Error = ipc_st_io.IPC_SAFE_IO_IPC_Error;
  bool ipc_st_io_Reset_ALM_M = ipc_st_io.IPC_SAFE_IO_Reset_ALM_M;

  flexisoft->TCPClientWriteArrayBit16(SAFE_IO_AGV_Run, ipc_st_io_AGV_Run);
  flexisoft->TCPClientWriteArrayBit16(SAFE_IO_AGV_Brake, ipc_st_io_AGV_Brake);
  flexisoft->TCPClientWriteArrayBit16(SAFE_IO_IPC_Ok, ipc_st_io_IPC_Ok);
  flexisoft->TCPClientWriteArrayBit16(SAFE_IO_IPC_Error, ipc_st_io_IPC_Error);
  flexisoft->TCPClientWriteArrayBit16(SAFE_IO_Reset_ALM_M, ipc_st_io_Reset_ALM_M);
}
void SafetyMs3ResetCallback(const agv_define::agv_flexisoft &st_ms3_reset)
{

  bool st_ms3_reset_Ms3_F = st_ms3_reset.IPC_SAFE_MS3_Reset_MS3_F;
  bool st_ms3_reset_Ms3_B = st_ms3_reset.IPC_SAFE_MS3_Reset_MS3_B;

  flexisoft->TCPClientWriteArrayBit16(SAFE_MS3_Reset_MS3_F, st_ms3_reset_Ms3_F);
  flexisoft->TCPClientWriteArrayBit16(SAFE_MS3_Reset_MS3_B, st_ms3_reset_Ms3_B);
}
void SafetySpeedLimitCallback(const agv_define::agv_flexisoft &st_speed_limit)
{

  uint16_t st_speed_limit_SAFE_SP_Limit = 100 * st_speed_limit.IPC_SAFE_SP_Limit; //m/s to cm/s

  bool *IPC_SAFE_SP_Limit = flexisoft->TCPClientDecToBin16(st_speed_limit_SAFE_SP_Limit);

  flexisoft->TCPClientWriteArrayBit16(SAFE_SP_Limit_0, IPC_SAFE_SP_Limit[0]);
  flexisoft->TCPClientWriteArrayBit16(SAFE_SP_Limit_1, IPC_SAFE_SP_Limit[1]);
  flexisoft->TCPClientWriteArrayBit16(SAFE_SP_Limit_2, IPC_SAFE_SP_Limit[2]);
  flexisoft->TCPClientWriteArrayBit16(SAFE_SP_Limit_3, IPC_SAFE_SP_Limit[3]);
  flexisoft->TCPClientWriteArrayBit16(SAFE_SP_Limit_4, IPC_SAFE_SP_Limit[4]);
  flexisoft->TCPClientWriteArrayBit16(SAFE_SP_Limit_5, IPC_SAFE_SP_Limit[5]);
  flexisoft->TCPClientWriteArrayBit16(SAFE_SP_Limit_6, IPC_SAFE_SP_Limit[6]);
  flexisoft->TCPClientWriteArrayBit16(SAFE_SP_Limit_7, IPC_SAFE_SP_Limit[7]);
}

// void ipcFxActionStatusCallback(const agv_define::agv_action &data)
// {
//   agv_define::agv_action action_msg;
//   action_msg = data;

//   uint8_t action_ = ActionState(action_msg.action);
//   switch (action_)
//   {
//   case ACTION_FLOAT: //ACTION_FLOAT

//     break;
//   case ACTION_MANUAL: //ACTION_MANUAL

//     break;
//   case ACTION_INITIAL_POSE: //ACTION_INITIAL_POSE

//     break;
//   case ACTION_QUALITY_POSE: //ACTION_MANUAL

//     break;
//   case ACTION_ROTATE_GOAL: //ACTION_MANUAL

//     break;
//   case ACTION_MOVE_GOAL: //ACTION_MANUAL

//     break;
//   case ACTION_CHARGING_IN: //ACTION_MANUAL

//     break;
//   case ACTION_CHARGING_OUT: //ACTION_CHARGING_OUT

//     break;
//   case ACTION_LIFT_IN: //ACTION_MANUAL

//     break;
//   case ACTION_LIFT_UP: //ACTION_LIFT_UP

//     break;
//   case ACTION_LIFT_DOWN: //ACTION_LIFT_DOWN

//     break;
//   case ACTION_LIFT_OUT: //ACTION_LIFT_OUT

//     break;
//   }

//   action_msg.status = 1;
//   fx_ipc_action_status_pub.publish(action_msg);
// }

int main(int argc, char **argv)
{
  // create a modbus object

  // set slave id

  // connect with the server
  flexisoft->TCPClient_connect();

  agv_define::agv_flexisoft fx_st_io;
  agv_define::agv_flexisoft fx_st_fc;
  agv_define::agv_flexisoft st_ms3_field;
  agv_define::agv_flexisoft st_ms3_speed;

  ros::init(argc, argv, "safety_function");
  ros::NodeHandle nh;

  ros::Publisher fx_st_io_pub = nh.advertise<agv_define::agv_flexisoft>("/fx_safety_io", 10);
  ros::Publisher fx_st_fc_pub = nh.advertise<agv_define::agv_flexisoft>("/fx_st_fc", 10);
  ros::Publisher st_ms3_field_pub = nh.advertise<agv_define::agv_flexisoft>("/st_ms3_field", 10);
  ros::Publisher st_ms3_speed_pub = nh.advertise<agv_define::agv_flexisoft>("/st_ms3_speed", 10);
  st_io_action_pub = nh.advertise<agv_define::agv_action>("/st_io_action_status", 10);

 // ros::Subscriber st_io_action_sub = nh.subscribe("st_io_action", 1000, SafetyIoActionCallback);
  ros::Subscriber ipc_st_io_sub = nh.subscribe("ipc_st_io", 10, IPCSafetyIoCallback);
  ros::Subscriber st_ms3_reset_sub = nh.subscribe("st_ms3_reset", 1000, SafetyMs3ResetCallback);
  ros::Subscriber st_speed_limit_sub = nh.subscribe("st_speed_limit", 1000, SafetySpeedLimitCallback);

  fx_ipc_action_status_pub = nh.advertise<agv_define::agv_action>("/fx_ipc_action_status", 10);
  ros::Subscriber ipc_fx_action_status_sub = nh.subscribe("ipc_fx_action_status", 10, ipcFxActionStatusCallback);

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    uint st_io_action_status_action;
    uint st_io_action_status_status;

    flexisoft->TCPClientReadInputGentArray(1);
    fx_st_io.ST_SAFE_IO_GB_EMC = 1;
    // fx_st_io.ST_SAFE_IO_GB_EMC = flexisoft->TCPClientReadArrayBit(flexisoft->Fx_Read_Gent, SAFE_IO_GB_EMC);
    fx_st_io.ST_SAFE_IO_GB_Start = flexisoft->TCPClientReadArrayBit(flexisoft->Fx_Read_Gent, SAFE_IO_GB_Start);
    fx_st_io.ST_SAFE_IO_GB_Reset = flexisoft->TCPClientReadArrayBit(flexisoft->Fx_Read_Gent, SAFE_IO_GB_Reset);
    fx_st_io.ST_SAFE_IO_GB_Key_Brake = flexisoft->TCPClientReadArrayBit(flexisoft->Fx_Read_Gent, SAFE_IO_GB_Key_Brake);
    fx_st_io.ST_SAFE_IO_GB_EDM_MC = flexisoft->TCPClientReadArrayBit(flexisoft->Fx_Read_Gent, SAFE_IO_GB_EDM_MC);
    fx_st_io.ST_SAFE_IO_GB_ALM_M = flexisoft->TCPClientReadArrayBit(flexisoft->Fx_Read_Gent, SAFE_IO_GB_ALM_M);
    fx_st_io.ST_SAFE_IO_GB_IPC_Ok = flexisoft->TCPClientReadArrayBit(flexisoft->Fx_Read_Gent, SAFE_IO_GB_IPC_Ok);
    fx_st_io.ST_SAFE_IO_GB_IPC_Run = flexisoft->TCPClientReadArrayBit(flexisoft->Fx_Read_Gent, SAFE_IO_GB_IPC_Run);
    fx_st_io.ST_SAFE_IO_GB_FX_Run = flexisoft->TCPClientReadArrayBit(flexisoft->Fx_Read_Gent, SAFE_IO_GB_FX_Run);

    fx_st_io_pub.publish(fx_st_io);

    fx_st_fc.ST_SAFE_FC_ST_Ready = flexisoft->TCPClientReadArrayBit(flexisoft->Fx_Read_Gent, SAFE_FC_ST_Ready);
    fx_st_fc.ST_SAFE_FC_ST_G = flexisoft->TCPClientReadArrayBit(flexisoft->Fx_Read_Gent, SAFE_FC_ST_G);
    // fx_st_fc.ST_SAFE_FC_ST_G = false;
    fx_st_fc.ST_SAFE_FC_ST_B_G = flexisoft->TCPClientReadArrayBit(flexisoft->Fx_Read_Gent, SAFE_FC_ST_B_G);
    fx_st_fc.ST_SAFE_FC_SW_Ok = flexisoft->TCPClientReadArrayBit(flexisoft->Fx_Read_Gent, SAFE_FC_SW_Ok);
    fx_st_fc.ST_SAFE_FC_Auto_Restart = flexisoft->TCPClientReadArrayBit(flexisoft->Fx_Read_Gent, SAFE_FC_Auto_Restart);
    fx_st_fc.ST_SAFE_FC_EDM_E = flexisoft->TCPClientReadArrayBit(flexisoft->Fx_Read_Gent, SAFE_FC_EDM_E);
    fx_st_fc.ST_SAFE_FC_En_Ctr_ST = flexisoft->TCPClientReadArrayBit(flexisoft->Fx_Read_Gent, SAFE_FC_En_Ctr_ST);

    fx_st_fc_pub.publish(fx_st_fc);

    //  st_ms3_field.ST_SAFE_MS3_Field =
    st_ms3_field.ST_SAFE_MS3_Power_Ok = flexisoft->TCPClientReadArrayBit(flexisoft->Fx_Read_Gent, SAFE_MS3_Power_Ok);
    st_ms3_field.ST_SAFE_MS3_Brake_Ok = flexisoft->TCPClientReadArrayBit(flexisoft->Fx_Read_Gent, SAFE_MS3_Brake_Ok);
    st_ms3_field.ST_SAFE_MS3_Warn_Ok = flexisoft->TCPClientReadArrayBit(flexisoft->Fx_Read_Gent, SAFE_MS3_Warn_Ok);
    st_ms3_field.ST_SAFE_MS3_Detect_Ok = flexisoft->TCPClientReadArrayBit(flexisoft->Fx_Read_Gent, SAFE_MS3_Detect_Ok);
    st_ms3_field.ST_SAFE_MS3_Contour_Ok = flexisoft->TCPClientReadArrayBit(flexisoft->Fx_Read_Gent, SAFE_MS3_Contour_Ok);

    st_ms3_field_pub.publish(st_ms3_field);

    //   st_ms3_speed.ST_SAFE_MS3_SP =
    st_ms3_speed.ST_SAFE_MS3_SP_SStill = flexisoft->TCPClientReadArrayBit(flexisoft->Fx_Read_Gent, SAFE_MS3_SP_SStill);
    st_ms3_speed.ST_SAFE_MS3_SP_VSlow = flexisoft->TCPClientReadArrayBit(flexisoft->Fx_Read_Gent, SAFE_MS3_SP_VSlow);
    st_ms3_speed.ST_SAFE_MS3_SP_Slow = flexisoft->TCPClientReadArrayBit(flexisoft->Fx_Read_Gent, SAFE_MS3_SP_Slow);
    st_ms3_speed.ST_SAFE_MS3_SP_Medium = flexisoft->TCPClientReadArrayBit(flexisoft->Fx_Read_Gent, SAFE_MS3_SP_Medium);
    st_ms3_speed.ST_SAFE_MS3_SP_Fast = flexisoft->TCPClientReadArrayBit(flexisoft->Fx_Read_Gent, SAFE_MS3_SP_Fast);
    st_ms3_speed.ST_SAFE_MS3_SP_VFast = flexisoft->TCPClientReadArrayBit(flexisoft->Fx_Read_Gent, SAFE_MS3_SP_VFast);

    st_ms3_speed_pub.publish(st_ms3_speed);

    flexisoft->TCPClientWriteOutputGentArray(2);

    ros::spinOnce();
    loop_rate.sleep();
  }
  ros::spin();

  return 0;
}