#include "velocity_control_ax.h"

VelocityControl::VelocityControl()
    :node_handle_(""),
     dxl_cnt_(4)
{
  std::string device_name   = node_handle_.param<std::string>("device_name_mx", "/dev/ttyUSB1");
  uint32_t dxl_baud_rate    = node_handle_.param<int>("baud_rate_mx", 57600);

  uint32_t profile_velocity     = node_handle_.param<int>("profile_velocity", 200);
  uint32_t profile_acceleration = node_handle_.param<int>("profile_acceleration", 100);
  dynamixel_state_list_pub_a = node_handle_.advertise<dynamixel_workbench_msgs::DynamixelStateList>("/MX/dynamixel_state", 1);

  dxl_id_[0] = node_handle_.param<int>("joint_3", 3);
  dxl_id_[1] = node_handle_.param<int>("joint_4", 4);
  dxl_id_[2] = node_handle_.param<int>("joint_5", 5);
  dxl_id_[3] = node_handle_.param<int>("joint_6", 6);

  dxl_wb_ = new DynamixelWorkbench;

  dxl_wb_->begin(device_name.c_str(), dxl_baud_rate);

  for (int index = 0; index < dxl_cnt_; index++)
  {
    uint16_t get_model_number;
    if (dxl_wb_->ping(dxl_id_[index], &get_model_number) != true)
    {
      ROS_ERROR("Not found Motors, Please check id and baud rate");

      ros::shutdown();
      return;
    }
  }

  initMsg();

  // Set Reverse Mode to Right Motor(ID : 2)
  //dxl_wb_->itemWrite(dxl_id_[1], "Drive_Mode", 1);//Drive mode 1 es negativo, controla la direccion de rotacion
  for (int index = 0; index < dxl_cnt_; index++)
    dxl_wb_->wheelMode(dxl_id_[index], profile_velocity, profile_acceleration);

  dxl_wb_->addSyncWrite("Goal_Velocity");

  initServer();
}

VelocityControl::~VelocityControl()
{
  for (int index = 0; index < 4; index++)
    dxl_wb_->itemWrite(dxl_id_[index], "Torque_Enable", 0);

  ros::shutdown();
}

void VelocityControl::initMsg()
{
  printf("-----------------------------------------------------------------------\n");
  printf("        dynamixel_workbench controller; velocity control example       \n");
  printf("              -This example supports MX2.0 and X Series-               \n");
  printf("-----------------------------------------------------------------------\n");
  printf("\n");

  for (int index = 0; index < dxl_cnt_; index++)
  {
    printf("MODEL   : %s\n", dxl_wb_->getModelName(dxl_id_[index]));
    printf("ID      : %d\n", dxl_id_[index]);
    printf("\n");
  }
  printf("-----------------------------------------------------------------------\n");
}

//========================================================================================================================

//========================================FOR JOINTS COMMAND SERVICE======================================================


void VelocityControl::initServer()
{
  joints_command_server_ = node_handle_.advertiseService("mx_joints_command", &VelocityControl::jointsCommandMsgCallback, this);

}

bool VelocityControl::jointsCommandMsgCallback(pro_arm_move::mx_joints::Request &req,
                                               pro_arm_move::mx_joints::Response &res)
{
  static int32_t goal_velocity[4] = {0, 0, 0, 0};

  goal_velocity[0] = dxl_wb_->convertVelocity2Value(dxl_id_[0], req.joint_3);
  goal_velocity[1] = dxl_wb_->convertVelocity2Value(dxl_id_[1], req.joint_4);
  goal_velocity[2] = dxl_wb_->convertVelocity2Value(dxl_id_[2], req.joint_5);
  goal_velocity[3] = dxl_wb_->convertVelocity2Value(dxl_id_[3], req.joint_6);

  bool ret = dxl_wb_->syncWrite("Goal_Velocity", goal_velocity);

  res.result = ret;
}

//========================================================================================================================

//=============================================FOR STATE PUBLISH==========================================================



void VelocityControl::dynamixelStatePublish()
{
  dynamixel_workbench_msgs::DynamixelState     dynamixel_state[dxl_cnt_];
  dynamixel_workbench_msgs::DynamixelStateList dynamixel_state_list;

  for (int index = 0; index < dxl_cnt_; index++)
  {
    dynamixel_state[index].model_name          = "a";
    dynamixel_state[index].id                  = index;
    dynamixel_state[index].torque_enable       = 1;
    dynamixel_state[index].present_position    = dxl_wb_->itemRead(dxl_id_[index], "Present_Position");//se borra el resto de datos porque genera retardos
    dynamixel_state[index].present_velocity    = 0;
    dynamixel_state[index].goal_position       = 0;
    dynamixel_state[index].goal_velocity       = 0;
    dynamixel_state[index].moving              = 0;

    dynamixel_state_list.dynamixel_state.push_back(dynamixel_state[index]);
  }
  dynamixel_state_list_pub_a.publish(dynamixel_state_list);

 // dynamixel_state_list_pub_.publish(dynamixel_state_list);
}



//========================================================================================================================

//=====================================================MAIN===============================================================


int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "velocity_control");
  VelocityControl vel_ctrl;

  ros::Rate loop_rate(50);

  while (ros::ok())
  {
      vel_ctrl.dynamixelStatePublish();
      ros::spinOnce();
      loop_rate.sleep();
  }

  return 0;
}
