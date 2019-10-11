
#include "position_controller_ax.h"

double mapd(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
  uint32_t profile_velocity ;
  uint32_t profile_acceleration;
DynamixelController::DynamixelController()
    :node_handle_(""),
     priv_node_handle_("~")
{
  robot_name_   = priv_node_handle_.param<std::string>("robot_name", "ed_manip");

  std::string device_name   = priv_node_handle_.param<std::string>("device_name_pro", "/dev/ttyUSB0");
  uint32_t dxl_baud_rate    = priv_node_handle_.param<int>("baud_rate_pro", 57600);
  protocol_version_         = priv_node_handle_.param<float>("protocol_version", 2.0);

  joint_states_pub_ = node_handle_.advertise<sensor_msgs::JointState>("/MX/joint_states", 5);
  joint_mode_   = priv_node_handle_.param<std::string>("joint_controller", "position_mode");

  joint_id_.push_back(priv_node_handle_.param<int>("joint1_id", 1));
  /*joint_id_.push_back(priv_node_handle_.param<int>("joint4_id", 4));
  joint_id_.push_back(priv_node_handle_.param<int>("joint5_id", 5));
  joint_id_.push_back(priv_node_handle_.param<int>("joint6_id", 6));*/

  profile_velocity     = priv_node_handle_.param<int>("profile_velocity", 50);
  profile_acceleration = priv_node_handle_.param<int>("profile_acceleration", 5);
  joint_controller_   = new DynamixelWorkbench;

  joint_controller_->begin(device_name.c_str(), dxl_baud_rate);

  getDynamixelInst();

  initServer();

  ROS_INFO("MX_controller : Init OK!");
}

DynamixelController::~DynamixelController()
{
  for (uint8_t num = 0; num < JOINT_NUM; num++)
    joint_controller_->itemWrite(joint_id_.at(num), "Torque_Enable", false);

  ros::shutdown();
}


void DynamixelController::initServer()
{
  joints_command_server_ = node_handle_.advertiseService("mx_joints_command", &DynamixelController::jointsCommand, this);
}
void DynamixelController::getDynamixelInst()
{
  uint16_t get_model_number;
  for (uint8_t index = 0; index < JOINT_NUM; index++)
  {

    if (joint_controller_->ping(joint_id_.at(index), &get_model_number) != true)
    {
      std::cout<<int(index)<<std::endl;
      ROS_ERROR("No MX Joints found, Please check id and baud rate");


      ros::shutdown();
      return;
    }
  }

  setOperatingMode();
  setSyncFunction();
}

void DynamixelController::setOperatingMode()
{
  if (joint_mode_ == "position_mode")
  {
    for (uint8_t num = 0; num < JOINT_NUM; num++)
      joint_controller_->jointMode(joint_id_.at(num), profile_velocity, profile_acceleration);
  }
  else if (joint_mode_ == "current_mode")
  {
    for (uint8_t num = 0; num < JOINT_NUM; num++)
      joint_controller_->currentMode(joint_id_.at(num));
  }
  else
  {
    for (uint8_t num = 0; num < JOINT_NUM; num++)
      joint_controller_->jointMode(joint_id_.at(num));
  }

}

void DynamixelController::setSyncFunction()
{
  joint_controller_->addSyncWrite("Goal_Position");

  if (protocol_version_ == 2.0)
  {
    joint_controller_->addSyncRead("Present_Position");
    joint_controller_->addSyncRead("Present_Velocity");
  }
}

bool DynamixelController::jointsCommand(arm_ax_control::ax_joints::Request &req,arm_ax_control::ax_joints::Response &res)
{
  double goal_joint_position[JOINT_NUM] = {0.0};//, 0.0, 0.0, 0.0};

    goal_joint_position[0] = req.joint_1;
    /*goal_joint_position[1] = req.joint_4;
    goal_joint_position[2] = req.joint_5;
    goal_joint_position[3] = req.joint_6;*/
    int32_t goal_position[JOINT_NUM] = {0, };

    int32_t max_position=4096,min_position=-4096;
    float min_radian=-6.283185307,max_radian=6.283185307;

  for (int index = 0; index < JOINT_NUM; index++)
  {
      if(index==1||index==3)
          goal_position[index] = joint_controller_->convertRadian2Value(goal_joint_position[index],min_position,max_position,min_radian,max_radian);
      else
          goal_position[index] = joint_controller_->convertRadian2Value(joint_id_.at(index), goal_joint_position[index]);

  }

  joint_controller_->syncWrite("Goal_Position", goal_position);
}

//=============================================================================================================================

//=============================================FOR JOINT STATE PUBLISH==========================================================


void DynamixelController::jointStatePublish()
{
  int32_t present_position[JOINT_NUM] = {0, };

  for (int index = 0; index < JOINT_NUM; index++)
    present_position[index] = joint_controller_->itemRead(joint_id_[index], "Present_Position");

  int32_t present_velocity[JOINT_NUM] = {0, };

  for (int index = 0; index < JOINT_NUM; index++)
    present_velocity[index] = joint_controller_->itemRead(joint_id_[index], "Present_Velocity");

  sensor_msgs::JointState dynamixel_;
  dynamixel_.header.stamp = ros::Time::now();

  for (int index = 0; index < JOINT_NUM; index++)
  {
    std::stringstream id_num;
    id_num << "id_" << (int)(joint_id_[index]);

    dynamixel_.name.push_back(id_num.str());

    int32_t max_position=4096,min_position=-4096;
    float min_radian=-6.283185307,max_radian=6.283185307;

    dynamixel_.position.push_back(joint_controller_->convertValue2Radian(present_position[index],min_position,max_position,min_radian,max_radian));
    dynamixel_.velocity.push_back(joint_controller_->convertValue2Velocity(joint_id_[index], present_velocity[index]));
  }
  joint_states_pub_.publish(dynamixel_);
}

//=============================================================================================================================
//=============================================================================================================================


int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "position_controller_mx");
  DynamixelController dynamixel_controller;
  ros::Rate loop_rate(ITERATION_FREQUENCY);

  while (ros::ok())
  {
    dynamixel_controller.jointStatePublish();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
