
#ifndef POSITION_CONTROL_MX
#define POSITION_CONTROL_MX

#include <ros/ros.h>

#include <vector>
#include <string>
#include <arm_ax_control/ax_joints.h>
#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include <sensor_msgs/JointState.h>

//namespace dynamixel
//{
#define ITERATION_FREQUENCY  (30)
#define JOINT_NUM   1
#define GRIPPER_NUM 0
#define DXL_NUM     1
#define PALM_NUM    0

class DynamixelController
{
 private:
  // ROS NodeHandle
  ros::NodeHandle node_handle_;
  ros::NodeHandle priv_node_handle_;

  // ROS Parameters

  // ROS Topic Publisher
  ros::Publisher joint_states_pub_;

  // ROS Topic Subscriber
  ros::Subscriber goal_joint_states_sub_;

  // ROS Service Server

  // ROS Service Client

  // Dynamixel Workbench Parameters
  std::string robot_name_;
  float protocol_version_;
  ros::ServiceServer joints_command_server_;
  DynamixelWorkbench *joint_controller_;

  std::vector<uint8_t> joint_id_;

  std::string joint_mode_;

 public:
  DynamixelController();
  ~DynamixelController();
  void jointStatePublish();
 private:

  void initMsg();
  void getDynamixelInst();
  void setOperatingMode();
  void setSyncFunction();
  void readPosition(double *value);
  void readVelocity(double *value);

  void initServer();
  bool jointsCommand(arm_ax_control::ax_joints::Request &req, arm_ax_control::ax_joints::Response &res);

};
//}

#endif //POSITION_CONTROL_MX
