
#ifndef DYNAMIXEL_WORKBENCH_VELOCITY_CONTROL_H
#define DYNAMIXEL_WORKBENCH_VELOCITY_CONTROL_H

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>

#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include <dynamixel_workbench_msgs/DynamixelStateList.h>
#include <dynamixel_workbench_msgs/WheelCommand.h>
#include <pro_arm_move/mx_joints.h>

class VelocityControl
{
 private:
  // ROS NodeHandle
  ros::NodeHandle node_handle_;

  // ROS Parameters

  // ROS Topic Publisher

  // ROS Topic Subscriber
  ros::Subscriber cmd_vel_sub_;

  // ROS Service Server
  ros::ServiceServer joints_command_server_;
    ros::Publisher   dynamixel_state_list_pub_a;
  // ROS Service Client

  // Dynamixel Workbench Parameters
  DynamixelWorkbench *dxl_wb_;
  uint8_t dxl_id_[4];
  uint8_t dxl_cnt_;

  // Other Parameters
  float wheel_separation_;
  float wheel_radius_;

 public:
  VelocityControl();
  ~VelocityControl();
 void dynamixelStatePublish();
 private:
  void initMsg();



  void initServer();
  bool jointsCommandMsgCallback(pro_arm_move::mx_joints::Request &req,
                                pro_arm_move::mx_joints::Response &res);
};

#endif //DYNAMIXEL_WORKBENCH_VELOCITY_CONTROL_H
