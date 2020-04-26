#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/Joy.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <chrono>
#include <sstream>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

using namespace std;

sensor_msgs::Joy controller_message;
bool controller_message_active=false;
//controller_message.axes[0]=0.0;

void Controller_msg_Handler(const sensor_msgs::Joy &message)
{
    controller_message = message;
    controller_message_active=true;    
}
control_msgs::FollowJointTrajectoryActionGoal Add_2nd_Double_Joint(moveit_msgs::RobotTrajectory Trajectory)
{
    auto it_joint_names = Trajectory.joint_trajectory.joint_names.begin();
    Trajectory.joint_trajectory.joint_names.insert(it_joint_names + 1, "joint2g");
    for (int i = 0; i < Trajectory.joint_trajectory.points.size(); i++)
    {
        auto it_pos = Trajectory.joint_trajectory.points[i].positions.begin();
        Trajectory.joint_trajectory.points[i].positions.insert(it_pos + 1, Trajectory.joint_trajectory.points[i].positions[2]);
        auto it_vel = Trajectory.joint_trajectory.points[i].velocities.begin();
        Trajectory.joint_trajectory.points[i].velocities.insert(it_vel + 1, Trajectory.joint_trajectory.points[i].velocities[2]);
        auto it_ac = Trajectory.joint_trajectory.points[i].accelerations.begin();
        Trajectory.joint_trajectory.points[i].accelerations.insert(it_ac + 1, Trajectory.joint_trajectory.points[i].accelerations[2]);
    }
    //Copy into a controller message
    control_msgs::FollowJointTrajectoryActionGoal ActionGoal;
    ActionGoal.goal.trajectory = Trajectory.joint_trajectory;
    return ActionGoal;
}

int main(int argc, char **argv)
{
    cout << "group to load" << endl;
    // Init the ROS node
    ros::init(argc, argv, "arm_ax_control_node");
    ros::NodeHandle ros_node_handler;
    ros::Rate loop_rate(3);
    
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Publisher pub_arm_commands_gazebo = ros_node_handler.advertise<control_msgs::FollowJointTrajectoryActionGoal>("/arm_controller/follow_joint_trajectory/goal", 5);
    ros::Publisher pub_arm_commands_dynamixel = ros_node_handler.advertise<control_msgs::FollowJointTrajectoryActionGoal>("/arm_controller_dynamixel/follow_joint_trajectory/goal", 1);

    //Controller
    ros::Subscriber sub_UAVmark = ros_node_handler.subscribe("joy", 1, Controller_msg_Handler);
    //MOVEIT GROUP
    sleep(1.0);
    static const std::string PLANNING_GROUP = "dual_arm";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    const robot_state::JointModelGroup *joint_model_group =
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    std::vector<double> joint_values(6);
    joint_values[0] = 0.0;
    joint_values[1] = -1.5;
    joint_values[2] = 1.5; //PI/2;
    joint_values[3] = 0.0; // PI/2;
    joint_values[4] = 0.0;
    joint_values[5] = 0.5; // PI/2;

    //Plan starting position
    move_group.setJointValueTarget(joint_values);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("Visualizing starting plan (Joint goal) %s", success ? "" : "FAILED");
    control_msgs::FollowJointTrajectoryActionGoal ActionGoal = Add_2nd_Double_Joint(my_plan.trajectory_);
    pub_arm_commands_gazebo.publish(ActionGoal);
    pub_arm_commands_dynamixel.publish(ActionGoal);
sleep(4.0);
    float XY_axes = 0.04;
    float Z_axes = 0.01;
    auto pose_current = move_group.getCurrentPose().pose;

    bool controller =false;

    while (controller)
    {
        if(controller_message_active)
        {
            pose_current.position.x -=  XY_axes * controller_message.axes[1];
            pose_current.position.y -=  XY_axes * controller_message.axes[0];
            pose_current.position.z +=  Z_axes * controller_message.axes[3];
        }
        controller_message_active=false;
        auto pose_new_command = pose_current;

        move_group.setPoseTarget(pose_new_command);
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        ROS_INFO_NAMED("Visualizing plan 1 (pose goal) %s", success ? "SUCCESS" : "FAILED");
        if(!success) {
            pose_current = move_group.getCurrentPose().pose;
            continue;
        }
        control_msgs::FollowJointTrajectoryActionGoal ActionGoal = Add_2nd_Double_Joint(my_plan.trajectory_);
        pub_arm_commands_gazebo.publish(ActionGoal);
        pub_arm_commands_dynamixel.publish(ActionGoal);
        const int last_index = ActionGoal.goal.trajectory.points.size();
        float waiting_time = ActionGoal.goal.trajectory.points[last_index-1].time_from_start.nsec * 1000000000.0;
        if(waiting_time > 2.0) waiting_time=2.0;
        sleep(waiting_time);
        loop_rate.sleep();
        ros::spinOnce();
    }

//Sample movements
    joint_values[0] = 0.6;
    joint_values[1] = -1.5;
    joint_values[2] = 1.5; 
    joint_values[5] = 1.2; // PI/2;
    move_group.setJointValueTarget(joint_values);
    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("Visualizing starting plan (Joint goal) %s", success ? "" : "FAILED");
     ActionGoal = Add_2nd_Double_Joint(my_plan.trajectory_);
    pub_arm_commands_gazebo.publish(ActionGoal);
    pub_arm_commands_dynamixel.publish(ActionGoal);
    int last_index = ActionGoal.goal.trajectory.points.size();
    float waiting_time = ActionGoal.goal.trajectory.points[last_index-1].time_from_start.nsec * 1000000000.0;
    if(waiting_time > 2.0) waiting_time=2.0;
    sleep(waiting_time);
    sleep(2.0);
    joint_values[0] = -0.6;
    joint_values[1] = -1.0;
    joint_values[2] = 1.0;
    joint_values[3] = 0.0; 
    joint_values[5] = -1.2;
    move_group.setJointValueTarget(joint_values);
    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("Visualizing starting plan (Joint goal) %s", success ? "" : "FAILED");
    ActionGoal = Add_2nd_Double_Joint(my_plan.trajectory_);
    pub_arm_commands_gazebo.publish(ActionGoal);
    pub_arm_commands_dynamixel.publish(ActionGoal);
    last_index = ActionGoal.goal.trajectory.points.size();
    waiting_time = ActionGoal.goal.trajectory.points[last_index-1].time_from_start.nsec * 1000000000.0;
    if(waiting_time > 2.0) waiting_time=2.0;
    sleep(waiting_time);
    sleep(2.0);
    joint_values[0] = 0.6;
    joint_values[1] = -1.0;
    joint_values[2] = 1.0; 
    joint_values[4] = 0.1;
    joint_values[5] = 1.3; 
    move_group.setJointValueTarget(joint_values);
    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    sleep(2.0);
    ROS_INFO_NAMED("Visualizing starting plan (Joint goal) %s", success ? "" : "FAILED");
    ActionGoal = Add_2nd_Double_Joint(my_plan.trajectory_);
    pub_arm_commands_gazebo.publish(ActionGoal);
    pub_arm_commands_dynamixel.publish(ActionGoal);
    last_index = ActionGoal.goal.trajectory.points.size();
    waiting_time = ActionGoal.goal.trajectory.points[last_index-1].time_from_start.nsec * 1000000000.0;
    if(waiting_time > 2.0) waiting_time=2.0;
    sleep(waiting_time);

    joint_values[0] = 0.0;
    joint_values[1] = -1.5;
    joint_values[2] = 1.5; 
    joint_values[4] = 0.0;
    joint_values[5] = 0.0; 
    move_group.setJointValueTarget(joint_values);
    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("Visualizing starting plan (Joint goal) %s", success ? "" : "FAILED");
    ActionGoal = Add_2nd_Double_Joint(my_plan.trajectory_);
    pub_arm_commands_gazebo.publish(ActionGoal);
    pub_arm_commands_dynamixel.publish(ActionGoal);
    last_index = ActionGoal.goal.trajectory.points.size();
    waiting_time = ActionGoal.goal.trajectory.points[last_index-1].time_from_start.nsec * 1000000000.0;
    if(waiting_time > 2.0) waiting_time=2.0;
    sleep(waiting_time);
    sleep(1.0);
}
