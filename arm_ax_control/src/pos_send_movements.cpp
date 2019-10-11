
#include <ros/ros.h>
#include <arm_ax_control/ax_joints.h>
#include <dynamixel_workbench_msgs/DynamixelStateList.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <geometry_msgs/Twist.h>

#define PI                          3.141592654
#define RadtoVal                    651.088403558
#define RadtoValPro54               250961.5/PI
#define RadtoValPro42               151875/PI

trajectory_msgs::JointTrajectory trajectory;
std::vector<double> joints_positions_req(1); //Inicialmente en rad
std::vector<double> joints_positions_reqM(1); //Inicialmente en rad
std::vector<double> mx_positions_state(1);//inicialmente en value
int key=0 ;
joints_positions_reqM=0.0;
void boundValue(float& val, float maxv)
{
    if (val>=maxv)
        val=maxv;
    else if(val<=-maxv)
        val=-maxv;
}

void Key_Handler(const geometry_msgs::Twist& key_sc)
{

 if (key_sc.angular.y==1.0)
     key=1;
 else if (key_sc.angular.y==2.0)
     key=2;
 else
     key=0;

}

void All_Joints_Data_Handler(const control_msgs::FollowJointTrajectoryGoal& goal)
{
  trajectory = goal.trajectory;
  joints_positions_req = trajectory.points[0].positions; //Main request from ed_pmov
}

void AX_Joints_State_Handler(const dynamixel_workbench_msgs::DynamixelStateList& mx_state)
{
  for(int i=0;i<1;i++)
  {
      mx_positions_state[i]=mx_state.dynamixel_state[i].present_position;//Rad-State of MX joints
  }
}


void ax_velocity_calculation(std::vector<double> joints_positions_reqA,  arm_ax_control::ax_joints& ax_joints_command)
{

    ax_joints_command.request.joint_1  = joints_positions_reqA[0];
    /*ax_joints_command.request.joint_2  = joints_positions_reqA[1];
    ax_joints_command.request.joint_3  = joints_positions_reqA[2];
    ax_joints_command.request.joint_4  = joints_positions_reqA[3];
    ax_joints_command.request.joint_5  = joints_positions_reqA[4];
    ax_joints_command.request.joint_6  = joints_positions_reqA[5];*/

}

int calculate_joints_mov(int key, arm_ax_control::ax_joints& ax_joints_command)
{
    int escape=0;
    if (key == 5)  //parar programa
     {
        return escape=1;
     }
    else if (key == 1|| key==2) //Hombre muerto tecla g
    {

        ax_joints_command.request.joint_1  += 0.0;//dont move
       /* ax_joints_command.request.joint_2  += 0.0;
        ax_joints_command.request.joint_3  += 0.0;
        ax_joints_command.request.joint_4  += 0.0;
        ax_joints_command.request.joint_5  += 0.0;
        ax_joints_command.request.joint_6  += 0.0;*/
    }
    else
    {
        ax_velocity_calculation(joints_positions_reqM,ax_joints_command);

    }

    return escape;

}


int main(int argc, char **argv)
{
    // Init ROS node
    ros::init(argc, argv, "pos_send_movements");

    arm_ax_control::mx_joints   mx_joints_command;


    ros::NodeHandle node_handle;

    ros::Subscriber Joints_data_req = node_handle.subscribe("/joints_data", 1, All_Joints_Data_Handler);//recibo mensajes de p_mov TODAS LAS JOINTS
    ros::Subscriber MX_Joints_data_state = node_handle.subscribe("/AX/dynamixel_state", 1, AX_Joints_State_Handler);//recibo mensajes de velocity_control_mx SOLO MX JOINTS

    ros::Subscriber keyboard =   node_handle.subscribe("/turtlebot_teleop/cmd_vel", 1, Key_Handler);

    ros::ServiceClient ax_joints_command_client = node_handle.serviceClient<arm_ax_control::mx_joints>("/ax_joints_command");
    ros::Rate loop_rate(30);
    bool sendcommands=true;
    sleep(3.0);
    while(ros::ok())
    {
           
          int escape=0;
            escape = calculate_joints_mov(key, ax_joints_command);
        if (escape==1)
        {
            sendcommands = false;
            break;
        }
        if (sendcommands)
        { 
            joints_positions_reqM += 0.001;
            if ( ax_joints_command_client.call(mx_joints_command))
            {
                if (mx_joints_command.response.result)
                    ROS_INFO("Succeed to write ax_goal_position");
                else
                    ROS_WARN("Failed to write ax_goal_position, receive response failed");
            }
            else
            {
                if (!ax_joints_command_client.call(mx_joints_command))
                    ROS_ERROR("Failed to call service /ax_joints_command");
            }
        }


        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

