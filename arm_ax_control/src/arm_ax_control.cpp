#include "arm_ax_control.hpp"

double PI=3.141592654;
namespace arm_ax_control
{

FollowJointTrajectoryClient::FollowJointTrajectoryClient() :
    traj_client_("/earm_controller/follow_joint_trajectory", true), got_joint_state_(false), spinner_(0)
{
    joint_names_.push_back("joint1");
    joint_names_.push_back("joint2");
    joint_names_.push_back("joint3");
    joint_names_.push_back("joint4");
    joint_names_.push_back("joint5");
    joint_names_.push_back("joint6"); //PARA EARM


  joint_state_sub_ = nh_.subscribe("/joint_states", 1, &FollowJointTrajectoryClient::jointStateCB, this);
   spinner_.start();

  // wait for action server to come up
  while (!traj_client_.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the follow_joint_trajectory server");
  }
}

FollowJointTrajectoryClient::~FollowJointTrajectoryClient()
{
}

void FollowJointTrajectoryClient::jointStateCB(const sensor_msgs::JointState::ConstPtr &msg)
{
  std::vector<double> ordered_js;

  ordered_js.resize(joint_names_.size());

  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    bool found = false;
    for (size_t j = 0; j < msg->name.size(); ++j)
    {
      if (joint_names_[i] == msg->name[j])
      {
        ordered_js[i] = msg->position[j];
        found = true;
        break;
      }
    }
    if (!found)
      return;
  }

  ROS_INFO_ONCE("Got joint state!");
  current_joint_state_ = ordered_js;
  got_joint_state_ = true;
}

//! Sends the command to start a given trajectory
void FollowJointTrajectoryClient::startTrajectory(control_msgs::FollowJointTrajectoryGoal goal)
{
  // When to start the trajectory: 1s from now
  goal.trajectory.header.stamp = ros::Time::now();// + ros::Duration(0.1);

  //std::cout<<goal;
  traj_client_.sendGoal(goal);
}

control_msgs::FollowJointTrajectoryGoal FollowJointTrajectoryClient::makeArmUpTrajectory(std::vector<double> joints_obj)
{
  const size_t NUM_TRAJ_POINTS = 2;
  const size_t NUM_JOINTS = 6;

  // positions after calibration
  std::vector<double> mid_positions(NUM_JOINTS);
  mid_positions[0] = joints_obj[0];
  mid_positions[1] = joints_obj[1];
  mid_positions[2] = joints_obj[2];
  mid_positions[3] = joints_obj[3];
  mid_positions[4] = joints_obj[4];
  mid_positions[5] = joints_obj[5];

  // arm pointing straight up
  std::vector<double> straight_up_positions(NUM_JOINTS);
  straight_up_positions[0] = joints_obj[0];
  straight_up_positions[1] = joints_obj[1];
  straight_up_positions[2] = joints_obj[2];
  straight_up_positions[3] = joints_obj[3];
  straight_up_positions[4] = joints_obj[4];
  straight_up_positions[5] = joints_obj[5];

  trajectory_msgs::JointTrajectory trajectory;

  for (ros::Rate r = ros::Rate(10); !got_joint_state_; r.sleep())
  {
    ROS_DEBUG("waiting for joint state...");

    if (!ros::ok())
      exit(-1);
  }

  // First, the joint names, which apply to all waypoints
  trajectory.joint_names = joint_names_;

  trajectory.points.resize(NUM_TRAJ_POINTS);

  // trajectory point:
  int ind = 0;
  trajectory.points[ind].time_from_start = ros::Duration(0.05* ind);
  trajectory.points[ind].positions = current_joint_state_;

  // trajectory point:
  ind++;
  trajectory.points[ind].time_from_start = ros::Duration(0.1 * ind);
  trajectory.points[ind].positions.resize(NUM_JOINTS);
  trajectory.points[ind].positions = mid_positions;

  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory = trajectory;
  return goal;
}

//! Returns the current state of the action
actionlib::SimpleClientGoalState FollowJointTrajectoryClient::getState()
{
  return traj_client_.getState();
}

void FollowJointTrajectoryClient::PrintPose(std::string workspace,  geometry_msgs::Pose req_pose)
  {
    std::cout<<workspace<< ", Position x: "<< req_pose.position.x<<", y: "<<req_pose.position.y<<", z: "<<req_pose.position.z<<std::endl;
    std::cout<<workspace<< ", Orientation ow: "<< req_pose.orientation.w<<", oy: "<< req_pose.orientation.x<<", z: "<<req_pose.orientation.y<<", w: "<<req_pose.orientation.z<<std::endl;
    return;
  }
bool FollowJointTrajectoryClient::ReqMovement_byPose(geometry_msgs::Pose pose_req, robot_state::RobotStatePtr &kinematic_state, robot_state::JointModelGroup *joint_model_group, FollowJointTrajectoryClient &arm)
  {
    ros::Duration tiempo_traj(0.0);
    bool fk = kinematic_state->setFromIK(joint_model_group, pose_req, 2, 0.025);

    if (fk)
    {
      std::vector<double> jv(6);
      control_msgs::FollowJointTrajectoryGoal goale;
      kinematic_state->copyJointGroupPositions(joint_model_group, jv);

      goale = arm.makeArmUpTrajectory(jv);

      tiempo_traj = goale.trajectory.points[1].time_from_start; //tiempo del punto final
      auto delay_time = std::chrono::microseconds(int(tiempo_traj.toSec() * 1000000));
      arm.startTrajectory(goale); //Inicio de trayectoria en GAZEBO
    }
    else
    {

      PrintPose("NO solution for pose request!", pose_req);
    }
    return fk;
  }

}

double err_x,err_y,p_err_x,p_err_y,ctrl_x,ctrl_y;
double de_dt,e_dt=0;
double c_x,c_y;
float Kp=0.01,Ki=0.01,Kd=0.01;
int rate=3.0;
float period=1+1/rate;

struct quat {
    double x,y,z,w;
}quaternion;

struct quat toQuaternion(double yawMark, geometry_msgs::Pose cpose)
{            quat qc;

                      qc.x=cpose.orientation.x; qc.y=cpose.orientation.y;
                      qc.z=cpose.orientation.z; qc.w=cpose.orientation.w;
             double ysqrc = qc.y * qc.y;
                     double t3c = +2.0 * (qc.w * qc.z + qc.x * qc.y);
                     double t4c = +1.0 - 2.0 * (ysqrc + qc.z * qc.z);
                     double yawc = std::atan2(t3c, t4c);//end effector
                    double yaw=yawMark;

                             float yawc1,yaw1;
            if ( yawc >= 0  )       yawc1 =  yawc - PI ;
            if ( yawc < 0)       yawc1 =  PI + yawc;

            if ( yaw >= 0  )       yaw1 =  -(yaw - PI) ;
            if ( yaw < 0)       yaw1 =  -(PI + yaw);

                     yaw=yawc1-(0.2*yaw1);

                double roll=0;
                         double  pitch=PI;
        quat q1;
        double t0 = std::cos(yaw   * 0.5);
        double t1 = std::sin(yaw   * 0.5);
        double t2 = std::cos(roll  * 0.5);
        double t3 = std::sin(roll  * 0.5);
        double t4 = std::cos(pitch * 0.5);
        double t5 = std::sin(pitch * 0.5);

        q1.w = t0 * t2 * t4 + t1 * t3 * t5;
        q1.x = t0 * t3 * t4 - t1 * t2 * t5;
        q1.y = t0 * t2 * t5 + t1 * t3 * t4;
        q1.z = t1 * t2 * t4 - t0 * t3 * t5;
        return q1;
}
struct quat ConvtoQuaternion(double pitch, double roll, double yaw)
{
        quat q1;
        double t0 = std::cos(yaw * 0.5);
        double t1 = std::sin(yaw * 0.5);
        double t2 = std::cos(roll * 0.5);
        double t3 = std::sin(roll * 0.5);
        double t4 = std::cos(pitch * 0.5);
        double t5 = std::sin(pitch * 0.5);
        q1.w = t0 * t2 * t4 + t1 * t3 * t5;
        q1.x = t0 * t3 * t4 - t1 * t2 * t5;
        q1.y = t0 * t2 * t5 + t1 * t3 * t4;
        q1.z = t1 * t2 * t4 - t0 * t3 * t5;
        return q1;
}



struct quat ConvtoAngles(geometry_msgs::Pose mpose)
{
        quat q1;
        quat q;
                 q.x=mpose.orientation.x; q.y=mpose.orientation.y;
                 q.z=mpose.orientation.z; q.w=mpose.orientation.w;
                 double ysqr = q.y * q.y;
                // roll (x-axis rotation)
                double t00 = +2.0 * (q.w * q.x + q.y * q.z);
                double t11 = +1.0 - 2.0 * (q.x * q.x + ysqr);
                double roll = std::atan2(t00, t11);

                // pitch (y-axis rotation)
                double t22 = +2.0 * (q.w * q.y - q.z * q.x);
                t22 = t22 > 1.0 ? 1.0 : t22;
                t22 = t22 < -1.0 ? -1.0 : t22;
                double pitch = std::asin(t22);

                // yaw (z-axis rotation)
                double t33 = +2.0 * (q.w * q.z + q.x * q.y);
                double t44 = +1.0 - 2.0 * (ysqr + q.z * q.z);
                double yaw = std::atan2(t33, t44);
                q1.x=roll;q1.y=pitch;q1.z=yaw;
        return q1;
}
