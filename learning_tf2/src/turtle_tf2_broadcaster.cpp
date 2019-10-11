#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>

//this is needed so that we can use the ros tf broadcaster
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/TransformStamped.h>
#include <turtlesim/Pose.h>

std::string turtle_name;

void poseCallback(const turtlesim::PoseConstPtr& msg){
  //initialized and construct a tf broadcaster object
  static tf2_ros::TransformBroadcaster br;

  //intialized a tf container using geometry transformStamped message
  geometry_msgs::TransformStamped transformStamped;
  //set the time for each transform
  transformStamped.header.stamp = ros::Time::now();
  //the parent frame is 'world'
  transformStamped.header.frame_id = "world";
  
  //the child frame is the turtle
  transformStamped.child_frame_id = turtle_name;

  //this is a pointer to msg container inside transformedStamped object (msg*).x (msg*).y
  transformStamped.transform.translation.x = msg->x;
  transformStamped.transform.translation.y = msg->y;
  transformStamped.transform.translation.z = 0.0;
  
  //transformedStampe uses quaternion to represent rotation 
  tf2::Quaternion q;
  
  //this sets the quaternion from Roll pitch yaw variable
  q.setRPY(0, 0, msg->theta);

  //this can be used to set the rotation matrix using quaternions
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();

  br.sendTransform(transformStamped);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf2_broadcaster");

  ros::NodeHandle private_node("~");
  if (! private_node.hasParam("turtle"))
  {
    if (argc != 2){ROS_ERROR("need turtle name as argument"); return -1;};
    turtle_name = argv[1];
  }
  else
  {
    private_node.getParam("turtle", turtle_name);
  }
    
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe(turtle_name+"/pose", 10, &poseCallback);
  ros::spin();
  return 0;
};
