#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main ( int argc, char** argv)
{
		ros::init (argc, argv, "basic_shapes");
		ros::NodeHandle n;
		ros::Rate r(1); 
		ros::Publisher marker_pub = n.advertise <visualization_msgs::Marker> ("visualization_marker",1);
		
		//set initial shape type to be a cube
		uint32_t shape = visualization_msgs::Marker::CUBE;

		while (ros::ok())
		{
				visualization_msgs::Marker marker;
				//set the frame ID and the timestamp. TF tutorial explains more.
				marker.header.frame_id = "/my_frame";
				marker.header.stamp = ros::Time::now();
				marker.type = shape;
				//set the namespace and id for this marker. This serves to create a new unique ID
				//Any marker sent with the same namespace and id will overwrite the old one
				marker.ns = "basic_shapes";
				marker.id = 0;

				//set the marker action. Options are ADD, DELETE. and DELETEALL
				marker.action = visualization_msgs::Marker::ADD;

				//set the pose of the marker. This is a full 6DOF pose relative to the frame/time 
				//specfified in the header
				
				marker.pose.position.x = 0;
				marker.pose.position.y = 0;
				marker.pose.position.z = 0;
				marker.pose.orientation.x = 0.0;
				marker.pose.orientation.y = 0.0;
				marker.pose.orientation.z = 0.0;
				marker.pose.orientation.w	= 1.0;
				
				//Set the scale of the direction

				marker.scale.x = 1.0;
				marker.scale.y = 1.0;
				marker.scale.z = 1.0;				
				//Set the color == be sure to set alpha to a non-zero value
				marker.color.r = 0.0f;
				marker.color.g = 1.0f;
				marker.color.b = 0.0f;
				marker.color.a = 1.0;
				
				marker.lifetime = ros::Duration();
				
				//Publish the marker
				while (marker_pub.getNumSubscribers() < 1)
				{
					if (!ros::ok())
					{
						return 0;
					}
					ROS_WARN_ONCE("Please create a subscriber to the marker");
					sleep(1);
				}
				
				marker_pub.publish(marker);
				
				//this is a nice algorithm to do a cycle switching
				switch(shape)
				{
						case visualization_msgs::Marker::CUBE:
							shape = visualization_msgs::Marker::SPHERE;
							break;
						case visualization_msgs::Marker::SPHERE:
							shape = visualization_msgs::Marker::ARROW;
							break;
						case visualization_msgs::Marker::ARROW:
							shape = visualization_msgs::Marker::CYLINDER;
							break;
						case visualization_msgs::Marker::CYLINDER:
							shape = visualization_msgs::Marker::CUBE;
							break;
				}

				r.sleep();
		}
}



