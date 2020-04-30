#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>

void processFeedback(
		const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
	ROS_INFO_STREAM( feedback->marker_name << " is now at "
			<< feedback->pose.position.x << ", " 
			<< feedback->pose.position.y << ", "
			<< feedback->pose.position.z << );
}

int main (int argc, char** argv)
{
	ros::init(argc, argv, "simple_marker_node");

	// I create an interactive marker server on the topic namespace simple_marker
	interactive_markers::InteractiveMarkerServer server("simple_marker");

	//I create an interactive marker for our server
	visualization_msgs::InteractiveMarker int_marker;
	
