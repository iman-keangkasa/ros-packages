#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <cmath>

int main (int argc, char** argv)
{
	ros::init(argc, argv, "points_and_lines_node");
	ros::NodeHandle n;
	ros::Publisher marker_pub = n.advertise <visualization_msgs::Marker> ("visualization_marker",10);
	
	ros::Rate r(30);
	float f = 0.0;
	while (ros::ok())
	{
		visualization_msgs::Marker points, line_strips, line_list;
		points.header.frame_id = line_strips.header.frame_id = line_list.header.frame_id = "/my_frame";
		points.header.stamp = line_strips.header.stamp = line_list.header.stamp = ros::Time::now();
		points.ns = line_strips.ns = line_list.ns = "points_and_lines";
		points.action = line_strips.action = line_list.action = visualization_msgs::Marker::ADD;
		points.pose.orientation.w = line_strips.pose.orientation.w = line_list.pose.orientation.w = 1.0;

		points.id = 0;
		line_strips.id = 1;
		line_list.id = 2;

		points.type = visualization_msgs::Marker::POINTS;
		line_strips.type = visualization_msgs::Marker::LINE_STRIP;
		line_list.type = visualization_msgs::Marker::LINE_LIST;
		
		//POINTS marker uses x and y scale for width/height respectively
		points.scale.x = 0.1;
		points.scale.y = 0.1;

		//LINE_STRIP and LINE_LIST markers use only the x component of scale for 
		//the line width
		line_strips.scale.x = 0.1;
		line_list.scale.x = 0.1;

		//Points uses green color
		points.color.g =1.0f;
		points.color.a = 1.0;

		//Line strip is blue:
		line_strips.color.b = 1.0;
		line_strips.color.a = 1.0f; //[TODO] Change to 1.0?

		//Line list is red
		line_list.color.r = 1.0;
		line_list.color.a = 1.0;

		//Creating vertices using for loop 
		for (uint32_t i = 0; i < 100; ++i)
		{
			float y = 5 * sin(f + i /100.0f * 2 * M_PI);
			float z = 5 * cos(f + i/100.0f * 2 * M_PI);
			
			geometry_msgs::Point p;
			p.x = (int32_t)i -50;
			p.y = y;
			p.z = z;

			points.points.push_back(p);
			line_strips.points.push_back(p);

			//The line list needs two points for each line
			line_list.points.push_back(p);
			p.z +=2.0;
			line_list.points.push_back(p);
		}

		//publish all  
		marker_pub.publish(points);
		marker_pub.publish(line_strips);
		marker_pub.publish(line_list);

		r.sleep();
		f+= 0.04;
	}
}




		
