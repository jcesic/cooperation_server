#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main( int argc, char** argv )
{
	ros::init(argc, argv, "basic_shapes");
	ros::NodeHandle n;
	ros::Rate r(100);
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	char buffer[100];

	//while (ros::ok())
	for(int i=0;i<688;i++)
	{

		visualization_msgs::Marker marker;

		// Set the frame ID and timestamp.  See the TF tutorials for information on these.
		marker.header.frame_id = "/base_link";
		marker.header.stamp = ros::Time::now();

		// Set the namespace and id for this marker.  This serves to create a unique ID
		// Any marker sent with the same namespace and id will overwrite the old one
		marker.ns = "basic_shapes";
		marker.id = i;

		// Set the marker type
		marker.type = visualization_msgs::Marker::MESH_RESOURCE;

		// Set the marker action.  Options are ADD and DELETE
		marker.action = visualization_msgs::Marker::ADD;

		// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
		marker.pose.position.x = 0;
		marker.pose.position.y = 0;
		marker.pose.position.z = 0;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		// Set the scale of the marker -- 1x1x1 here means 1m on a side
		marker.scale.x = 1.0;
		marker.scale.y = 1.0;
		marker.scale.z = 1.0;

		// Set the color -- be sure to set alpha to something non-zero!
		marker.color.r = 0.0f;
		marker.color.g = 1.0f;
		marker.color.b = 0.0f;
		marker.color.a = 1.0;

		marker.lifetime = ros::Duration();

		// Publish the marker
		snprintf(buffer, sizeof(buffer), "package://rvl_velodyne/models/%d_mod.stl",i);

		marker.mesh_resource = buffer;
		marker_pub.publish(marker);
		printf("poslano\n");

		r.sleep();
	}
}
