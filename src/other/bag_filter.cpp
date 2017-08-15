#include <ros/ros.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <math.h> 
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <message_filters/subscriber.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/tf.h>




using namespace std;
using namespace sensor_msgs;

#define _2PI	6.28318530717959
#define PI 3.14159265

class MyClass {
public:
	MyClass() :
		pcd_sub_( nh_, "/velodyne_points", 1 ),
		image_sub_( nh_, "/axis/image_raw_out", 1 ),
		odom_sub_( nh_, "/slam/odom", 1 ),
		sync( MySyncPolicy( 10 ), pcd_sub_, image_sub_, odom_sub_)
{
		sync.registerCallback( boost::bind( &MyClass::callback, this, _1, _2, _3) );
		image_pub = nh_.advertise<sensor_msgs::Image>("/axis/image_raw_out_f",1);
		pcd_pub = nh_.advertise<sensor_msgs::PointCloud2>("/velodyne_points_f",1);
		//odom_pub = nh_.advertise<nav_msgs::Odometry>("/slam/odom_f",1);

		msg_count = 0;
		lastOdometryPose.position.x = 0;
		lastOdometryPose.position.y = 0;
		lastOdometryPose.position.z = 0;
		lastOdometryPose.orientation.w = 1;
		lastOdometryPose.orientation.x = 0;
		lastOdometryPose.orientation.y = 0;
		lastOdometryPose.orientation.z = 0;

		printf ("Unesi poseTH (mm): ");
		scanf ("%lf",&poseTH);
		printf ("Unesi angleTH (deg): ");
		scanf ("%lf",&angleTH);
		//filter_br=5;
		printf("Filter initialized and ready for pcd images!... (%f, %f)\n",poseTH,angleTH);
}

	void callback(const sensor_msgs::PointCloud2ConstPtr& c_msg, const sensor_msgs::ImageConstPtr& i_msg, const nav_msgs::OdometryConstPtr& odom){
		printf("	%d. Robot's odometry pose = %7.2lf %7.2lf %7.2lf\n", odom->header.seq, 1000 * odom->pose.pose.position.x, 1000 * odom->pose.pose.position.y, tf::getYaw(odom->pose.pose.orientation));

		u[0] = 1000*(odom->pose.pose.position.x - lastOdometryPose.position.x);
		u[1] = 1000*(odom->pose.pose.position.y - lastOdometryPose.position.y);

		// logika da se izbjegne prekid Eulerovog kuta u PI
		double kut1 = tf::getYaw(odom->pose.pose.orientation);
		double kut2 = tf::getYaw(lastOdometryPose.orientation);

		if (kut1-kut2 > PI) {
			u[2] = kut1 - _2PI -kut2;
		}
		else if(kut1-kut2 < -PI) {
			u[2] = kut1 - kut2 + _2PI;
		}
		else {
			u[2] = kut1-kut2;
		}

		delta_pose = sqrt(pow(u[0],2)+pow(u[1],2));
		delta_angle = fabs(u[2]);

		printf("Deltas: %f %f\n",delta_pose,delta_angle);

		if((delta_pose>(poseTH)) || (delta_angle > (angleTH*PI/180))){
			lastOdometryPose.position = odom->pose.pose.position;
			lastOdometryPose.orientation = odom->pose.pose.orientation;
			pcd_pub.publish(*c_msg);
			image_pub.publish(*i_msg);
			printf("Published message %d.\n",msg_count);
			msg_count++;
		}
	}

private:
	ros::NodeHandle nh_;
	message_filters::Subscriber< sensor_msgs::PointCloud2 > pcd_sub_;
	message_filters::Subscriber< sensor_msgs::Image > image_sub_;
	message_filters::Subscriber< nav_msgs::Odometry > odom_sub_;
	double u[3],delta_pose,delta_angle,poseTH,angleTH;
	int msg_count;
	ros::Publisher image_pub,pcd_pub,odom_pub;
	typedef message_filters::sync_policies::ApproximateTime< sensor_msgs::PointCloud2, sensor_msgs::Image, nav_msgs::Odometry > MySyncPolicy;
	message_filters::Synchronizer< MySyncPolicy > sync;
	geometry_msgs::Pose lastOdometryPose;
};

int main(int argc, char** argv) {
	ros::init( argc, argv, "filter" );
	MyClass mc;

	while( ros::ok() ){
		ros::spin();
	}

	return EXIT_SUCCESS;
}

