#include <stdio.h>
#include <time.h>
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <rosbag/bag.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/highgui.h>
#include "sensor_msgs/Image.h"
#include <sensor_msgs/image_encodings.h>
#include <string.h>
#include <math.h>
#include <fstream>
#include <sstream>

#ifndef _2PI
#define _2PI	6.28318530717959
#endif
#define PI			3.14159265358979

using namespace std;
using namespace sensor_msgs;
namespace enc = sensor_msgs::image_encodings;
using namespace cv;

class store_odom{

	char buffer[100];
	string f_pcl, f_odom, f_odom_t, f_pcl_t, f_img;
	boost::format pcl_format_,odom_format_, ground_format_, pcl_time_, odom_time_, img_format_;
	FILE *fp;
	ros::NodeHandle n_;
	ros::Subscriber odom_sub;
	ros::Time last_time;
	ros::Duration d_time;
	double theta;
	int num_odom;
	bool first_theta;
public:

	store_odom(){
		odom_format_.parse("%s/%s%05i-OZ.%s");
		odom_sub = n_.subscribe("stereo_odometer/odometry",1,&store_odom::callback,this);
		num_odom = 0;
		theta=0.0;
		first_theta = true;
	}

	void callback(const nav_msgs::OdometryConstPtr& odom){
		double x,y,z;
		x = odom->pose.pose.position.x;
		y = odom->pose.pose.position.y;
		z = odom->pose.pose.position.z;

		if(!first_theta){
			d_time = odom->header.stamp - last_time;
			theta = theta+d_time.toSec()*odom->twist.twist.angular.y;
		}else{
			first_theta = false;
			theta = 0.0;
		}
		if(theta>PI){
			theta = theta-2*PI;
		}
		if(theta<-PI){
			theta = theta +2*PI;
		}

		fp = fopen("/media/kruno/Data/TEST/Kiti/StereoOdometry05.txt","a");
		fprintf(fp,"%f %f %f\n",x,z,theta);
		fclose(fp);

		last_time = odom->header.stamp;

		f_odom = (odom_format_ % "/media/kruno/Data/TEST/Kiti/StereoOdom05" % "sl-" % num_odom % "txt").str();
		fp = fopen(f_odom.c_str(),"w");
		fprintf(fp,"%f %f %f\n",x,z,theta);
		fclose(fp);
		num_odom++;

		printf("Received odometry: %f %f %f %f %f %d\n",d_time.toSec(),x,y,z,theta*180/PI,num_odom);
	}
};

int main(int argc, char* argv[]){

	ros::init(argc, argv, "bagcopy");
	store_odom store_odom;
	ros::spin();
	return 0;
}
