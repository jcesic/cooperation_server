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
	double alpha,beta,theta,dtheta,dbeta,dalpha;
	int num_odom;
	bool first_theta;
public:

	store_odom(){
		odom_format_.parse("%s/%s%05i-OZ.%s");
		odom_sub = n_.subscribe("stereo_odometer/odometry",1,&store_odom::callback,this);
		num_odom = 0;
		alpha = 0.0;
		beta = 0.0;
		theta=0.0;
		first_theta = true;
	}

	void check_angle(double &th){
		if(th>PI){
			th = th-2*PI;
		}else if(th<-PI){
			th = th +2*PI;
		}

	}

	void callback(const nav_msgs::OdometryConstPtr& odom){
		double x,y,z,dx,dy,dz;
		x = odom->pose.pose.position.x;
		y = odom->pose.pose.position.y;
		z = odom->pose.pose.position.z;
		dx = odom->pose.pose.orientation.x;
		dy = odom->pose.pose.orientation.y;
		dz = odom->pose.pose.orientation.z;

		/*tf::Quaternion delta_rot;
		delta_rot.setX(odom->pose.pose.orientation.x);
		delta_rot.setY(odom->pose.pose.orientation.y);
		delta_rot.setZ(odom->pose.pose.orientation.z);
		delta_rot.setW(odom->pose.pose.orientation.w);

		tfScalar angle = delta_rot.getAngle();
		tf::Vector3 axis = delta_rot.getAxis();*/

		double dangle = odom->twist.twist.linear.x;
		tf::Vector3 daxis;
		daxis.setX(odom->twist.twist.angular.x);
		daxis.setY(odom->twist.twist.angular.y);
		daxis.setZ(odom->twist.twist.angular.z);

		/*if(angle==0){
			 theta = 2*atan2((axis.x()*sin(angle/2)),cos(angle/2));
			 beta= 0;
		}else{
			theta = atan2(axis.y()*sin(angle)-axis.x()*axis.z()*(1-cos(angle)), 1-(pow(axis.y(),2) + pow(axis.z(),2))*(1-cos(angle)));
			beta = atan2(axis.x()*sin(angle)-axis.y()*axis.z()*(1-cos(angle)),1-(pow(axis.x(),2) + pow(axis.z(),2)) * (1 - cos(angle)));
		}
		alpha = asin(axis.x()*axis.y()*(1-cos(angle))+axis.z()*sin(angle));*/

		check_angle(theta);
		check_angle(alpha);
		check_angle(beta);

		if(dangle==0){
			 dbeta = 2*atan2((daxis.x()*sin(dangle/2)),cos(dangle/2));
			 dalpha= 0;
		}else{
			dbeta = atan2(daxis.y()*sin(dangle)-daxis.x()*daxis.z()*(1-cos(dangle)), 1-(pow(daxis.y(),2) + pow(daxis.z(),2))*(1-cos(dangle)));
			dalpha = atan2(daxis.x()*sin(dangle)-daxis.y()*daxis.z()*(1-cos(dangle)),1-(pow(daxis.x(),2) + pow(daxis.z(),2)) * (1 - cos(dangle)));
		}
		dtheta = asin(daxis.x()*daxis.y()*(1-cos(dangle))+daxis.z()*sin(dangle));

		check_angle(dtheta);
		check_angle(dalpha);
		check_angle(dbeta);

		fp = fopen("/media/kruno/Data/TEST/Kiti/StereoOdometry003d.txt","a");
		fprintf(fp,"%f %f %f %f %f %f %f %f %f %f %f %f %f %f\n",x,y,z,dx,dy,dz,dalpha,dbeta,dtheta,odom->twist.twist.linear.y,odom->pose.pose.orientation.x,odom->pose.pose.orientation.y,odom->pose.pose.orientation.z,odom->pose.pose.orientation.w);
		fclose(fp);

		last_time = odom->header.stamp;

		f_odom = (odom_format_ % "/media/kruno/Data/TEST/Kiti/StereoOdom003d" % "sl-" % num_odom % "txt").str();
		fp = fopen(f_odom.c_str(),"w");
		fprintf(fp,"%f %f %f %f %f %f %f %f %f\n",dx,dy,dz,dalpha,dbeta,dtheta,x,y,z);
		fclose(fp);
		num_odom++;

		printf("Received odometry: %f %f %f %f %f %f %f %d\n",d_time.toSec(),x,y,z,alpha*180/PI,theta*180/PI,beta*180/PI,num_odom);
	}
};

int main(int argc, char* argv[]){

	ros::init(argc, argv, "bagcopy");
	store_odom store_odom;
	ros::spin();
	return 0;
}

