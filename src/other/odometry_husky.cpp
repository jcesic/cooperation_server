#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <cooperation_server/DifferentialSpeed.h>
#include <Eigen/Eigen> // whole Eigen matrix library (dense and sparse)
#include "tf_conversions/tf_eigen.h"

using namespace Eigen;

#define VEHICLE_TRACK 0.56
#define ROTATION_SLIP 1.828
#define _2PI	6.28318530717959
#define PI 3.14159265

#include <iostream>
#include <fstream>

#include <sensor_msgs/Imu.h>

#include <math.h>
#include <tf/tf.h>

using namespace std;

class imu_odometry{

private:
	ros::NodeHandle n;

public:
	FILE * pFile;
	int stojim,broj_poruka;
	bool prva,imu_data;
	double current_time_imu, last_time_imu, current_time_speed, last_time_speed, x, y, th, vx, omega, dt, th_current, delta_x, delta_y,th_delta,xdth,ydth;
	tf::Matrix3x3 Rot0;
	tf::Matrix3x3 CurrRot;
	Eigen::Matrix3d lRot;
	Eigen::Matrix4d Pose;
	ros::Publisher odom_pub;
	ros::Subscriber enc_sub,imu_sub;
	ros::Time time_g;

	imu_odometry(){
		current_time_imu = last_time_imu = current_time_speed = last_time_speed = 0;
		x = y = 0.0;
		xdth = ydth = 0.0;
		stojim=0;
		th =0;
		th_delta = 0.0;
		broj_poruka=0;
		Pose.setIdentity();
		imu_data = false;
		imu_sub=n.subscribe("/imu/data",500,&imu_odometry::imuCallback,this);
		enc_sub=n.subscribe("/husky/data/differential_speed",500,&imu_odometry::speedCallback,this);
		odom_pub=n.advertise<nav_msgs::Odometry>("/slam/odom",500);
		prva = true;
		CurrRot.setIdentity();
		ROS_INFO("Odometry started v2");
	}

	void imuCallback(const sensor_msgs::Imu::ConstPtr& msg_imu){

		tf::Quaternion q(msg_imu->orientation.x,
				msg_imu->orientation.y,
				msg_imu->orientation.z,
				msg_imu->orientation.w);
		CurrRot.setRotation(q);
		imu_data = true;

	}

	void speedCallback(const cooperation_server::DifferentialSpeed::ConstPtr& speed_msg){

		tf::Vector3 tran;
		Eigen::Matrix3d cRot,dRot;
		Eigen::Matrix4d frot;
		frot.setIdentity();
		tf::matrixTFToEigen(CurrRot,cRot);
		if(prva){
			lRot = cRot;
			prva = false;
		}
		dRot = lRot.inverse()*cRot;
		frot.block(0,0,3,3) = dRot;
		current_time_speed = speed_msg->header.stamp.sec+(speed_msg->header.stamp.nsec / 1e9);
		vx=(speed_msg->right_speed+speed_msg->left_speed)/2;
		if (speed_msg->right_speed==0 && speed_msg->left_speed==0){
			stojim++;
		}else{
			stojim=0;
		}
		dt = current_time_speed - last_time_speed;
		if(dt>5)
			dt=0;
		frot(0,3) = vx * dt;
		tran.setValue(0.0,0.0,delta_x);
		last_time_speed = current_time_speed;
		lRot = cRot;
		Pose = Pose*frot;
		tf::Quaternion q;
		tf::Matrix3x3 tmp_Rot;
		tf::matrixEigenToTF(Pose.block(0,0,3,3),tmp_Rot);
		tmp_Rot.getRotation(q);

		nav_msgs::Odometry odometry;
		odometry.header.stamp = speed_msg->header.stamp;// ros::Time::now();
		odometry.pose.pose.position.x = Pose(0,3);
		odometry.pose.pose.position.y = Pose(1,3);
		odometry.pose.pose.position.z = Pose(2,3);

		odometry.pose.pose.orientation.x = q.x();
		odometry.pose.pose.orientation.y = q.y();
		odometry.pose.pose.orientation.z = q.z();
		odometry.pose.pose.orientation.w = q.w();
		odom_pub.publish(odometry);
	}
};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "imu_odometry");
	imu_odometry *imuodometry = new imu_odometry();
	ros::spin();
	delete imuodometry;

	return 0;
}
