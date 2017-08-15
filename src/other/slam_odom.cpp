#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <iostream>
#include <fstream>
#include <sensor_msgs/Imu.h>
#include <math.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>


#define _2PI	6.28318530717959
#define PI 3.14159265

using namespace std;

class slam_odometry{

private:
	ros::NodeHandle n;

public:
	FILE * pFile;
	geometry_msgs::PoseWithCovarianceStamped slam_pose;
	nav_msgs::Odometry slam_odom;
	bool nasao;
	nav_msgs::Odometry povijest[50];
	nav_msgs::Odometry upose;
	int br_buffer;
	int buff_size;

	ros::Publisher odom_pub;
	ros::Subscriber slam_sub, odom_sub;

	double u[3],kut_slam,kut_upose,delta_kut,c_delta,s_delta,kut1;

	slam_odometry(){
		slam_sub=n.subscribe("/SLAM/pose",500,&slam_odometry::slamCallback,this);
		odom_sub=n.subscribe("/slam/odom",500,&slam_odometry::odomCallback,this);
		odom_pub=n.advertise<nav_msgs::Odometry>("/odom",500);
		br_buffer = 0;
		buff_size = 50;
		ROS_INFO("SLAM Odometry started v21");
	}

	void slamCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg_slam){
		slam_pose = *msg_slam;
		nasao = false;
		for(int i = 0; i<buff_size; i++){
			if ((int)slam_pose.pose.pose.position.z==povijest[i].header.seq){
				upose = povijest[i];
				nasao=true;
				break;
			}
		}
		if(nasao){
			kut_slam = tf::getYaw(slam_pose.pose.pose.orientation);
			kut_upose = tf::getYaw(upose.pose.pose.orientation);
			delta_kut = kut_slam-kut_upose;
			if (delta_kut > PI) {
				delta_kut = delta_kut - _2PI;
			}else if(delta_kut < -PI) {
				delta_kut = delta_kut + _2PI;
			}
			c_delta = cos(delta_kut);
			s_delta = sin(delta_kut);
		}else{
			ROS_ERROR("Unable to synchronize slam and odom, buffer to small...\n");
		}

		//printf("Primio slam pozu id: %f\n",slam_pose.pose.pose.position.z);
	}

	void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg){
		povijest[br_buffer]=*odom_msg;
		br_buffer++;
		if(br_buffer>(buff_size-1))
			br_buffer = 0;

		//printf("Primio odom pozu id: %d\n", odom_msg->header.seq);
		u[0] = (odom_msg->pose.pose.position.x - upose.pose.pose.position.x);
		u[1] = (odom_msg->pose.pose.position.y - upose.pose.pose.position.y);

		kut1 = tf::getYaw(odom_msg->pose.pose.orientation);

		u[2] = kut1-kut_upose;

		if (u[2] > PI) {
			u[2] = u[2] - _2PI;
		}else if(u[2] < -PI) {
			u[2] = u[2] + _2PI;
		}

		slam_odom.header.stamp = odom_msg->header.stamp;
		slam_odom.header.frame_id = odom_msg->header.frame_id;

		slam_odom.pose.pose.position.x = slam_pose.pose.pose.position.x + (c_delta*u[0]-s_delta*u[1]);
		slam_odom.pose.pose.position.y = slam_pose.pose.pose.position.y + (s_delta*u[0]+c_delta*u[1]);
		slam_odom.pose.pose.position.z = 0.0;
		slam_odom.twist.twist.linear.x= odom_msg->twist.twist.linear.x;
		slam_odom.twist.twist.linear.y= odom_msg->twist.twist.linear.y;
		slam_odom.twist.twist.linear.z= odom_msg->twist.twist.linear.z;
		slam_odom.twist.twist.angular.z = odom_msg->twist.twist.angular.z;
		double kut = kut_slam + u[2];
		if(kut>PI){
			kut = kut-_2PI;
		}else if(kut < -PI){
			kut = kut+_2PI;
		}
		tf::quaternionTFToMsg(tf::createQuaternionFromYaw(kut), slam_odom.pose.pose.orientation);

		odom_pub.publish(slam_odom);

		tf::Vector3 v(slam_odom.pose.pose.position.x,slam_odom.pose.pose.position.y,slam_odom.pose.pose.position.z);
		tf::Transform l(tf::createQuaternionFromYaw(kut),v);
		tf::StampedTransform c(l,odom_msg->header.stamp, "/map", "/base_footprint");

		static tf::TransformBroadcaster slam_odom_broadcaster;
		slam_odom_broadcaster.sendTransform(c);
	}

};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "slam_odometry");
	slam_odometry *slamodometry = new slam_odometry();
	ros::spin();
	delete slamodometry;

	return 0;
}
