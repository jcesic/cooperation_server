#include <ros/ros.h>
#include <iostream>
#include <nav_msgs/Path.h>
#include <tf/tf.h>

using namespace std;
#define PI 3.14159265

class MyClass {
public:
	MyClass(){

		odom_sub=nh_.subscribe("/husky/Optitrack",500,&MyClass::callback,this);
		opt_trajectory_pub = nh_.advertise<nav_msgs::Path>("/optitrack/trajectory",1);
		x0 = -0.481720;
		y0 = 0.393121;
		theta0 = 3.155;
		A0[0][0] = cos(-theta0);
		A0[0][1] = sin(-theta0);
		A0[1][0] = -sin(-theta0);
		A0[1][1] = cos(-theta0);
		brojac = 0;

		printf("Odometry initialized and ready...\n");
	}

	void callback(const geometry_msgs::PoseStampedConstPtr& o_msg){

		if(!(brojac%10)){
			x=o_msg->pose.position.x;
			y=o_msg->pose.position.y;
			z=o_msg->pose.position.z;
			a=o_msg->pose.orientation.z;

			geometry_msgs::PoseStamped pose;

			opt_path.header.stamp = o_msg->header.stamp;
			opt_path.header.frame_id = "map";

			pose.pose.position.x = (A0[0][0]*(x-x0)+A0[0][1]*(y-y0))*1000;
			pose.pose.position.y = (A0[1][0]*(x-x0)+A0[1][1]*(y-y0))*1000;
			tf::Quaternion q = tf::createQuaternionFromYaw(theta0+a);
			pose.pose.orientation.w = q.w();
			pose.pose.orientation.z = q.z();
			pose.header = opt_path.header;
			opt_path.poses.push_back(pose);

			opt_trajectory_pub.publish(opt_path);

			double  current_time = o_msg->header.stamp.sec+(o_msg->header.stamp.nsec / 1e9);

			/*pFile = fopen("/home/kruno/catkin_ws/TEST/SLAM/odom_simu.txt","a");
	  	  	fprintf(pFile,"%f %f %f %f %f\n",x,y,z,a,current_time);
	  	    fclose(pFile);*/
		}
		brojac++;
	}

private:
	ros::NodeHandle nh_;
	double x,y,z,a;
	double A0[2][2];
	ros::Subscriber odom_sub;
	ros::Publisher opt_trajectory_pub;
	FILE *pFile;
	double x0,y0,theta0;
	nav_msgs::Path opt_path;
	int brojac;
};

int main(int argc, char** argv) {
	ros::init( argc, argv, "Optitrack" );
	MyClass mc;

	while( ros::ok() ){
		ros::spin();
	}

	return EXIT_SUCCESS;
}

