#include <ros/ros.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <math.h> 
#include <sensor_msgs/Image.h>
#include <tf/tf.h>

#include <iostream>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Odometry.h>


#include <cooperation_server/augment.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sstream>
#include <string>
#include <fstream>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;
using namespace sensor_msgs;
namespace enc = sensor_msgs::image_encodings;

#define PI 3.14159265
#define DO_LOG

class MyClass {
public:
	MyClass(){

		odom_sub=nh_.subscribe("/slam/odom",500,&MyClass::odom_callback,this);
		laser_sub=nh_.subscribe("/laser_odom",500,&MyClass::laser_odom_callback,this);

		slam_sub_1=nh_.subscribe("/slam/trajectory_0",500,&MyClass::slam_callback1,this);
		slam_sub_2=nh_.subscribe("/slam/trajectory_1",500,&MyClass::slam_callback2,this);
		slam_sub_3=nh_.subscribe("/slam/trajectory_2",500,&MyClass::slam_callback3,this);

		true_sub=nh_.subscribe("/true/odom",500,&MyClass::true_callback,this);

		if(!ros::param::get("/temp_path", temp_path)){
			temp_path  = "/media/kruno/Data/TEST/SLAM/";//boost::filesystem::current_path().string();
		}

		odom_trajectory_pub = nh_.advertise<visualization_msgs::Marker>("/odom/trajectory_m",1);
		slam_trajectory_pub_1 = nh_.advertise<visualization_msgs::Marker>("/slam/trajectory_m1",1);
		slam_trajectory_pub_2 = nh_.advertise<visualization_msgs::Marker>("/slam/trajectory_m2",1);
		slam_trajectory_pub_3 = nh_.advertise<visualization_msgs::Marker>("/slam/trajectory_m3",1);

		slam_trajectory_pub_prev = nh_.advertise<visualization_msgs::Marker>("/slam/trajectory_m_prev",1);
		true_trajectory_pub = nh_.advertise<visualization_msgs::Marker>("/optitrack/trajectory_m",1);
		laser_trajectory_pub = nh_.advertise<visualization_msgs::Marker>("/odom/laser_m",1);


		marker_laser.header.frame_id = marker_slam.header.frame_id = marker_true.header.frame_id = marker_odom.header.frame_id = marker_slam_prev.header.frame_id ="map";
		marker_laser.ns=marker_slam.ns = marker_true.ns = marker_odom.ns = "my";

		marker_laser.type = marker_slam.type = marker_true.type = marker_odom.type  = marker_slam_prev.type =visualization_msgs::Marker::LINE_STRIP;
		marker_laser.action = marker_slam.action = marker_true.action = marker_odom.action  = marker_slam_prev.action = visualization_msgs::Marker::ADD;

		marker_laser.pose.orientation.x = marker_slam.pose.orientation.x = marker_true.pose.orientation.x = marker_odom.pose.orientation.x = marker_slam_prev.pose.orientation.x =0.0;
		marker_laser.pose.orientation.y = marker_slam.pose.orientation.y = marker_true.pose.orientation.y = marker_odom.pose.orientation.y = marker_slam_prev.pose.orientation.y = 0.0;
		marker_laser.pose.orientation.z = marker_slam.pose.orientation.z = marker_true.pose.orientation.z = marker_odom.pose.orientation.z = marker_slam_prev.pose.orientation.z = 0.0;

		marker_laser.color.a = marker_slam.color.a = marker_true.color.a = marker_odom.color.a = marker_slam_prev.color.a = 1.0;

		marker_slam.scale.x = marker_slam.scale.y = marker_slam.scale.z = 1.1;
		marker_slam_prev.scale.x = 0.8;
		marker_odom.scale.x = 0.8;
		marker_true.scale.x = 0.8;
		marker_laser.scale.x = marker_laser.scale.y = marker_laser.scale.z = 1.1;

		marker_laser.color.r = 1.0;
		marker_laser.color.g = 1.0;
		marker_laser.color.b = 0.0;

		marker_slam.color.r = 0.0;
		marker_slam.color.g = 1.0;
		marker_slam.color.b = 0.0;

		marker_odom.color.r = 1.0;
		marker_odom.color.g = 0.0;
		marker_odom.color.b = 0.0;

		marker_true.color.r = 0.0;
		marker_true.color.g = 0.0;
		marker_true.color.b = 1.0;

		marker_slam_prev.color.r = 0.0;
		marker_slam_prev.color.g = 1.0;
		marker_slam_prev.color.b = 1.0;

		marker_true.id = 0;
		marker_odom.id = 1;
		marker_slam.id = 2;
		marker_slam_prev.id = 3;
		marker_laser.id = 4;

		marker_laser.points.clear();
		marker_odom.points.clear();
		marker_slam.points.clear();
		marker_true.points.clear();
		marker_slam_prev.points.clear();

		p0.x = 0;
		p0.y = 0;
		p0.z = 0;

		marker_slam.points.push_back(p0);
		marker_true.points.push_back(p0);
		marker_odom.points.push_back(p0);
		marker_slam_prev.points.push_back(p0);
		marker_laser.points.push_back(p0);
		printf("Odometry initialized and ready...\n");
	}

	void laser_odom_callback(const nav_msgs::OdometryConstPtr& msg){
		marker_laser.header = msg->header;
		marker_laser.header.frame_id = "map";
		geometry_msgs::Point p;
		p.x = msg->pose.pose.position.x;
		p.y = msg->pose.pose.position.y;
		p.z = msg->pose.pose.position.z;

		marker_laser.points.push_back(p);

		laser_trajectory_pub.publish(marker_laser);
	}

	void odom_callback(const nav_msgs::OdometryConstPtr& msg){
		marker_odom.header = msg->header;
		marker_odom.header.frame_id = "map";
		geometry_msgs::Point p;
		p.x = msg->pose.pose.position.x;
		p.y = msg->pose.pose.position.y;
		p.z = msg->pose.pose.position.z;
#ifdef DO_LOG
		snprintf(buffer, sizeof(buffer), "odometry.txt");
		pFile = fopen((temp_path + buffer).c_str(),"a");
		fprintf(pFile,"%f %f %f\n",p.x,p.y,p.z);
		fclose(pFile);
#endif
		marker_odom.points.push_back(p);

		odom_trajectory_pub.publish(marker_odom);
	}

	void slam_callback1(const nav_msgs::PathConstPtr& msg){
		marker_slam.header = msg->header;
		marker_slam.header.frame_id = "map";
		marker_slam.points.clear();
		marker_slam.color.r = 0.0;
		marker_slam.color.g = 1.0;
		marker_slam.color.b = 0.0;

		geometry_msgs::Point p;
		geometry_msgs::PoseStamped pose;

#ifdef DO_LOG
		snprintf(buffer, sizeof(buffer), "trajectory_1/%ld_traj.txt",msg->poses.size());
		pFile = fopen((temp_path + buffer).c_str(),"w");
#endif
		for(int i=0;i<msg->poses.size();i++){
			pose = msg->poses[i];
			p.x = pose.pose.position.x;
			p.y = pose.pose.position.y;
			p.z = pose.pose.position.z;
			marker_slam.points.push_back(p);
#ifdef DO_LOG
			fprintf(pFile,"%f %f %f\n",p.x,p.y,p.z);
#endif
		}
#ifdef DO_LOG
		fclose(pFile);
#endif
		slam_trajectory_pub_1.publish(marker_slam);
	}

	void slam_callback2(const nav_msgs::PathConstPtr& msg){
		marker_slam.header = msg->header;
		marker_slam.header.frame_id = "map";
		marker_slam.points.clear();
		marker_slam.color.r = 1.0;
		marker_slam.color.g = 0.0;
		marker_slam.color.b = 1.0;

		geometry_msgs::Point p;
		geometry_msgs::PoseStamped pose;

#ifdef DO_LOG
		snprintf(buffer, sizeof(buffer), "trajectory_2/%ld_traj.txt",msg->poses.size());
		pFile2 = fopen((temp_path + buffer).c_str(),"w");
#endif
		for(int i=0;i<msg->poses.size();i++){
			pose = msg->poses[i];
			p.x = pose.pose.position.z;
			p.y = -pose.pose.position.x;
			p.z = -pose.pose.position.y;
			marker_slam.points.push_back(p);
#ifdef DO_LOG
			fprintf(pFile2,"%f %f %f\n",p.x,p.y,p.z);
#endif
		}
#ifdef DO_LOG
		fclose(pFile2);
#endif
		slam_trajectory_pub_2.publish(marker_slam);
	}

	void slam_callback3(const nav_msgs::PathConstPtr& msg){
		marker_slam.header = msg->header;
		marker_slam.header.frame_id = "map";
		marker_slam.points.clear();
		marker_slam.color.r = 1.0;
		marker_slam.color.g = 1.0;
		marker_slam.color.b = 0.0;

		geometry_msgs::Point p;
		geometry_msgs::PoseStamped pose;

#ifdef DO_LOG
		snprintf(buffer, sizeof(buffer), "trajectory_3/%ld_traj.txt",msg->poses.size());
		pFile = fopen((temp_path + buffer).c_str(),"w");
#endif
		for(int i=0;i<msg->poses.size();i++){
			pose = msg->poses[i];
			p.x = pose.pose.position.z;
			p.y = -pose.pose.position.x;
			p.z = -pose.pose.position.y;
			marker_slam.points.push_back(p);
#ifdef DO_LOG
			fprintf(pFile,"%f %f %f\n",p.x,p.y,p.z);
#endif
		}
#ifdef DO_LOG
		fclose(pFile);
#endif
		slam_trajectory_pub_3.publish(marker_slam);
	}


	void true_callback(const nav_msgs::OdometryConstPtr& msg){
		marker_true.header = msg->header;
		marker_true.header.frame_id = "map";
		geometry_msgs::Point p;
		p.x = msg->pose.pose.position.z;
		p.y = -msg->pose.pose.position.x;
		p.z = -msg->pose.pose.position.y;

		marker_true.points.push_back(p);
		true_trajectory_pub.publish(marker_true);
	}

private:
	ros::NodeHandle nh_;
	FILE* pFile;
	FILE* pFile2;
	std::string temp_path;
	char buffer[100];
	double x,y,z,la,a;
	ros::Subscriber odom_sub,slam_sub_1,slam_sub_2,slam_sub_3,true_sub,laser_sub;
	ros::Publisher true_trajectory_pub,slam_trajectory_pub_1,slam_trajectory_pub_2,odom_trajectory_pub,slam_trajectory_pub_prev,slam_trajectory_pub_3,laser_trajectory_pub;
	visualization_msgs::Marker marker_laser, marker_slam, marker_true, marker_odom, marker_slam_prev;
	geometry_msgs::Point p0;
};

int main(int argc, char** argv) {
	ros::init( argc, argv, "t_m" );
	MyClass mc;

	while( ros::ok() ){
		ros::spin();
	}

	return EXIT_SUCCESS;
}

