#include <stdio.h>
#include <time.h>
#include <iostream>
#include <ros/ros.h>
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
#include <cooperation_server/SlamCallback.h>
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

class slam_callback{
	string f_pcl, f_odom, f_odom_t, f_pcl_t, f_img;
	boost::format pcl_format_, odom_format_, ground_format_, pcl_time_, odom_time_, img_format_;
	ros::NodeHandle n_;
	ros::Publisher odom_pub,true_pub;
	ros::ServiceClient callback_client;
	FILE *fp;
	bool ucitao;
	char line[200];
	int br_pcd;

public:
	slam_callback(){
		pcl_format_.parse("%s/%s%05i-PC.%s");
		ground_format_.parse("%s/%s%05i-O.%s");
		odom_format_.parse("%s/%s%05i-OZ.%s");
		img_format_.parse("%s/%06i.%s");
		callback_client = n_.serviceClient<cooperation_server::SlamCallback>("slam_callback");
		odom_pub = n_.advertise<nav_msgs::Odometry>("/slam/odom",1);
		true_pub = n_.advertise<nav_msgs::Odometry>("/true/odom",1);
		br_pcd = 4540;//2761
	}

	void send_data(){
		pcl::PointCloud<pcl::PointXYZ> cloud;
        cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
		double x,y,z,Alpha,Beta,Theta,ox,oy,oz,oAlpha,oBeta,oTheta;
		ros::Time laser_t;
		nav_msgs::Odometry last_odometry,last_true;
		double u[3];
		double u_true[3];
		for(int i = 0; i < br_pcd; i++){
			cooperation_server::SlamCallback servCallback;

			f_odom = (odom_format_ % "/media/kruno/Data/TEST/Kiti/Odom" % "sl-" % i % "txt").str();
			fp=fopen(f_odom.c_str(),"r");
			ucitao=fgets(line, 200, fp);
			sscanf(line, "%lf %lf %lf %lf %lf %lf\n", &x,&y, &z, &Alpha, &Beta, &Theta);
			fclose(fp);

			f_odom = (odom_format_ % "/media/kruno/Data/TEST/Kiti/StereoOdom" % "sl-" % i % "txt").str();
			fp=fopen(f_odom.c_str(),"r");
			ucitao=fgets(line, 200, fp);
			sscanf(line, "%lf %lf %lf\n", &ox,&oz,&oBeta);
			fclose(fp);

			f_pcl = (pcl_format_ % "/media/kruno/Data/TEST/Kiti/PCD" % "sl-" % i % "pcd").str();
			f_img = (img_format_ % "/media/kruno/Data/Downloads/dataset_images/sequences/00/image_0" % i % "png").str();

			const clock_t begin_t = clock();
			if (pcl::io::loadPCDFile<pcl::PointXYZ> (f_pcl.c_str(), cloud) == -1){
				PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
			}
			double elapsed_secs = double(clock() - begin_t) / CLOCKS_PER_SEC;
			printf("reading pcd: %f ms\n",elapsed_secs*1000);

			const clock_t begin_t2 = clock();
			pcl::toROSMsg(cloud,servCallback.request.pcd);
			elapsed_secs = double(clock() - begin_t2) / CLOCKS_PER_SEC;
			printf("putting to: %f ms\n",elapsed_secs*1000);

			laser_t = ros::Time::now();

			servCallback.request.pcd.header.frame_id = "velodyne";
			servCallback.request.pcd.header.stamp = laser_t;
			servCallback.request.pcd.header.seq = i;

			cv_ptr->header.seq=i;
			cv_ptr->header.stamp=laser_t;
			cv_ptr->header.frame_id="image_rgb";
			const clock_t begin_t3 = clock();
			cv_ptr->image = cv::imread(f_img.c_str());
			elapsed_secs = double(clock() - begin_t3) / CLOCKS_PER_SEC;
			printf("reading img: %f ms\n",elapsed_secs*1000);
			cv_ptr->encoding = enc::BGR8;
			servCallback.request.img = *(cv_ptr->toImageMsg());

			servCallback.request.odom.header.frame_id = "velodyne";
			servCallback.request.odom.header.stamp = laser_t;
			servCallback.request.odom.header.seq = i;
			servCallback.request.odom.pose.pose.position.x = -oz;
			servCallback.request.odom.pose.pose.position.y = ox;
			servCallback.request.odom.pose.pose.position.z = 0.0;
			tf::quaternionTFToMsg(tf::createQuaternionFromYaw(-oBeta), servCallback.request.odom.pose.pose.orientation);


			nav_msgs::Odometry true_odometry;
			true_odometry.header.frame_id = "velodyne";
			true_odometry.header.stamp = laser_t;
			true_odometry.header.seq = i;
			true_odometry.pose.pose.position.x = z;
			true_odometry.pose.pose.position.y = -x;
			true_odometry.pose.pose.position.z = -y;//broj_poruka;
			tf::quaternionTFToMsg(tf::createQuaternionFromYaw(-Alpha), true_odometry.pose.pose.orientation);

			odom_pub.publish(servCallback.request.odom);
			true_pub.publish(true_odometry);

			last_odometry = servCallback.request.odom;
			last_true = true_odometry;

			if (callback_client.call(servCallback)){
				printf("%d Successfully sent to SLAM: %f %f %f  %f %f %f\n",i,z,-x,-Alpha*180/PI,-oz,ox,-oBeta*180/PI);
			}else{
				ROS_ERROR("Could not contact SLAM");
			}
		}
	}
};

int main(int argc, char* argv[]){

	ros::init(argc, argv, "slam_callback");
	slam_callback SlamCallback;
	SlamCallback.send_data();
	return 0;
}
