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
	string f_pcl, f_odom, f_odom_t, f_odom_cov, f_pcl_t, f_img,odom_location,true_location,img_location,pcd_location;
	boost::format pcl_format_, odom_format_, ground_format_, pcl_time_, odom_time_, img_format_;
	ros::NodeHandle n_;
	ros::Publisher odom_pub,true_pub,image_pub;
	ros::ServiceClient callback_client;
	FILE *fp;
	bool ucitao,use_img;
	char line[200];
	char line_cov[300];
	int br_pcd;
	double cov[9][3];

public:
	slam_callback(){
		pcl_format_.parse("%s/%s%05i-PC.%s");
		ground_format_.parse("%s/%s%05i-O.%s");
		odom_format_.parse("%s/%s%05i-OZ.%s");
		img_format_.parse("%s/%06i.%s");
		callback_client = n_.serviceClient<cooperation_server::SlamCallback>("slam_callback");
		odom_pub = n_.advertise<nav_msgs::Odometry>("/slam/odom",1);
		true_pub = n_.advertise<nav_msgs::Odometry>("/true/odom",1);
		image_pub = n_.advertise<sensor_msgs::Image>("/slam/image",1);

		if(!ros::param::get("/odom_location", odom_location)){
			odom_location  = "/media/kruno/Data/TEST/SLAM/Odom";
		}
		if(!ros::param::get("/true_location", true_location)){
			true_location  = "/media/kruno/Data/TEST/SLAM/TrueOdom";
		}
		if(!ros::param::get("/img_location", img_location)){
			img_location  = "/media/kruno/Data/TEST/SLAM/Img";
		}
		if(!ros::param::get("/pcd_location", pcd_location)){
			pcd_location  = "/media/kruno/Data/TEST/SLAM/PCD";
		}
		if(!ros::param::get("/use_img", use_img)){
			use_img  = false;
		}
		if(!ros::param::get("/br_img", br_pcd)){
			br_pcd  = 0;
		}

		cout << "Images loaded from: " << img_location << endl;
		cout << "Odom loaded from:   " << odom_location << endl;
		cout << "True loaded from:   " << true_location << endl;
		cout << "Number of images:   " << br_pcd << endl;

		//br_pcd = 4540;//2761
	}

	void send_data(){
		FILE *pFile;
		pcl::PointCloud<pcl::PointXYZ> cloud;
		cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
		double x,y,z,Alpha,Beta,Theta,ox,oy,oz,oAlpha,oBeta,oTheta,oxa,oya,oza;
		ros::Time laser_t;
		nav_msgs::Odometry last_odometry,last_true;
		double u[3];
		double u_true[3];
		for(int i = 0; i < br_pcd; i++){
			cooperation_server::SlamCallback servCallback;

			f_odom = (odom_format_ % true_location % "sl-" % i % "txt").str();
			fp=fopen(f_odom.c_str(),"r");
			ucitao=fgets(line, 200, fp);
			sscanf(line, "%lf %lf %lf %lf %lf %lf\n", &x,&y, &z, &Alpha, &Beta, &Theta);
			fclose(fp);

			f_odom = (odom_format_ % odom_location % "sl-" % i % "txt").str();
			fp=fopen(f_odom.c_str(),"r");
			ucitao=fgets(line, 200, fp);
			sscanf(line, "%lf %lf %lf %lf %lf %lf %lf %lf %lf\n", &ox,&oy,&oz,&oAlpha,&oBeta,&oTheta,&oxa,&oya,&oza);
			fclose(fp);

			/*f_odom_cov = (odom_format_ % "/media/kruno/Data/TEST/Kiti/LaserOdom3d" % "slcov-" % i % "txt").str();
			fp=fopen(f_odom_cov.c_str(),"r");
			for(int j = 0; j < 9; j++){
				ucitao=fgets(line_cov, 200, fp);
				sscanf(line_cov, "%lf %lf %lf\n", &cov[j][0], &cov[j][1], &cov[j][2]);
				for(int k=0; k<3; k++){
					servCallback.request.odom.pose.covariance[j*3+k]=cov[j][k];
				}
			}
			fclose(fp);*/

			f_pcl = (pcl_format_ % pcd_location % "sl-" % i % "pcd").str();
			if (pcl::io::loadPCDFile<pcl::PointXYZ> (f_pcl.c_str(), cloud) == -1){
				PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
			}
			pcl::toROSMsg(cloud,servCallback.request.pcd);

			laser_t = ros::Time::now();

			servCallback.request.pcd.header.frame_id = "velodyne";
			servCallback.request.pcd.header.stamp = laser_t;
			servCallback.request.pcd.header.seq = i;

			if(use_img){
				f_img = (img_format_ % img_location % i % "png").str();
				cv_ptr->header.seq=i;
				cv_ptr->header.stamp=laser_t;
				cv_ptr->header.frame_id="image_rgb";
				const clock_t begin_t3 = clock();
				cv_ptr->image = cv::imread(f_img.c_str());
				cv_ptr->encoding = enc::BGR8;
				servCallback.request.img = *(cv_ptr->toImageMsg());
			}

			servCallback.request.odom.header.frame_id = "velodyne";
			servCallback.request.odom.header.stamp = laser_t;
			servCallback.request.odom.header.seq = i;
			servCallback.request.odom.pose.pose.position.x = ox;
			servCallback.request.odom.pose.pose.position.y = oy;
			servCallback.request.odom.pose.pose.position.z = oz;
			servCallback.request.odom.pose.pose.orientation.x = oAlpha;
			servCallback.request.odom.pose.pose.orientation.y = oBeta;
			servCallback.request.odom.pose.pose.orientation.z = oTheta;

			nav_msgs::Odometry true_odometry;
			true_odometry.header.frame_id = "velodyne";
			true_odometry.header.stamp = laser_t;
			true_odometry.header.seq = i;
			true_odometry.pose.pose.position.x = x;
			true_odometry.pose.pose.position.y = y;
			true_odometry.pose.pose.position.z = z;//broj_poruka;
			tf::quaternionTFToMsg(tf::createQuaternionFromYaw(-Alpha), true_odometry.pose.pose.orientation);

			nav_msgs::Odometry odometry_send;
			odometry_send.header.seq = i;
			odometry_send.header.stamp = laser_t;
			odometry_send.header.frame_id = "velodyne";
			odometry_send.pose.pose.position.x = oxa;
			odometry_send.pose.pose.position.y = oya;
			odometry_send.pose.pose.position.z = oza;
			tf::quaternionTFToMsg(tf::createQuaternionFromRPY(0,0,0), odometry_send.pose.pose.orientation);
			odom_pub.publish(odometry_send);
			true_pub.publish(true_odometry);
			image_pub.publish(cv_ptr->toImageMsg());

			last_odometry = servCallback.request.odom;
			last_true = true_odometry;

			if (callback_client.call(servCallback)){
				printf("%d %d Successfully sent to SLAM: %f %f %f  %f %f %f\n",i,servCallback.response.done,z,-x,-Alpha*180/PI,-oz,ox,-oBeta*180/PI);
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
