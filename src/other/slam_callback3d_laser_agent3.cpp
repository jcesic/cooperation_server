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
	string f_odom, f_odom_t, f_odom_cov, f_img,odom_location,true_location,img_location,seq_cov_location, f_pcl,pcd_location;
	boost::format odom_format_, ground_format_, pcl_time_, odom_time_, img_format_, cov_format_, pcl_format_;
	ros::NodeHandle n_;
	ros::Publisher odom_pub,true_pub,image_pub;
	ros::ServiceClient callback_client;
	FILE *fp;
	bool ucitao,use_img,use_true,use_cov;
	char line[400];
	char line_cov[400];
	int br_img,max_loops,delta;
	float cov[3][3];

public:
	slam_callback(){
		ground_format_.parse("%s/%s%05i-O.%s");
		odom_format_.parse("%s/%s%05i-OZ.%s");
		//img_format_.parse("%s/%s%05i.%s");
		img_format_.parse("%s/%06i.%s");
		cov_format_.parse("%s/%s%05i-OC.%s");
		pcl_format_.parse("%s/%s%05i-PC.%s");

		callback_client = n_.serviceClient<cooperation_server::SlamCallback>("slam_callback_2");
		odom_pub = n_.advertise<nav_msgs::Odometry>("/slam/odom_2",1);
		image_pub = n_.advertise<sensor_msgs::Image>("/slam/image_2",1);

		if(!ros::param::get("/odom_location_3", odom_location)){
			odom_location  = "/media/kruno/Data/TEST/SLAM/Odom";
		}
		if(!ros::param::get("/pcd_location_3", pcd_location)){
			pcd_location  = "/media/kruno/Data/TEST/SLAM/PCD";
		}
		if(!ros::param::get("/img_location_3", img_location)){
			img_location  = "/media/kruno/Data/TEST/SLAM/Img";
		}

		if(!ros::param::get("/use_img", use_img)){
			use_img  = false;
		}
		if(!ros::param::get("/use_true", use_true)){
			use_true  = false;
		}
		if(!ros::param::get("/use_cov", use_cov)){
			use_cov  = false;
		}
		if(!ros::param::get("/br_img", br_img)){
			br_img  = 0;
		}
		if(!ros::param::get("/max_loops", max_loops)){
			max_loops  = 0;
		}
		delta=201;
		cout << "Odom loaded from:   " << odom_location << endl;
		cout << "Number of images:   " << br_img << endl;
	}

	void send_data(){
		FILE *pFile;
		pcl::PointCloud<pcl::PointXYZ> cloud;
		cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
		double x,y,z,Alpha,Beta,Theta,ox,oy,oz,oAlpha,oBeta,oTheta,oxa,oya,oza;
		ros::Time image_t;
		nav_msgs::Odometry last_odometry,last_true;
		double u[3];
		double u_true[3];
		int num_loops = 0;

		for(int i = 3835; (i < 4484 && ros::ok()); i++){
			cooperation_server::SlamCallback servCallback;
			double elapsed_secs;

			if(use_true){
				f_odom = (odom_format_ % true_location % "sl-" % i % "txt").str();
				fp=fopen(f_odom.c_str(),"r");
				ucitao=fgets(line, 400, fp);
				sscanf(line, "%lf %lf %lf %lf %lf %lf\n", &x,&y, &z, &Alpha, &Beta, &Theta);
				fclose(fp);
			}

			f_odom = (odom_format_ % odom_location % "sl-" % i % "txt").str();
			fp=fopen(f_odom.c_str(),"r");
			ucitao=fgets(line, 400, fp);
			sscanf(line, "%lf %lf %lf %lf %lf %lf %lf %lf %lf\n", &ox,&oy,&oz,&oAlpha,&oBeta,&oTheta,&oxa,&oya,&oza);
			fclose(fp);

			//f_odom_cov = (cov_format_ % seq_cov_location % "cov_" % i % "txt").str();
			if(use_cov){
				f_odom_cov = (cov_format_ % seq_cov_location % "sl-" % i % "txt").str();
				cout << "f_odom_cov " << f_odom_cov << endl;
				fp=fopen(f_odom_cov.c_str(),"r");
				for(int j = 0; j < 3; j++){
					ucitao=fgets(line_cov, 400, fp);
					sscanf(line_cov, "%f %f %f %f %f %f %f %f %f\n", &cov[0][0], &cov[0][1], &cov[0][2],&cov[1][0],&cov[1][1],&cov[1][2],&cov[2][0],&cov[2][1],&cov[2][2]);
					for(int bri=0; bri<3; bri++){
						for(int brj=0; brj<3;brj++){
							if(j==0){
								servCallback.request.odom.pose.covariance[bri*3+brj+j*9]=cov[bri][brj]; //translacija
							}else if(j==1){
								servCallback.request.odom.pose.covariance[bri*3+brj+j*9]=cov[bri][brj]; //rotacija
							}else{
								servCallback.request.odom.pose.covariance[bri*3+brj+j*9]=cov[bri][brj]; //medu
							}
						}
					}
				}
				fclose(fp);
			}

			image_t = ros::Time::now();

			f_pcl = (pcl_format_ % pcd_location % "sl-" % i % "pcd").str();
			if (pcl::io::loadPCDFile<pcl::PointXYZ> (f_pcl.c_str(), cloud) == -1){
				PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
			}
			pcl::toROSMsg(cloud,servCallback.request.pcd);
			servCallback.request.pcd.header.frame_id = "velodyne";
			servCallback.request.pcd.header.stamp = image_t;
			servCallback.request.pcd.header.seq = i;


			if(use_img){
				//f_img = (img_format_ % img_location % "sl-" % i % "bmp").str();
				f_img = (img_format_ % img_location % i % "png").str();
				cv_ptr->header.seq=i;
				cv_ptr->header.stamp=image_t;
				cv_ptr->header.frame_id="image_rgb";
				cv_ptr->image = cv::imread(f_img.c_str());
				cv_ptr->encoding = enc::BGR8;
				//servCallback.request.img = *(cv_ptr->toImageMsg());
			}

			servCallback.request.odom.header.frame_id = "velodyne";
			servCallback.request.odom.header.stamp = image_t;
			servCallback.request.odom.header.seq = i;
			servCallback.request.odom.pose.pose.position.x = ox;
			servCallback.request.odom.pose.pose.position.y = oy;
			servCallback.request.odom.pose.pose.position.z = oz;
			servCallback.request.odom.pose.pose.orientation.x = oAlpha;
			servCallback.request.odom.pose.pose.orientation.y = oBeta;
			servCallback.request.odom.pose.pose.orientation.z = oTheta;

			nav_msgs::Odometry true_odometry;
			if(use_true){
				true_odometry.header.frame_id = "velodyne";
				true_odometry.header.stamp = image_t;
				true_odometry.header.seq = i;
				true_odometry.pose.pose.position.x = x;
				true_odometry.pose.pose.position.y = y;
				true_odometry.pose.pose.position.z = z;//broj_poruka;
				tf::quaternionTFToMsg(tf::createQuaternionFromYaw(-Alpha), true_odometry.pose.pose.orientation);
			}

			nav_msgs::Odometry odometry_send;
			odometry_send.header.seq = i;
			odometry_send.header.stamp = image_t;
			odometry_send.header.frame_id = "velodyne";
			odometry_send.pose.pose.position.x = oxa;
			odometry_send.pose.pose.position.y = oya;
			odometry_send.pose.pose.position.z = oza;
			tf::quaternionTFToMsg(tf::createQuaternionFromRPY(0,0,0), odometry_send.pose.pose.orientation);
			odom_pub.publish(odometry_send);
			if(use_true)
				true_pub.publish(true_odometry);
			if(use_img)
				image_pub.publish(cv_ptr->toImageMsg());

			last_odometry = servCallback.request.odom;
			last_true = true_odometry;
			servCallback.request.fab_id=-1;

			if (callback_client.call(servCallback)){
				//printf("%d %d Successfully sent to SLAM: %f %f %f  %f %f %f\n",i,servCallback.response.done,z,-x,-Alpha*180/PI,-oz,ox,-oBeta*180/PI);
			}else{
				ROS_ERROR("Could not contact SLAM");
			}
		}
	}
};

int main(int argc, char* argv[]){

	ros::init(argc, argv, "slam_callback_agent3");
	slam_callback SlamCallback;
	SlamCallback.send_data();
	return 0;
}
