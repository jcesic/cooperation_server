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

class send_images{

	char buffer[100];
	string f_pcl, f_odom, f_odom_t, f_pcl_t, f_img_left, f_img_right;
	boost::format pcl_format_,odom_format_, ground_format_, pcl_time_, odom_time_, img_format_;
	FILE *fp;
	bool ucitao;
	char line[200];
	int br_pcd;
	ros::NodeHandle n_;
	ros::Publisher image_left;
	ros::Publisher image_right;
public:

	send_images(){
		pcl_format_.parse("%s/%s%05i-PC.%s");
		ground_format_.parse("%s/%s%05i-O.%s");
		odom_format_.parse("%s/%s%05i-OZ.%s");
		img_format_.parse("%s/%06i.%s");
		image_left = n_.advertise<sensor_msgs::Image>("/kiti/image_left",1);
		image_right = n_.advertise<sensor_msgs::Image>("/kiti/image_right",1);


		br_pcd = 4540;//2761
	}

	void send(){
        cv_bridge::CvImagePtr cv_ptr_left(new cv_bridge::CvImage);
        cv_bridge::CvImagePtr cv_ptr_right(new cv_bridge::CvImage);

		ros::Time laser_t;
		bool first_image = true;
		std::ifstream infile("/media/kruno/Data/TEST/Kiti/times.txt");
		std::string lines;
		ros::Time t1s = ros::Time::now();

		int odoms,odomns,lasers,laserns;

		for(int i = 0; i < br_pcd; i++){

			f_img_left = (img_format_ % "/media/kruno/Data/Downloads/dataset_images/sequences/00/image_0" % i % "png").str();
			f_img_right = (img_format_ % "/media/kruno/Data/Downloads/dataset_images/sequences/00/image_1" % i % "png").str();

			std::getline(infile, lines);
		    std::istringstream iss(lines);
		    iss >> lasers >> laserns;

			laser_t.sec =lasers+t1s.sec;
			laser_t.nsec = laserns;

			printf("time %d %d\n",lasers,laserns);

			cv_ptr_left->header.seq=i;
			cv_ptr_left->header.stamp=laser_t;
			cv_ptr_left->header.frame_id="kiti_camera";
			cv_ptr_left->image = cv::imread(f_img_left.c_str());
			cv_ptr_left->encoding = enc::BGR8;


			cv_ptr_right->header.seq=i;
			cv_ptr_right->header.stamp=laser_t;
			cv_ptr_right->header.frame_id="kiti_camera";
			cv_ptr_right->image = cv::imread(f_img_right.c_str());
			cv_ptr_right->encoding = enc::BGR8;

			image_left.publish(cv_ptr_left->toImageMsg());
			image_right.publish(cv_ptr_right->toImageMsg());
			if(first_image){
				ros::Duration(2).sleep();
				image_left.publish(cv_ptr_left->toImageMsg());
				image_right.publish(cv_ptr_right->toImageMsg());
				ros::Duration(2).sleep();
			}else
				ros::Duration(0.2).sleep();
			first_image=false;
		}
	}
};

int main(int argc, char* argv[]){
	ros::init(argc, argv, "send_images");
	send_images sendImages;
	sendImages.send();
	return 0;
}
