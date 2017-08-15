// RVLPSuLMdemo.cpp : Defines the entry point for the console application.
//

//#include "highgui.h"
#include <stdio.h>
#include <time.h>
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <rvl_psulmdemo/RelativePoseWithCovariance.h>
#include <rvl_psulmdemo/AddModel.h>
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
#include <rvl_psulmdemo/MatchImage.h>
#include <string.h>
#include <math.h>

#ifndef _2PI
#define _2PI	6.28318530717959
#endif
#define PI			3.14159265358979



using namespace std;
using namespace sensor_msgs;
namespace enc = sensor_msgs::image_encodings;
using namespace cv;


class RVLdemoROS{

	ros::NodeHandle n_;
	ros::Publisher pose_pub;
	ros::Subscriber match_sub;
	ros::ServiceClient client;
	char buffer[100];
	string f_pcl, f_odom, f_odom_t, f_pcl_t,pcl2, f_img;
	boost::format pcl_format_,odom_format_, ground_format_, pcl_time_, odom_time_,img_format_;
	FILE *fp;
	bool ucitao;
	char line[200];
	int br_pcd;
	int br_odom,br_ground;
	bool prva;
public:

	RVLdemoROS(ros::NodeHandle n):n_(n) {
		pcl_format_.parse("%s/%s%05i-PC.%s");
		ground_format_.parse("%s/%s%05i-O.%s");
		odom_format_.parse("%s/%s%05i-OZ.%s");
		img_format_.parse("%s/%s%05i-I.%s");

		br_pcd = 246;
		//br_ground = 1746;
		//br_odom = 38879;

	}

	bool kopiraj(){
		pcl::PointCloud<pcl::PointXYZ> cloud,cloud2;
		rosbag::Bag bag;
		bag.open("/home/kruno/catkin_ws/bags/mag.bag", rosbag::bagmode::Read);
		int odoms,odomns,lasers,laserns;
        cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

		double x,y,z,a,b,c,c0;
		ros::Time laser_t, odom_t;
		int id;

		for(int i = 0; i < br_pcd; i++){
			f_odom = (odom_format_ % "/media/kruno/Data/TEST/BRL/ODOM_BAG1" % "sl-" % i % "txt").str();
			printf("%d\n",id);

			fp=fopen(f_odom.c_str(),"r");
			ucitao=fgets(line, 200, fp);
			sscanf(line, "%lf %lf %lf %lf %lf %lf %d\n", &x,&y, &z, &a, &b, &c, &id);
			fclose(fp);

			f_pcl = (pcl_format_ % "/media/kruno/Data/TEST/BRL/PCD1" % "sl-" % id % "pcd").str();
			f_pcl_t = (pcl_format_ % "/media/kruno/Data/TEST/BRL/PCD1" % "sl-" % id % "time").str();
			//f_img = (img_format_ % "/media/kruno/Data/TEST/Ford/OdomLaser" % "sl-" % (id+1) % "ppm").str();

			if (pcl::io::loadPCDFile<pcl::PointXYZ> (f_pcl.c_str(), cloud) == -1){
				PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
				return false;
			}
			sensor_msgs::PointCloud2 output;
			pcl::toROSMsg(cloud,output);

			fp=fopen(f_pcl_t.c_str(),"r");
			ucitao=fgets(line, 200, fp);
			sscanf(line, "%d %d\n", &lasers, &laserns);
			printf("time:  %d %d\n",lasers,laserns);
			fclose(fp);

			laser_t.sec =lasers;
			laser_t.nsec = laserns;

			output.header.frame_id = "velodyne";
			output.header.stamp = laser_t;
			output.header.seq = i;

/*			cv_ptr->header.seq=i;
			cv_ptr->header.stamp=laser_t;
			cv_ptr->header.frame_id="image_rgb";
			cv_ptr->image = cv::imread(f_img.c_str());
			cv_ptr->encoding = enc::BGR8;*/


			nav_msgs::Odometry odometry;
			odometry.pose.pose.position.x = x;
			odometry.pose.pose.position.y = y;
			odometry.pose.pose.position.z = 0;//broj_poruka;
			tf::quaternionTFToMsg(tf::createQuaternionFromYaw(c), odometry.pose.pose.orientation);

			odometry.header.frame_id = "velodyne";
			odometry.header.stamp = laser_t;
			odometry.header.seq = i;

			bag.write("/slam/odom", laser_t, odometry);
			bag.write("/velodyne_points", laser_t, output);
			//bag.write("/axis/image_raw_out", laser_t, *(cv_ptr->toImageMsg()));

			printf("Zapisao pcd: %d\n",i);
		}

		printf("Kopirao pcd\n");

/*		for(int i = 0; i<br_ground; i++){
			f_odom = (ground_format_ % "/home/kruno/catkin_ws/TEST/Ford_new" % "sl-" % i % "txt").str();
			f_odom_t = (ground_format_ % "/home/kruno/catkin_ws/TEST/Ford_new" % "sl-" % i % "time").str();

			fp=fopen(f_odom.c_str(),"r");
			ucitao=fgets(line, 200, fp);
			sscanf(line, "%lf %lf %lf %lf %lf %lf\n", &x,&y, &z, &a, &b, &c);
			fclose(fp);
			if(i==0)
				c0 = c;

			c = c-c0;
			if (c > PI) {
				c = c - _2PI;
			}else if(c < -PI){
				c = c + _2PI;
			}

			nav_msgs::Odometry odometry;
			odometry.pose.pose.position.x = x;
			odometry.pose.pose.position.y = y;
			odometry.pose.pose.position.z = 0;//broj_poruka;
			tf::quaternionTFToMsg(tf::createQuaternionFromYaw(c), odometry.pose.pose.orientation);

			fp=fopen(f_odom_t.c_str(),"r");
			ucitao=fgets(line, 200, fp);
			sscanf(line, "%d %d\n", &odoms, &odomns);
			fclose(fp);

			odom_t.sec = odoms;
			odom_t.nsec =odomns;

			odometry.header.frame_id = "velodyne";
			odometry.header.stamp = odom_t;
			odometry.header.seq = i;

			double kut = tf::getYaw(odometry.pose.pose.orientation);
			printf("Odometry: %lf %lf %lf %lf\n",x,y,c,kut);

			bag.write("/ground_truth", odom_t, odometry);
			printf("Zapisao ground: %d\n",i);
		}

		for(int i = 0; i<br_odom; i++){
			f_odom = (odom_format_ % "/home/kruno/catkin_ws/TEST/Ford_odomZ" % "sl-" % i % "txt").str();
			f_odom_t = (odom_format_ % "/home/kruno/catkin_ws/TEST/Ford_odomZ" % "sl-" % i % "time").str();

			fp=fopen(f_odom.c_str(),"r");
			ucitao=fgets(line, 200, fp);
			sscanf(line, "%lf %lf %lf %lf %lf %lf\n", &x,&y, &z, &a, &b, &c);
			fclose(fp);

			nav_msgs::Odometry odometry;
			odometry.pose.pose.position.x = x;
			odometry.pose.pose.position.y = y;
			odometry.pose.pose.position.z = 0;//broj_poruka;
			tf::quaternionTFToMsg(tf::createQuaternionFromYaw(c), odometry.pose.pose.orientation);

			fp=fopen(f_odom_t.c_str(),"r");
			ucitao=fgets(line, 200, fp);
			sscanf(line, "%d %d\n", &odoms, &odomns);
			fclose(fp);

			odom_t.sec = odoms;
			odom_t.nsec =odomns;

			odometry.header.frame_id = "velodyne";
			odometry.header.stamp = odom_t;
			odometry.header.seq = i;

			double kut = tf::getYaw(odometry.pose.pose.orientation);
			printf("Odometry: %lf %lf %lf %lf\n",x,y,c,kut);

			bag.write("/slam/odom", odom_t, odometry);
			printf("Zapisao odom: %d\n",i);
		}*/


		bag.close();
		printf("kopirao\n");
		return false;
	}

};

int main(int argc, char* argv[]){

	ros::init(argc, argv, "bagcopy");
	ros::NodeHandle n;
	RVLdemoROS rosdemo = RVLdemoROS(n);
	rosdemo.kopiraj();
	return 0;
}
