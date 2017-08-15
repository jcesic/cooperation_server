#include <stdio.h>
#include <time.h>
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
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


class bagLaser{
	char buffer[100];
	string f_pcl, f_odom, f_pcl_t, f_img,f_odom_t;
	boost::format pcl_format_,odom_format_, ground_format_, img_format_;
	FILE *fp;
	bool ucitao;
	char line[200];
	int br_pcd;
public:

	bagLaser(){
		pcl_format_.parse("%s/%s%05i-PC.%s");
		ground_format_.parse("%s/%s%05i-O.%s");
		odom_format_.parse("%s/%s%05i-OZ.%s");
		img_format_.parse("%s/%06i.%s");
		br_pcd = 2617;
	}

	bool kopiraj(){
		pcl::PointCloud<pcl::PointXYZ> cloud,cloud2;
		rosbag::Bag bag;
		bag.open("/home/kruno/catkin_ws/bags/kiti_laser.bag", rosbag::bagmode::Write);
		int odoms,odomns,lasers,laserns;
        cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

		double x,y,z,Alpha,Beta,Theta;
		int idlaser;
		ros::Time laser_t;
		int tmp;

		for(int i = 0; i < br_pcd; i++){
			f_odom = (odom_format_ % "/media/kruno/Data/TEST/Kiti/LaserOdom" % "sl-" % i % "txt").str();
			fp=fopen(f_odom.c_str(),"r");
			ucitao=fgets(line, 200, fp);
			sscanf(line, "%lf %lf %lf %lf %lf %lf %d\n", &x,&y, &z, &Theta, &Beta, &Alpha, &idlaser);
			fclose(fp);

			f_odom_t = (odom_format_ % "/media/kruno/Data/TEST/Kiti/LaserOdom" % "sl-" % i % "time").str();

			fp=fopen(f_odom_t.c_str(),"r");
			ucitao=fgets(line, 200, fp);
			sscanf(line, "%d %d %d\n", &lasers, &laserns, &tmp);
			fclose(fp);


			f_pcl = (pcl_format_ % "/media/kruno/Data/TEST/Kiti/PCD" % "sl-" % idlaser % "pcd").str();
			f_img = (img_format_ % "/media/kruno/Data/Downloads/dataset_images/sequences/00/image_0" % idlaser % "png").str();

			if (pcl::io::loadPCDFile<pcl::PointXYZ> (f_pcl.c_str(), cloud) == -1){
				PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
				return false;
			}
			sensor_msgs::PointCloud2 output;
			pcl::toROSMsg(cloud,output);


			laser_t.sec =lasers;
			laser_t.nsec = laserns;

			printf("time %d %d\n",lasers,laserns);
			output.header.frame_id = "velodyne";
			output.header.stamp = laser_t;
			output.header.seq = i;

			cv_ptr->header.seq=i;
			cv_ptr->header.stamp=laser_t;
			cv_ptr->header.frame_id="image_rgb";
			cv_ptr->image = cv::imread(f_img.c_str());
			cv_ptr->encoding = enc::BGR8;

			nav_msgs::Odometry odometry;
			odometry.pose.pose.position.x = x;
			odometry.pose.pose.position.y = y;
			odometry.pose.pose.position.z = z;//broj_poruka;

			tf::quaternionTFToMsg(tf::createQuaternionFromYaw(Alpha), odometry.pose.pose.orientation);

			odometry.header.frame_id = "velodyne";
			odometry.header.stamp = laser_t;
			odometry.header.seq = i;

			bag.write("/slam/odom", laser_t, odometry);
			bag.write("/velodyne_points", laser_t, output);
			bag.write("/axis/image_raw_out", laser_t, *(cv_ptr->toImageMsg()));

			printf("Zapisao pcd: %d %d\n",i,idlaser);
		}

		printf("Kopirao pcd\n");
	}
};

int main(int argc, char* argv[]){

	ros::init(argc, argv, "bagLaser");
	bagLaser bag_laser;
	bag_laser.kopiraj();
	return 0;
}
