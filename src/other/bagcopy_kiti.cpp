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

class bagcopy_kiti{

	char buffer[100];
	string f_pcl, f_odom, f_odom_t, f_pcl_t, f_img;
	boost::format pcl_format_,odom_format_, ground_format_, pcl_time_, odom_time_, img_format_;
	FILE *fp;
	bool ucitao;
	char line[200];
	int br_pcd;
public:

	bagcopy_kiti(){
		pcl_format_.parse("%s/%s%05i-PC.%s");
		ground_format_.parse("%s/%s%05i-O.%s");
		odom_format_.parse("%s/%s%05i-OZ.%s");
		img_format_.parse("%s/%06i.%s");

		br_pcd = 4540;
	}

	bool kopiraj(){
		pcl::PointCloud<pcl::PointXYZ> cloud,cloud2;
		rosbag::Bag bag;
		bag.open("/home/kruno/catkin_ws/bags/kiti.bag", rosbag::bagmode::Write);
		int odoms,odomns,lasers,laserns;
        cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

		double x,y,z,Alpha,Beta,Theta;
		ros::Time laser_t, odom_t;
		std::ifstream infile("/media/kruno/Data/TEST/Kiti/times.txt");
		std::string lines;
		ros::Time t1s = ros::Time::now();

		for(int i = 0; i < br_pcd; i++){
			f_odom = (odom_format_ % "/media/kruno/Data/TEST/Kiti/Odom" % "sl-" % i % "txt").str();
			fp=fopen(f_odom.c_str(),"r");
			ucitao=fgets(line, 200, fp);
			sscanf(line, "%lf %lf %lf %lf %lf %lf\n", &x,&y, &z, &Alpha, &Beta, &Theta);
			fclose(fp);

			f_pcl = (pcl_format_ % "/media/kruno/Data/TEST/Kiti/PCD" % "sl-" % i % "pcd").str();
			f_img = (img_format_ % "/media/kruno/Data/Downloads/dataset_images/sequences/00/image_0" % i % "png").str();

			if (pcl::io::loadPCDFile<pcl::PointXYZ> (f_pcl.c_str(), cloud) == -1){
				PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
				return false;
			}

			sensor_msgs::PointCloud2 output;
			pcl::toROSMsg(cloud,output);

			std::getline(infile, lines);
		    std::istringstream iss(lines);
		    iss >> lasers >> laserns;

			laser_t.sec =lasers+t1s.sec;
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
			odometry.pose.pose.position.x = z;
			odometry.pose.pose.position.y = -x;
			odometry.pose.pose.position.z = -y;//broj_poruka;

			tf::quaternionTFToMsg(tf::createQuaternionFromYaw(-Alpha), odometry.pose.pose.orientation);

			odometry.header.frame_id = "velodyne";
			odometry.header.stamp = laser_t;
			odometry.header.seq = i;

			bag.write("/slam/odom", laser_t, odometry);
			bag.write("/velodyne_points", laser_t, output);
			bag.write("/axis/image_raw_out", laser_t, *(cv_ptr->toImageMsg()));

			printf("Zapisao: %d\n",i);
		}

		printf("Bag copy complete\n");
	}
};

int main(int argc, char* argv[]){

	ros::init(argc, argv, "bagcopy");
	bagcopy_kiti bagcopyKiti;
	bagcopyKiti.kopiraj();
	return 0;
}
