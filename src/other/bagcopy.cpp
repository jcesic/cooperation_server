#include <stdio.h>
#include <time.h>
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

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
#include <sensor_msgs/Imu.h>

#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h> //hydro
#include <sensor_msgs/Image.h> //hydro

#include <pcl/point_cloud.h>

#include <cooperation_server/DifferentialSpeed.h>


#ifndef _2PI
#define _2PI	6.28318530717959
#endif
#define PI			3.14159265358979

using namespace std;
using namespace sensor_msgs;
namespace enc = sensor_msgs::image_encodings;
using namespace cv;

class bagread{

	char buffer[100];
	string f_pcl, f_odom, f_odom_t, f_pcl_t, f_img, pcd_location,f_image;
	boost::format pcl_format_,odom_format_, ground_format_, pcl_time_, odom_time_, img_format_;
	FILE *fp;
	bool ucitao;
	char line[200];
	int br_pcd;
	double l_time_imu, l_time_speed;
	bool f_imu,f_speed;
public:

	bagread(){
		pcl_format_.parse("%s/%s%05i-PC.%s");
		ground_format_.parse("%s/%s%05i-O.%s");
		odom_format_.parse("%s/%s%05i-OZ.%s");
		img_format_.parse("%s/%s%05i.%s");
		br_pcd=0;
		f_speed = f_imu = true;
		pcd_location = "/media/kruno/Data/TEST/FER/PCDall";

		fp = fopen("/media/kruno/Data/TEST/FER/speed.txt","w");
		fclose(fp);
		fp = fopen("/media/kruno/Data/TEST/FER/imu.txt","w");
		fclose(fp);
		fp = fopen("/media/kruno/Data/TEST/FER/pcd.txt","w");
		fclose(fp);
	}

	bool read_data(){
		rosbag::Bag bag;
		bag.open("/media/kruno/Data/Bag/mag.bag", rosbag::bagmode::Read);

		std::vector<std::string> topics;
		topics.push_back(std::string("/husky/data/differential_speed"));
		topics.push_back(std::string("/imu/data"));
		topics.push_back(std::string("/velodyne_points_f"));
		topics.push_back(std::string("/axis/image_raw_out_f"));

		rosbag::View view(bag, rosbag::TopicQuery(topics));
		foreach(rosbag::MessageInstance const m, view)
		{
			cooperation_server::DifferentialSpeed::ConstPtr speed_msg = m.instantiate<cooperation_server::DifferentialSpeed>();
			sensor_msgs::Imu::ConstPtr msg_imu = m.instantiate<sensor_msgs::Imu>();
			sensor_msgs::PointCloud2::ConstPtr pcd = m.instantiate<sensor_msgs::PointCloud2>();
			sensor_msgs::Image::ConstPtr img = m.instantiate<sensor_msgs::Image>();

/*			if(speed_msg){
				double dt;
				double t_tmp = speed_msg->header.stamp.sec+(speed_msg->header.stamp.nsec / 1e9);
				if(f_speed){
					f_speed=false;
					l_time_speed = t_tmp;
				}
				dt = t_tmp-l_time_speed;
				l_time_speed = t_tmp;
				fp = fopen("/media/kruno/Data/TEST/FER/speed.txt","a");
				fprintf(fp,"%d %f %f %f %f %f %f %f %f %f %f %f\n",0,t_tmp,dt,speed_msg->right_speed,speed_msg->left_speed,0.0,0.0,0.0,0.0,0.0,0.0,0.0);
				fclose(fp);
			}
			if(msg_imu){
				double dt;

				double t_tmp = msg_imu->header.stamp.sec+(msg_imu->header.stamp.nsec / 1e9);
				if(f_imu){
					f_imu=false;
					l_time_imu = t_tmp;
				}
				dt = t_tmp-l_time_imu;
				l_time_imu = t_tmp;
				fp = fopen("/media/kruno/Data/TEST/FER/speed.txt","a");
				tf::Quaternion tfQ(msg_imu->orientation.x,msg_imu->orientation.y,msg_imu->orientation.z,msg_imu->orientation.w);
				tf::Matrix3x3 rot(tfQ);
				double r00=rot[0][0];	double r01=rot[0][1];	double r02=rot[0][2];
				double r10=rot[1][0];	double r11=rot[1][1];	double r12=rot[1][2];
				double r20=rot[2][0];	double r21=rot[2][1];	double r22=rot[2][2];
				fprintf (fp, "%d %f %f %e %e %e %e %e %e %e %e %e\n",1,t_tmp,dt,r00,r01,r02,r10,r11,r12,r20,r21,r22);
				fclose(fp);
			}*/
			if(img){
				/*pcl::PointCloud<pcl::PointXYZ> cloud;
				pcl::fromROSMsg(*pcd, cloud);
				f_pcl = (pcl_format_ % pcd_location % "sl-" % br_pcd % "pcd").str();
				pcl::io::savePCDFileASCII (f_pcl, cloud);*/


				f_image = (img_format_ % "/media/kruno/Data/TEST/FER/Img" % "sl-" % br_pcd % "bmp").str();
				cout << "f_image" << " " << f_image << endl;
				cout << img->width << " " << img->height << endl;
				cv_bridge::CvImagePtr image;
				try{
					image = cv_bridge::toCvCopy(img, enc::BGR8);
				}catch (cv_bridge::Exception& e){
					ROS_ERROR("Unable to convert %s image to bgr8: %s", img->encoding.c_str(), e.what());
				}
				if (image){
					cv::imwrite(f_image.c_str(), image->image);
				}else{
					ROS_WARN("Couldn't save image, no data!");
				}

/*				double t_tmp = pcd->header.stamp.sec+(pcd->header.stamp.nsec / 1e9);
				fp = fopen("/media/kruno/Data/TEST/FER/speed.txt","a");
				fprintf(fp,"%d %f %f %f %f %f %f %f %f %f %f %f\n",2,t_tmp,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0);
				fclose(fp);*/
				br_pcd++;
				printf("br: %d\n",br_pcd);
			}


		}

		return true;
	}
};

int main(int argc, char* argv[]){

	ros::init(argc, argv, "bagcopy");
	bagread bagread_;
	bagread_.read_data();
	return 0;
}
