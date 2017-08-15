#include <stdio.h>
#include <time.h>
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <cooperation_server/MatchImage.h>
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

class test_fabmap{

	ros::NodeHandle n_;
	ros::ServiceClient match_client;
	string f_img;
	boost::format img_format_;
	int br_pcd;
public:

	test_fabmap(){
		match_client = n_.serviceClient<cooperation_server::MatchImage>("match_images");
		img_format_.parse("%s/%06i.%s");
		br_pcd = 2761;
	}

	void test(){
		cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
		for(int i = 0; i < br_pcd; i++){
			f_img = (img_format_ % "/media/kruno/Data/Downloads/dataset_images/sequences/05/image_0" % i % "png").str();
			cv_ptr->header.seq=i;
			cv_ptr->header.stamp=ros::Time::now();
			cv_ptr->header.frame_id="image_rgb";
			cv_ptr->image = cv::imread(f_img.c_str());
			cv_ptr->encoding = enc::BGR8;

			cooperation_server::MatchImage client_img;
			client_img.request.img = *(cv_ptr->toImageMsg());
			client_img.request.img_id = i;
			if (match_client.call(client_img)){
				for(int m = 0; m < client_img.response.toImgSeq.size();m++){
					if(abs(client_img.response.toImgSeq[m]-client_img.response.fromImgSeq)>10){
						printf("%d %d (%f, %f)\n",client_img.response.fromImgSeq,client_img.response.toImgSeq[m],client_img.response.toImgLike[m], client_img.response.toImgMatch[m]);
					}
				}
			}else{
				ROS_ERROR("Failed to call fabMap\n");
			}
		}

		/*cooperation_server::MatchImage client_img;
		client_img.request.img_id = -1;
		if (match_client.call(client_img)){
			printf("Codebook OK\n");
		}*/
	}
};

int main(int argc, char* argv[]){

	ros::init(argc, argv, "bagcopy");
	test_fabmap t_fabmap;
	t_fabmap.test();
	return 0;
}
