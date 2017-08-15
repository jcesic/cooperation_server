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
#include <rosbag/bag.h>
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
#define DEG2RAD PI/180
#define RAD2DEG 180/PI


using namespace std;
using namespace sensor_msgs;
namespace enc = sensor_msgs::image_encodings;
using namespace cv;


class fileToPCD{
	char buffer[100];
	string f_pcl, f_pcl_w, f_odom, f_odom_t, f_pcl_t,pcl2, f_img;
	boost::format pcl_format_,pcl_format_w_,odom_format_, ground_format_, pcl_time_, odom_time_,img_format_;
	FILE *fp;
	FILE *fp_write;
	int br_pcd,brojac,height,width,npoints,brmod;

public:

	fileToPCD(){
		pcl_format_.parse("%s/%06i.%s");
		pcl_format_w_.parse("%s/%s%05i-PC.%s");
		odom_format_.parse("%s/%s%05i-OZ.%s");

		brojac=0;
		height = 1;
		br_pcd = 1591;
		brmod =1;

	}

	bool test(){
		for(int i = 0; i < br_pcd; i=i+8){
			f_pcl = (pcl_format_ % "/media/kruno/Data/Downloads/dataset_laser/sequences/06/velodyne" % i % "bin").str();
			int32_t num = 1000000;
			float *data = (float*)malloc(num*sizeof(float));

			// pointers
			float *px = data+0;
			float *py = data+1;
			float *pz = data+2;
			float *pr = data+3;

			// load point cloud
			FILE *stream;
			stream = fopen (f_pcl.c_str(),"rb");
			num = fread(data,sizeof(float),num,stream)/4;
			for (int32_t i=0; i<num; i++) {
				px+=4; py+=4; pz+=4; pr+=4;
			}
			fclose(stream);
			cout << "num: " << num << endl;
		}
	}


	bool kopiraj_odom(){
		brojac = 0;
		std::ifstream infile("/media/kruno/Data/Downloads/dataset/poses/06.txt");
		std::string line;
		int br=0;
		while (std::getline(infile, line)){
			std::istringstream iss(line);
			double Alpha,Beta,Theta;
			double r11, r12, r13, r21, r22, r23, r31, r32, r33, x,y,z;
			if (!(iss >> r11 >> r12 >> r13 >> x >> r21 >> r22 >> r23 >> y >> r31 >> r32 >> r33 >> z)) { break; } // error
					if(!(brojac%brmod)){
						Alpha = atan2(r13, r33);
						Beta = asin(-r23);
						Theta = atan2(r21, r22);

						f_odom = (odom_format_ % "/media/kruno/Data/TEST/Kiti/Odom02" % "sl-" % br % "txt").str();
						fp = fopen(f_odom.c_str(),"w");
						fprintf(fp,"%lf %lf %lf %lf %lf %lf\n",x,y,z,Alpha,Beta,Theta);
						fclose(fp);
						br++;
						printf("zapisao odom %d %f %f %f %f %f %f\n",br,x,y,z,Alpha*RAD2DEG,Beta*RAD2DEG,Theta*RAD2DEG);
					}
					brojac++;
		}
	}

	bool kopiraj_pcd(){
		brojac = 0;
		for(int i = 0; i < br_pcd; i=i+brmod){
			f_pcl = (pcl_format_ % "/media/kruno/Data/Downloads/Kiti_src/dataset_laser/sequences/09/velodyne" % i % "bin").str();
			f_pcl_w = (pcl_format_w_ % "/media/kruno/Data/TEST/Kiti/PCD09" % "sl-" % brojac % "pcd").str();

			fp_write=fopen(f_pcl_w.c_str(),"w");
			int32_t num = 1000000;
			float *data = (float*)malloc(num*sizeof(float));
			// pointers
			float *px = data+0;
			float *py = data+1;
			float *pz = data+2;
			float *pr = data+3;

			FILE *stream;
			stream = fopen (f_pcl.c_str(),"rb");
			num = fread(data,sizeof(float),num,stream)/4;

			width = num;
			npoints = num;

			fprintf(fp_write, "# .PCD v0.7 - Point Cloud Data file format\n");
			fprintf(fp_write, "VERSION 0.7\n");

			fprintf(fp_write, "FIELDS x y z\n");
			fprintf(fp_write, "SIZE %d %d %d\n",4,4,4);
			fprintf(fp_write, "TYPE F F F\n");
			fprintf(fp_write, "COUNT %d %d %d\n", 1,1,1);
			fprintf(fp_write, "WIDTH %d\n", width);
			fprintf(fp_write, "HEIGHT %d\n", height);
			fprintf(fp_write, "VIEWPOINT %d %d %d %d %d %d %d\n",0,0,0,1,0,0,0);
			fprintf(fp_write, "POINTS %d\n", npoints);
			fprintf(fp_write, "DATA ascii\n");


			// allocate 4 MB buffer (only ~130*4*4 KB are needed)

			// load point cloud

			for (int32_t i=0; i<num; i++) {
				fprintf(fp_write,"%f %f %f\n",*px,*py,*pz);
				px+=4; py+=4; pz+=4; pr+=4;
			}
			fclose(stream);
			fclose(fp_write);
			cout << "written pcd " << brojac << " " << num << " to " << f_pcl_w << endl;
			brojac++;
		}
	}

};

int main(int argc, char* argv[]){

	ros::init(argc, argv, "bagcopy");
	ros::NodeHandle n;
	fileToPCD ftpcd;
	//ftpcd.kopiraj_odom();
	ftpcd.kopiraj_pcd();

	return 0;
}
