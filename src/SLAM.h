#include <stdio.h>
#include <ctime>
#include <iostream>
#include <math.h>
#include <pthread.h>
#include "stopwatch.h"
#include <queue>          // std::queue

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/tf.h>
#include "tf_conversions/tf_eigen.h"
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include "Common.h"
#include "CommonStruct.h"
#include "LESDSF.h"
#include "UnscentedTransform.h"
#include "ProcessDefinitions.h"
#include "ASLAMSE3.h"

#include <cv.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl_ros/point_cloud.h>
#include "ndt_matcher_d2d.h"
#include "FTPClient.h"

namespace enc = sensor_msgs::image_encodings;
using namespace std;

#define ACTIVE_SEARCH
//#define USE_IMAGE
//#define ACTIVE_SEARCH

struct odom_data{
	Eigen::Matrix<double, 4, 4> Tf;
	pcl::PointCloud<pcl::PointXYZ> cloud;
};

struct match_pair{
	int id1,id2;
	M_POSE_PLANE Pose;
	bool is_coop;
};

struct ASLAM_states{
	int state_id;
	double x,y,w,z;
	int e_dist,t_dist, dist;
};

struct compare_states
{
	inline bool operator() (const ASLAM_states& a, const ASLAM_states& b)
	{
		return (a.dist < b.dist);
	}
};

class SLAM {

private:
	ros::NodeHandle n_;

public:

	bool AUGMENT_FLAG, PREDICT_FLAG, ROT_FLAG, MATCH_NEIGHBOUR_STATES, MATCH_AS_STATES, do_marg, do_loop_marg;
	bool first_cloud,Callback_ready,Saving_pcd,use_odom_init;
	double AUG_S_THRESHOLD, AUG_TH_THRESHOLD, PRED_S_THRESHOLD, PRED_TH_THRESHOLD, AS_angle_scale, AS_max_euclidean;

	char buffer[100];
	int num_states, last_state_id, odom_state_id, last_loop_id, num_predict, num_augment, num_update;
	int pcd_save_id, current_pcd_num, sparse_every, agent_id, last_coop_loop;
	int AS_min_index_diff, AS_max_dist, AS_final_state_id, AS_min_topological;

	string temp_path, FTP_path;
	FILE *pFile;

	lslgeneric::NDTMatcherD2D matcherD2D;
	Eigen::Matrix<double, 4, 4> T_c,T_imu;
	pcl::PointCloud<pcl::PointXYZ> l_cloud,last_cloud;
	vector<pcl::PointCloud<pcl::PointXYZ> > map_clouds;
	std::queue<odom_data> odom_queue,callback_queue;
	nsFTP::CFTPClient ftpClient;
	nsFTP::CLogonInfo logonInfo;

	LESDSF_class LESDSF;
	StateVector u; // control input
	a_slam active_slam;

	vector<ASLAM_states> AS_states;
	vector<int> zatvorena_stanja;
	queue<match_pair> matched_pairs;
	vector<StateVector> slam_trajectory;
	boost::format pcl_format_, image_format_, odom_format_;
	pthread_t thread_update_map, thread_save_pcd;

	//ROS - variables
	nav_msgs::Path slam_path;
	ros::Publisher trajectory_pub, odom_pub;
	message_filters::Subscriber< sensor_msgs::PointCloud2 > pcd_sub_;
	message_filters::Subscriber< nav_msgs::Odometry > odom_sub_;

#ifdef USE_IMAGE
	message_filters::Subscriber< sensor_msgs::Image > image_sub_;
	typedef message_filters::sync_policies::ApproximateTime< sensor_msgs::PointCloud2, sensor_msgs::Image, nav_msgs::Odometry > MySyncPolicy;
#else
	typedef message_filters::sync_policies::ApproximateTime< sensor_msgs::PointCloud2, nav_msgs::Odometry > MySyncPolicy;
#endif
	message_filters::Synchronizer< MySyncPolicy > sync;


	SLAM() :
		pcd_sub_( n_, "/velodyne_points", 1 ),
		odom_sub_( n_, "/slam/odom", 1 ),
		sync( MySyncPolicy( 10 ), pcd_sub_, odom_sub_){
		sync.registerCallback( boost::bind( &SLAM::odom_callback, this, _1, _2) );

		pcl_format_.parse("%s%s%05i-PC.%s");
		image_format_.parse("%s%s%05i-LW.%s");
		odom_format_.parse("%s%s%05i-ODOM.%s");

		u.setZero();
		LESDSF.Init();
		printf("Filter initialized...\n");

		slam_trajectory.clear();
		slam_path.poses.clear();
		zatvorena_stanja.clear();

		AUGMENT_FLAG = false;
		ROT_FLAG = false;
		PRED_S_THRESHOLD = 3/1000; // 3 mm
		PRED_TH_THRESHOLD = 0.5 * PI/180; // 0.5 degrees
		odom_state_id = -1;
		last_loop_id = -1;
		last_coop_loop = 0;
		num_predict = num_augment = num_update = 0;

		first_cloud = true;
		Callback_ready=true;
		Saving_pcd = false;
		pcd_save_id = 0;
		logonInfo.SetHost("161.53.68.50",21,"kruno","Fer19581");
		T_c.setIdentity();

		if(!ros::param::get("/temp_path", temp_path))
			temp_path  = "/media/kruno/Data/TEST/SLAM/";
		if(!ros::param::get("/FTP_path", FTP_path))
			FTP_path  = "/ftpFolder/lamor/";
		if(!ros::param::get("/agent_id", agent_id))
			agent_id = 0;
		if(!ros::param::get("/use_odom_init", use_odom_init))
			use_odom_init = false;

		ros::param::get("/AUG_S_THRESHOLD", AUG_S_THRESHOLD);
		ros::param::get("/AUG_TH_THRESHOLD", AUG_TH_THRESHOLD);
		AUG_TH_THRESHOLD = AUG_TH_THRESHOLD * PI/180.0;

		ros::param::get("/MATCH_NEIGHBOUR_STATES", MATCH_NEIGHBOUR_STATES);
		ros::param::get("/MATCH_AS_STATES", MATCH_AS_STATES);

		ros::param::get("/AS_max_euclidean", AS_max_euclidean);
		ros::param::get("/AS_min_index_diff", AS_min_index_diff);
		ros::param::get("/AS_min_topological", AS_min_topological);
		if(!ros::param::get("/AS_angle_scale", AS_angle_scale))
			AS_angle_scale = 0.0;

		if(!ros::param::get("/do_marg", do_marg))
			do_marg = false;
		if(!ros::param::get("/do_loop_marg", do_loop_marg))
			do_loop_marg = false;
		ros::param::get("/sparse_every", sparse_every);

#ifndef USE_CPP_COV
		ros::param::get("/Kx", LESDSF.Kssx);
		ros::param::get("/Ky", LESDSF.Kssy);
		ros::param::get("/Kz", LESDSF.Kssz);
		ros::param::get("/Kalpha", LESDSF.Kalpha);
		ros::param::get("/Kbeta", LESDSF.Kbeta);
		ros::param::get("/Ktheta", LESDSF.Ktheta);

		LESDSF.Kssx =LESDSF.Kssx/1000.0;
		LESDSF.Kssy = LESDSF.Kssy/1000.0;
		LESDSF.Kssz = LESDSF.Kssz/1000.0;
		LESDSF.Kalpha = LESDSF.Kalpha*PI/180 * PI/180;
		LESDSF.Kbeta = LESDSF.Kbeta*PI/180 * PI/180;
		LESDSF.Ktheta = LESDSF.Ktheta*PI/180 * PI/180;
#endif


		active_slam.init(AS_max_euclidean,AS_min_index_diff,AS_angle_scale);

		//ROS - init
		snprintf(buffer, sizeof(buffer), "/slam/trajectory_%d",agent_id);
		trajectory_pub = n_.advertise<nav_msgs::Path>(buffer,1);
		odom_pub=n_.advertise<nav_msgs::Odometry>("/laser_odom",5);

		printf("SLAM system initialized with following parameters: \n");
		printf("Agent: %d\n",agent_id);
		printf("	AUG_S_THRESHOLD:		%f m\n",AUG_S_THRESHOLD);
		printf("	AUG_TH_THRESHOLD:		%f deg\n",AUG_TH_THRESHOLD*180/PI);
		printf("	MATCH_NEIGHBOUR_STATES:	%d\n",MATCH_NEIGHBOUR_STATES);
		printf("	MATCH_AS_STATES:		%d\n", MATCH_AS_STATES);
		printf("	AS_max_euclidean:		%f m\n", AS_max_euclidean);
		printf("	AS_min_index_diff:		%d\n", AS_min_index_diff);
		printf("	AS_min_topological:		%d m\n", AS_min_topological);
		printf("	AS_angle_scale: 		%f m\n", AS_angle_scale);

		printf("	do_marg:				%d m\n", do_marg);
		printf("	do_loop_marg:			%d m\n", do_loop_marg);
		printf("	sparse_every:			%d m\n", sparse_every);

		printf("	Kxx:					%f\n", LESDSF.Kssx);
		printf("	Kyy:					%f\n", LESDSF.Kssy);
		printf("	Kzz:					%f\n", LESDSF.Kssz);
		printf("	Kalpha:					%f\n", LESDSF.Kalpha);
		printf("	Kbeta:					%f\n", LESDSF.Kbeta);
		printf("	Ktheta:					%f\n", LESDSF.Ktheta);
		printf("\n");

	}

	void callback();
	void odom_callback(const sensor_msgs::PointCloud2ConstPtr& c_msg, const nav_msgs::Odometry::ConstPtr& odom);

	void checkAugmentCondition();
	bool SLAM_update(match_pair &corr_pose);
	void UpdateG();
	void add_trajectory_state(StateVector& xt_1);

	void set_num_states();
	void scan_coop_loop();
	bool nema_blizu_zatvorenih(int id);

	void fill_trajectory();
	void printTrajectory(const char* filename);
	void printTrajectoryAlgebra(char* filename);
	void printTrajectoryGroup(const char* filename);
};
