#include <stdio.h>
#include <ctime>
#include <iostream>
#include <math.h>
#include <pthread.h>
#include "stopwatch.h"
#include <vector>

#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/BoundingBoxQuery.h>
#include <octomap_msgs/conversions.h>
#include <visualization_msgs/Marker.h>

#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include "SE3Mappings.h"
#include "CommonStruct.h"
#include "ProcessDefinitions.h"
#include "UnscentedTransform.h"
#include "ndt_matcher_d2d.h"

typedef pcl::PointCloud<pcl::PointXYZ> PCLPointCloud;

using namespace std;
using namespace octomap;
using octomap_msgs::Octomap;

struct found_loops{
	int id1,id2;
	int i,j;
};

class SLAM_server {

private:
	ros::NodeHandle n_;
	ros::Publisher m_markerPub,trajectory0_pub;
	vector<ros::Publisher> trajectory_pub;
public:

	char buffer[400];
	FILE *pFile;
	int num_agents;
	bool publish_map, octomap_rebuild;
	vector<bool> agents_update;
	double angle_scale, state_angle_TH, state_t_TH,max_euclidean_TH,min_index_TH;
	string temp_path;
	sensor_msgs::PointCloud2 last_cloud;
	vector<visualization_msgs::Marker> agent_trajectory_m;

	vector<vector<SE3Mat> > agent_trajectories,agent_trajectories_old;
	vector<found_loops> close_loops;
	vector<vector<PCLPointCloud> > map_cloud;

	octomap::OcTree* m_octree;
	double m_res;
	double m_probHit;
	double m_probMiss;
	double m_thresMin;
	double m_thresMax;


	lslgeneric::NDTMatcherD2D matcherD2D;

	SLAM_server(){
		if(!ros::param::get("/temp_path", temp_path))
			temp_path  = "/media/kruno/Data/TEST/SLAM/coop_test/";

		if(!ros::param::get("/num_agents", num_agents))
			num_agents = 1;
		if(!ros::param::get("/publish_map", publish_map))
			publish_map = false;

		if(!ros::param::get("/angle_scale", angle_scale))
			angle_scale = 0.0;
		if(!ros::param::get("/state_angle_TH", state_angle_TH))
			state_angle_TH = 2.0;
		if(!ros::param::get("/state_t_TH", state_t_TH))
			state_t_TH = 0.1;
		if(!ros::param::get("/max_euclidean_TH", max_euclidean_TH))
			max_euclidean_TH = 11.0;
		if(!ros::param::get("/min_index_TH", min_index_TH))
			min_index_TH = 14;

		if(!ros::param::get("/m_res", m_res))
			m_res = 0.5;
		if(!ros::param::get("/m_probHit", m_probHit))
			m_probHit = 0.63;
		if(!ros::param::get("/m_probMiss", m_probMiss))
			m_probMiss = 0.36;
		if(!ros::param::get("/m_thresMin", m_thresMin))
			m_thresMin = 0.12;
		if(!ros::param::get("/m_thresMax", m_thresMax))
			m_thresMax = 0.97;

		printf("read params...\n");
		m_octree = new OcTree(m_res);
		m_octree->setProbHit(m_probHit);
		m_octree->setProbMiss(m_probMiss);
		m_octree->setClampingThresMin(m_thresMin);
		m_octree->setClampingThresMax(m_thresMax);

		octomap_rebuild = false;

		vector<SE3Mat> tmp_trajectory;
		vector<PCLPointCloud> tmp_map;
		visualization_msgs::Marker tmp_trajectory_m;
		tmp_trajectory.clear();

		tmp_trajectory_m.header.frame_id = "map";
		tmp_trajectory_m.ns = "my";
		tmp_trajectory_m.type = visualization_msgs::Marker::LINE_STRIP;
		tmp_trajectory_m.action = visualization_msgs::Marker::ADD;
		tmp_trajectory_m.color.a = 1.0;
		tmp_trajectory_m.scale.x = tmp_trajectory_m.scale.y = tmp_trajectory_m.scale.z = 0.15;
		tmp_trajectory_m.points.clear();

		for(int i = 0; i < num_agents; i++){
			agent_trajectories.push_back(tmp_trajectory);
			agent_trajectories_old.push_back(tmp_trajectory);
			map_cloud.push_back(tmp_map);
			snprintf(buffer, sizeof(buffer), "agent_%d/ready_YES.txt",i);
			pFile = fopen((temp_path + buffer).c_str(),"w");
			fclose(pFile);
			agents_update.push_back(false);


			tmp_trajectory_m.color.r = 1.0;
			tmp_trajectory_m.color.g = 1.0;
			tmp_trajectory_m.color.b = 0.0;
			tmp_trajectory_m.id = i;
			agent_trajectory_m.push_back(tmp_trajectory_m);
			trajectory_pub.push_back(trajectory0_pub);
		}

		printf("agents: %d\n",num_agents);

		for(int i = 0; i < num_agents; i++){
			snprintf(buffer, sizeof(buffer), "trajectory_%d_m",i);
			trajectory_pub[i] = n_.advertise<visualization_msgs::Marker>(buffer,1);
		}

		m_markerPub = n_.advertise<octomap_msgs::Octomap>("occupied_cells_vis_array", 1);
	}

	void add_close(int a_1, int a_2, int s_1, int s_2);
	bool compare_Trajectories(int agent_id);
	bool state_diff(SE3Mat &X1, SE3Mat &X2);
	bool check_indexes(int i, int j, int id1, int id2);
	void update_map();
	void fill_trajectory(int agent_id);
	void init_map_update(int agent_id);
	double get_distance(SE3Mat& X1, SE3Mat& X2);
	bool find_loop();

	void insert_pcd(int agent_id, int state_id);
	void rebuild_octomap();
	void update_octomap(int agent_id);
};
