//
// Created by josip on 11.08.17..
//

#ifndef COOPERATION_SERVER_SLAMSERVER_H
#define COOPERATION_SERVER_SLAMSERVER_H


#include <stdio.h>
#include <ctime>
#include <iostream>
#include <math.h>
#include <boost/thread/locks.hpp>
#include <boost/mpl/size.hpp>
#include <boost/ref.hpp>
#include <pthread.h>
#include "stopwatch.h"
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/BoundingBoxQuery.h>
#include <octomap_msgs/conversions.h>

#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
//#include "ProcessDefinitions.h"

using namespace std;
typedef pcl::PointCloud<pcl::PointXYZ> PCLPointCloud;
typedef octomap::OcTree OcTreeT;

using namespace octomap;
using octomap_msgs::Octomap;

class SLAMserver {

private:
    ros::NodeHandle n_;
    ros::Publisher trajectory0_pub,m_markerPub;

public:

    FILE *pFile;
    char buffer[200];
    int num_agents;
    string temp_path;
    visualization_msgs::Marker agent_trajectory;
    visualization_msgs::MarkerArray occupiedNodesVis;

    geometry_msgs::Point p0;

    octomap::OcTree* m_octree;

    double param_res;
    double param_probHit;
    double param_probMiss;
    double param_thresMin;
    double param_thresMax;

    double m_pointcloudMinZ;
    double m_pointcloudMaxZ;

    int last_pcd_id;

    PCLPointCloud pc;
    std_msgs::ColorRGBA m_color;

    SLAMserver(){

        last_pcd_id = 0;

        m_pointcloudMinZ = -std::numeric_limits<double>::max();
        m_pointcloudMaxZ = std::numeric_limits<double>::max();

        if(!ros::param::get("/cooperation_server/param_res", param_res)){
            param_res = 0.1; //0.1
        }
        if(!ros::param::get("/cooperation_server/param_probHit", param_probHit)){
            param_probHit = 0.63; //0.63
        }
        if(!ros::param::get("/cooperation_server/param_probMiss", param_probMiss)){
            param_probMiss = 0.36; //0.36
        }
        if(!ros::param::get("/cooperation_server/param_thresMin", param_thresMin)){
            param_thresMin = 0.12; //0.12
        }
        if(!ros::param::get("/cooperation_server/param_thresMax", param_thresMax)){
            param_thresMax = 0.97; //0.97
        }

        m_octree = new OcTree(param_res);
        m_octree->setProbHit(param_probHit);
        m_octree->setProbMiss(param_probMiss);
        m_octree->setClampingThresMin(param_thresMin);
        m_octree->setClampingThresMax(param_thresMax);

        m_color.r = 1.0;
        m_color.g = 0.0;
        m_color.b = 0.0;

        agent_trajectory.header.frame_id = "map";
        agent_trajectory.ns = "my";
        agent_trajectory.type = visualization_msgs::Marker::LINE_STRIP;
        agent_trajectory.action = visualization_msgs::Marker::ADD;

        agent_trajectory.pose.orientation.x = 0.0;
        agent_trajectory.pose.orientation.y = 0.0;
        agent_trajectory.pose.orientation.z = 0.0;

        agent_trajectory.color.a = 1.0;
        agent_trajectory.scale.x = agent_trajectory.scale.y = agent_trajectory.scale.z = 0.20;

        agent_trajectory.color.r = 1.0;
        agent_trajectory.color.g = 1.0;
        agent_trajectory.color.b = 0.0;

        agent_trajectory.id = 0;

        agent_trajectory.points.clear();

        trajectory0_pub = n_.advertise<visualization_msgs::Marker>("trajectory_0_m", 1);
        m_markerPub = n_.advertise<octomap_msgs::Octomap>("occupied_cells_vis_array", 1);

        if(!ros::param::get("/temp_path", temp_path)){
            temp_path  = "/home/josip/ftpFolder/lamor/";
        }
        if(!ros::param::get("/num_agents", num_agents)){
            num_agents = 2;
        }

        printf("agents: %d\n",num_agents);
    }

    void update_map();
    void init_map_update(int agent_id);

};


#endif //COOPERATION_SERVER_SLAMSERVER_H
