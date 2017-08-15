//
// Created by josip on 11.08.17..
//

#include "SLAMserver.h"

void SLAMserver::init_map_update(int agent_id){
    char line[400];
    double x_;
    snprintf(buffer, sizeof(buffer), "agent_%d/X.txt",agent_id);
    bool fileok = pFile = fopen((temp_path + buffer).c_str(),"r");
    int n = 0;
    vector<Eigen::Matrix4f> trajectory;
    if(fileok){
        bool read_ok = fgets(line, 400, pFile);
        agent_trajectory.points.clear();
        while(read_ok){
            n++;
            Eigen::Matrix4f M;
            M.setIdentity();
            sscanf(line, "%f %f %f %f %f %f %f %f %f %f %f %f\n", &M(0,0),&M(0,1),&M(0,2),&M(0,3),&M(1,0),&M(1,1),&M(1,2),&M(1,3),&M(2,0),&M(2,1),&M(2,2),&M(2,3));
            trajectory.push_back(M);
            p0.x = M(0,3);
            p0.y = M(1,3);
            p0.z = M(2,3);
            agent_trajectory.points.push_back(p0);
            read_ok = fgets(line, 400, pFile);
        }
        fclose(pFile);
    }
    Octomap octo;
    octo.header.frame_id = "map";
    octo.header.stamp = ros::Time::now();
    for (int i = 1; i < n; i++){//last_pcd_id
        PCLPointCloud cloud;
        snprintf(buffer, sizeof(buffer), "agent_%d/pcd_%d.pcd",agent_id,i);

        if(!(pcl::io::loadPCDFile<pcl::PointXYZ> ((temp_path + buffer).c_str(), cloud)==-1)){
            pcl::transformPointCloud(cloud, cloud, trajectory[i]);

            pcl::PassThrough<pcl::PointXYZ> pass;
            pass.setFilterFieldName("z");
            pass.setFilterLimits(-1.5, 1.0);
            pass.setInputCloud(cloud.makeShared());
            pass.filter(cloud);

            sensor_msgs::PointCloud2 cloud_msg;
            octomap::Pointcloud octo_cloud;

            pcl::toROSMsg(cloud,cloud_msg);
            octomap::pointCloud2ToOctomap(cloud_msg, octo_cloud);

            point3d origin (trajectory[i](0,3), trajectory[i](1,3), trajectory[i](2,3));

            double maxrange=-1; bool lazy_eval=false; bool discretize=false;
            m_octree->insertPointCloud(octo_cloud,origin,maxrange,lazy_eval,discretize);

            ROS_INFO("Param:");
            cout << param_res << endl;

            octomap_msgs::binaryMapToMsg(*m_octree, octo);
            m_markerPub.publish(octo);
        }
    }
    last_pcd_id = n;
    octomap_msgs::binaryMapToMsg(*m_octree, octo);
    m_markerPub.publish(octo);
}

void SLAMserver::update_map(){
    char line[400];
    int agent_id;
    char buffer_work[100];
    int agent_new0;
    bool update_done;

    while(ros::ok()){
        update_done = false;
        /*snprintf(buffer_work, sizeof(buffer_work), "agent_%d/new.txt",0);
        pFile = fopen((temp_path + buffer_work).c_str(),"r");
        bool ucitao = fgets(line, 400, pFile);
        sscanf(line, "%d\n", &agent_new0);
        fclose(pFile);*/


        if(agent_new0 || 1){
            printf("New data from agent 0 detected\n");
            init_map_update(0);
            trajectory0_pub.publish(agent_trajectory);
            update_done = true;
        }

        ros::Duration(0.2).sleep();
        break;
    }
}