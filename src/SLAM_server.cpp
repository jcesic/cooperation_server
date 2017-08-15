#include "SLAM_server.h"

double SLAM_server::get_distance(SE3Mat &X1, SE3Mat &X2){
	SE3Mat dT = X1.inverse()*X2;
	double eTrans = sqrt(pow(dT(0,3),2)+pow(dT(2,3),2));
	double eRot = acos((dT.trace()/2)-1);
	return (eTrans + angle_scale*eRot);
}

bool SLAM_server::state_diff(SE3Mat &X1, SE3Mat &X2){
	SE3Mat dT = X1.inverse()*X2;
	double eTrans = sqrt(pow(dT(0,3),2)+pow(dT(2,3),2));
	double eRot = acos((dT.trace()/2)-1)*RAD2DEG;
	if((eTrans<state_t_TH)&&(eRot<state_angle_TH)){
		return false;
	}else{
		return true;
	}
}

bool SLAM_server::compare_Trajectories(int agent_id){
	for (int i = 0; i < agent_trajectories_old[agent_id].size(); i++){
		bool diff_state = state_diff(agent_trajectories[agent_id][i],agent_trajectories_old[agent_id][i]);
		if(diff_state){
			printf("change detected: %d %d\n",agent_id,i);
			return true;
		}
	}
	return false;
}

bool SLAM_server::check_indexes(int i, int j, int id1, int id2){
	for (int m = 0; m < close_loops.size(); m++){
		if((close_loops[m].id1==id1) && (close_loops[m].id2==id2)){
			if((abs(i-close_loops[m].i)<=min_index_TH)){
				if((abs(j-close_loops[m].j)<=min_index_TH)){
					return false;
				}
			}
		}
	}
	return true;
}

void SLAM_server::add_close(int a_1, int a_2, int s_1, int s_2){
	found_loops tmp_l;

	tmp_l.id1 = a_1;
	tmp_l.id2 = a_2;
	tmp_l.i=s_1;
	tmp_l.j=s_2;
	close_loops.push_back(tmp_l);
}

void SLAM_server::insert_pcd(int agent_id, int state_id){
	PCLPointCloud cloud;
	pcl::transformPointCloud(map_cloud[agent_id][state_id], cloud, agent_trajectories[agent_id][state_id]);
	octomap::Pointcloud octo_cloud;
	sensor_msgs::PointCloud2 cloud_msg;
	pcl::toROSMsg(cloud,cloud_msg);
	octomap::pointCloud2ToOctomap(cloud_msg, octo_cloud);
	point3d origin (agent_trajectories[agent_id][state_id](0,3), agent_trajectories[agent_id][state_id](1,3), agent_trajectories[agent_id][state_id](2,3));
	double maxrange=-1; bool lazy_eval=false; bool discretize=false;
	m_octree->insertPointCloud(octo_cloud, origin, maxrange, lazy_eval, discretize);
}

bool SLAM_server::find_loop(){
	bool new_found = false;
	int agent_1,agent_2;
	agent_1 = 0;
	agent_2 = 1;
	for(int i = 0; ((i < agent_trajectories[agent_1].size())&&!new_found); i++){
		SE3Mat x1G = agent_trajectories[agent_1][i];
		for(int j=0; ((j<agent_trajectories[agent_2].size())&&!new_found); j++){
			SE3Mat x2G = agent_trajectories[agent_2][j];
			double distance = get_distance(x1G,x2G);
			if(distance < max_euclidean_TH){
				if(check_indexes(i,j,agent_1,agent_2)){
					printf("Loop found: %d %d %d %d\n",i,j,agent_1,agent_2);

					Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> Tinit;

					SE3Mat dT,dXT,dF;
					SE3Mat x1G = agent_trajectories[agent_1][i];
					SE3Mat x2G = agent_trajectories[agent_2][j];
					Tinit.matrix() = x1G.inverse()*x2G;;

					bool successfull_match = matcherD2D.match(map_cloud[agent_1][i],map_cloud[agent_2][j],Tinit,true);
					add_close(agent_1,agent_2,i,j);
					dT = Tinit.matrix();

					//agent_1
					SE3Mat P1 = agent_trajectories[agent_1][0];
					dXT = x2G*dT.inverse();
					dF = P1.inverse()*dXT;

					snprintf(buffer, sizeof(buffer), "agent_%d/coop_loops.txt",agent_1);
					pFile = fopen((temp_path + buffer).c_str(),"a");
					fprintf(pFile,"%d %d %f %f %f %f %f %f %f %f %f %f %f %f\n",0,i,dF(0,0),dF(0,1),dF(0,2),dF(0,3),dF(1,0),dF(1,1),dF(1,2),dF(1,3),dF(2,0),dF(2,1),dF(2,2),dF(2,3));
					fclose(pFile);

					//agent_2
					SE3Mat P2 = agent_trajectories[agent_2][0];
					dXT = x1G*dT;
					dF = P2.inverse()*dXT;

					snprintf(buffer, sizeof(buffer), "agent_%d/coop_loops.txt",agent_2);
					pFile = fopen((temp_path + buffer).c_str(),"a");
					fprintf(pFile,"%d %d %f %f %f %f %f %f %f %f %f %f %f %f\n",0,j,dF(0,0),dF(0,1),dF(0,2),dF(0,3),dF(1,0),dF(1,1),dF(1,2),dF(1,3),dF(2,0),dF(2,1),dF(2,2),dF(2,3));
					fclose(pFile);

					new_found = true;
				}
			}
		}
	}
	return new_found;
}

void SLAM_server::init_map_update(int agent_id){
	char line[400];
	int id1,id2;
	SE3Mat x_;
	geometry_msgs::Point p0;

	x_.setIdentity();
	agent_trajectories[agent_id].clear();
	agent_trajectory_m[agent_id].points.clear();
	snprintf(buffer, sizeof(buffer), "agent_%d/X.txt",agent_id);
	pFile = fopen((temp_path + buffer).c_str(),"r");
	bool ucitao = fgets(line, 400, pFile);
	while(ucitao){
		sscanf(line,"%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",&x_(0,0),&x_(0,1),&x_(0,2),&x_(0,3),&x_(1,0),&x_(1,1),&x_(1,2),&x_(1,3),&x_(2,0),&x_(2,1),&x_(2,2),&x_(2,3));
		agent_trajectories[agent_id].push_back(x_);
		ucitao = fgets(line, 400, pFile);

		p0.x = x_(0,3);
		p0.y = x_(1,3);
		p0.z = x_(2,3);
		agent_trajectory_m[agent_id].points.push_back(p0);
	}
	fclose(pFile);
	for (int i = agent_trajectories_old[agent_id].size(); i < agent_trajectories[agent_id].size(); i++){
		PCLPointCloud cloud;
		snprintf(buffer, sizeof(buffer), "agent_%d/pcd_%d.pcd",agent_id,i);
		if(!(pcl::io::loadPCDFile<pcl::PointXYZ> ((temp_path + buffer).c_str(), cloud)==-1)){
			map_cloud[agent_id].push_back(cloud);
		}
	}

	if(!octomap_rebuild)
		octomap_rebuild = compare_Trajectories(agent_id);
}

void SLAM_server::rebuild_octomap(){
	m_octree->clear();
	for(int i = 0; i < num_agents; i++){
		for(int j =0; j < agent_trajectories[i].size(); j++){
			insert_pcd(i,j);
		}
	}
}

void SLAM_server::update_octomap(int agent_id){
	for (int i = agent_trajectories_old[agent_id].size(); i < agent_trajectories[agent_id].size(); i++){
		insert_pcd(agent_id,i);
	}
}

void SLAM_server::update_map(){
	char rename_buffer[100];
	bool update_done;
	while(ros::ok()){
		octomap_rebuild = false;

		for (int i = 0; i < num_agents; i++){
			snprintf(buffer, sizeof(buffer), "agent_%d/ready_NO.txt",i);
			pFile = fopen((temp_path + buffer).c_str(),"r");
			if(pFile){
				fclose(pFile);
				printf("New data from agent %d detected\n",i);
				init_map_update(i);
				printf("init done...\n");
				agents_update[i] = true;
				update_done = true;
			}else{
				printf("no new data %d\n",i);
			}
		}
		if(update_done){
			if(octomap_rebuild){
				rebuild_octomap();
			}else{
				for (int i = 0; i < num_agents; i++){
					if(agents_update[i]){
						update_octomap(i);
					}
				}
			}

			find_loop();
		}

		if(publish_map){
			Octomap octo;
			octo.header.frame_id = "map";
			octo.header.stamp = ros::Time::now();
			octomap_msgs::binaryMapToMsg(*m_octree, octo);
			m_markerPub.publish(octo);
		}

		for (int i = 0; i < num_agents; i++){
			if(agents_update[i]){
				agent_trajectories_old[i].clear();
				for(int j = 0; j < agent_trajectories[i].size();j++){
					agent_trajectories_old[i].push_back(agent_trajectories[i][j]);
				}
				trajectory_pub[i].publish(agent_trajectory_m[i]);

				agents_update[i]=false;
				snprintf(buffer, sizeof(buffer), "agent_%d/ready_NO.txt",i);
				snprintf(rename_buffer, sizeof(rename_buffer), "agent_%d/ready_YES.txt",i);
				rename((temp_path+buffer).c_str(),(temp_path+rename_buffer).c_str());
			}
		}

		ros::Duration(1).sleep();
	}
}

