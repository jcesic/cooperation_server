#include "SLAM.h"

void* slam_callback_thread(void *thread_data){
	class SLAM *tmp_class;
	tmp_class = (class SLAM *)thread_data;
	while(!tmp_class->callback_queue.empty()){
		odom_data tmp_data = tmp_class->callback_queue.front();
		tmp_class->callback_queue.pop();

		tmp_class->u(0) = tmp_data.Tf(0,3);
		tmp_class->u(1) = tmp_data.Tf(1,3);
		tmp_class->u(2) = tmp_data.Tf(2,3);
		RotMat tmpR = tmp_data.Tf.block(0,0,3,3);
		tmp_class->u.block(3,0,3,1) = tmp_class->LESDSF.SEMap.rot2angle(tmpR);
		tmp_class->callback();
		tmp_class->last_cloud = tmp_data.cloud;
#ifdef USE_ODOM_COVARIANCE
		RotMat tmpQTA;
		LESDSF.Qu_test.setZero();
		for(int j = 0; j < 3; j++){
			for(int k=0; k<3; k++){
				if(j==k){
					LESDSF.Qu_test(j,k) = odom->pose.covariance[j*3+k];
					LESDSF.Qu_test(j+3,k+3) = odom->pose.covariance[j*3+k+9];
					tmpQTA(j,k) = odom->pose.covariance[j*3+k+18];
				}
			}
		}
		//LESDSF.Qu_test.block(0,3,3,3) = tmpQTA;
		//LESDSF.Qu_test.block(3,0,3,3) = tmpQTA.transpose();
#endif
	}
	tmp_class->Callback_ready = true;
}

void* save_pcd_thread(void *thread_data){
	class SLAM *tmp_class;
	char buffer[100],buffer2[100];
	tmp_class = (class SLAM *)thread_data;

	bool success = tmp_class->ftpClient.Login(tmp_class->logonInfo);
	if(success){
		cout << "server login OK" << endl;
		snprintf(buffer, sizeof(buffer), "agent_%d/ready_YES.txt",tmp_class->agent_id);
		bool server_ready = tmp_class->ftpClient.DownloadFile((tmp_class->FTP_path+buffer).c_str(), (tmp_class->temp_path+buffer).c_str());
		if(server_ready){
			for (int i = tmp_class->pcd_save_id; i < tmp_class->current_pcd_num; i++){

				snprintf(buffer, sizeof(buffer), "agent_%d/pcd_%d.pcd",tmp_class->agent_id,i);
				pcl::io::savePCDFileASCII ((tmp_class->temp_path+buffer).c_str(),tmp_class->map_clouds[i]);
				tmp_class->ftpClient.UploadFile((tmp_class->temp_path+buffer).c_str(), (tmp_class->FTP_path+buffer).c_str());
				cout << "upload finish" <<endl;
			}

			snprintf(buffer,sizeof(buffer), "agent_%d/X.txt",tmp_class->agent_id);
			tmp_class->ftpClient.UploadFile((tmp_class->temp_path+buffer).c_str(), (tmp_class->FTP_path+buffer).c_str());
			snprintf(buffer,sizeof(buffer),"agent_%d/ready_YES.txt",tmp_class->agent_id);
			snprintf(buffer2,sizeof(buffer2),"agent_%d/ready_NO.txt",tmp_class->agent_id);
			tmp_class->ftpClient.Rename((tmp_class->FTP_path+buffer).c_str(), (tmp_class->FTP_path+buffer2).c_str());
			tmp_class->pcd_save_id = tmp_class->current_pcd_num;
			tmp_class->ftpClient.Logout();
		}
	}else{
		cout << "server not responding" << endl;
	}
	tmp_class->Saving_pcd = false;
}

void SLAM::scan_coop_loop(){
	snprintf(buffer, sizeof(buffer), "server_models/agent_%d/coop_loops.txt",agent_id);
	bool ucitao;
	char line[400];
	vector<match_pair> loops;
	int id1,id2;
	int num = 0;
	Matrix4d tmpRT;
	tmpRT.setIdentity();
	float r11,r12,r13,r14,r21,r22,r23,r24,r31,r32,r33,r34;

	loops.clear();

	pFile=fopen((temp_path + buffer).c_str(),"r");
	ucitao=fgets(line, 400, pFile);
	while(ucitao){
		num++;
		match_pair curr_match_pair;
		sscanf(line, "%d %d %f %f %f %f %f %f %f %f %f %f %f %f\n", &id1,&id2, &r11, &r12, &r13, &r14, &r21, &r22, &r23, &r24, &r31, &r32, &r33, &r34);
		curr_match_pair.Pose.Reset();
		curr_match_pair.Pose.m_Rot(0,0) = r11; curr_match_pair.Pose.m_Rot(0,1) = r12; curr_match_pair.Pose.m_Rot(0,2) = r13; curr_match_pair.Pose.m_X(0) = r14;
		curr_match_pair.Pose.m_Rot(1,0) = r21; curr_match_pair.Pose.m_Rot(1,1) = r22; curr_match_pair.Pose.m_Rot(1,2) = r23; curr_match_pair.Pose.m_X(1) = r24;
		curr_match_pair.Pose.m_Rot(2,0) = r31; curr_match_pair.Pose.m_Rot(2,1) = r32; curr_match_pair.Pose.m_Rot(2,2) = r33; curr_match_pair.Pose.m_X(2) = r34;
		curr_match_pair.id1 = id1;
		curr_match_pair.id2 = id2;
		/*ucitao=fgets(line, 400, pFile);
		sscanf(line, "%f %f %f %f %f %f %f %f %f\n", &r11, &r12, &r13, &r21, &r22, &r23, &r31, &r32, &r33);
		curr_match_pair.Pose.translation_C(0,0) = r11; curr_match_pair.Pose.translation_C(0,1) = r12; curr_match_pair.Pose.translation_C(0,2) = r13;
		curr_match_pair.Pose.translation_C(1,0) = r21; curr_match_pair.Pose.translation_C(1,1) = r22; curr_match_pair.Pose.translation_C(1,2) = r23;
		curr_match_pair.Pose.translation_C(2,0) = r31; curr_match_pair.Pose.translation_C(2,1) = r32; curr_match_pair.Pose.translation_C(2,2) = r33;
		ucitao=fgets(line, 400, pFile);
		sscanf(line, "%f %f %f %f %f %f %f %f %f\n", &r11, &r12, &r13, &r21, &r22, &r23, &r31, &r32, &r33);
		curr_match_pair.Pose.angles_C(0,0) = r11; curr_match_pair.Pose.angles_C(0,1) = r12; curr_match_pair.Pose.angles_C(0,2) = r13;
		curr_match_pair.Pose.angles_C(1,0) = r21; curr_match_pair.Pose.angles_C(1,1) = r22; curr_match_pair.Pose.angles_C(1,2) = r23;
		curr_match_pair.Pose.angles_C(2,0) = r31; curr_match_pair.Pose.angles_C(2,1) = r32; curr_match_pair.Pose.angles_C(2,2) = r33;
		ucitao=fgets(line, 400, pFile);
		sscanf(line, "%f %f %f %f %f %f %f %f %f\n", &r11, &r12, &r13, &r21, &r22, &r23, &r31, &r32, &r33);
		curr_match_pair.Pose.t_a_C(0,0) = r11; curr_match_pair.Pose.t_a_C(0,1) = r12; curr_match_pair.Pose.t_a_C(0,2) = r13;
		curr_match_pair.Pose.t_a_C(1,0) = r21; curr_match_pair.Pose.t_a_C(1,1) = r22; curr_match_pair.Pose.t_a_C(1,2) = r23;
		curr_match_pair.Pose.t_a_C(2,0) = r31; curr_match_pair.Pose.t_a_C(2,1) = r32; curr_match_pair.Pose.t_a_C(2,2) = r33;*/
		loops.push_back(curr_match_pair);
		ucitao=fgets(line, 400, pFile);
	}
	fclose(pFile);
	for(int i = last_coop_loop; i < loops.size(); i++){
		match_pair curr_match_pair = loops[i];
		curr_match_pair.is_coop = true;
		curr_match_pair.Pose.translation_C = curr_match_pair.Pose.translation_C * 0.02 * 0.02;
		curr_match_pair.Pose.angles_C = curr_match_pair.Pose.angles_C * 0.1*0.1*DEG2RAD*DEG2RAD;
		ROS_WARN("COOP LOOP: %d %d\n",curr_match_pair.id1,curr_match_pair.id2);
		matched_pairs.push(curr_match_pair);
	}
	last_coop_loop=loops.size();
}

bool SLAM::nema_blizu_zatvorenih(int id){
	for (int i =0; i<zatvorena_stanja.size();i++){
		if(abs(id-zatvorena_stanja[i])<0)//AS_min_index_diff)
			return false;
	}
	return true;
}

void SLAM::printTrajectoryAlgebra(char* filename){
	FILE* pFile;
	pFile = fopen(filename,"w");
	for(int i=0; i<slam_trajectory.size();i++){
		StateVector xt_ = slam_trajectory[i];
		fprintf(pFile,"%12.8lf %12.8lf %12.8lf %12.8lf %12.8lf %12.8lf\n", xt_(0),xt_(1),xt_(2),xt_(3),xt_(4),xt_(5));
	}
	fclose(pFile);
}

void SLAM::printTrajectoryGroup(const char* filename){
	FILE* pFile;
	pFile = fopen(filename,"w");
	for(int i=0; i<slam_trajectory.size();i++){
		StateVector xt_ = slam_trajectory[i];
		SE3Mat xtG_ = LESDSF.SEMap.exp(xt_);
		for(int j = 0; j < 3; j++){
			fprintf(pFile,"%12.8f %12.8f %12.8f %12.8lf ",xtG_(j,0),xtG_(j,1),xtG_(j,2),xtG_(j,3));
		}
		fprintf(pFile,"\n");
	}
	fclose(pFile);
}

void SLAM::printTrajectory(const char* filename){
	FILE* pFile;
	pFile = fopen(filename,"w");
	printf("Total track size: %ld\n",LESDSF.n.rows()/LESDSF.dimPose);
	printf("Total updates: %d\n",LESDSF.total_updates);
	cout << "Saving track...  " << filename << endl;
	for(int i=0; i<num_states;i++){
		StateVector xt_ = LESDSF.X.block(i*LESDSF.dimPose,0,LESDSF.dimPose,1);
		SE3Mat xtG_ = LESDSF.SEMap.exp(xt_);
		for(int j = 0; j < 3; j++){
			fprintf(pFile,"%12.8lf %12.8lff %12.8lf %12.8lf\n",xtG_(j,0),xtG_(j,1),xtG_(j,2),xtG_(j,3));
		}
	}
	fclose(pFile);
}

void SLAM::UpdateG(){
	active_slam.G.resize(LESDSF.L.rows()/LESDSF.dimPose-1,LESDSF.L.cols()/LESDSF.dimPose-1);
	active_slam.G.setZero();
	for (int k=0; k < LESDSF.L.outerSize(); ++k){
		for (Eigen::SparseMatrix<double>::InnerIterator it(LESDSF.L,k); it; ++it){
			double a = it.value();
			if(a!=0.0){
				int row_id = it.row();
				int col_id = it.col();
				row_id = row_id/LESDSF.dimPose;
				col_id = col_id/LESDSF.dimPose;
				if((row_id<active_slam.G.rows())&&(col_id<active_slam.G.cols()))
					active_slam.G(row_id,col_id)=1;
			}
		}
	}
}

void SLAM::add_trajectory_state(StateVector& xt_1){
	slam_trajectory.push_back(xt_1);

	geometry_msgs::PoseStamped cpose;
	slam_path.header.stamp = ros::Time::now();
	slam_path.header.frame_id = "map";
	cpose.header = slam_path.header;

	StateVector xt_ = LESDSF.SEMap.logV(LESDSF.SEMap.exp(xt_1));
	cpose.pose.position.x = xt_(0,0);
	cpose.pose.position.y = xt_(1,0);
	cpose.pose.position.z = xt_(2,0);

	tf::quaternionTFToMsg(tf::createQuaternionFromRPY(xt_(5,0),xt_(3,0),xt_(4,0)), cpose.pose.orientation);
	slam_path.poses.push_back(cpose);
}

void SLAM::set_num_states(){
	num_states = LESDSF.n.rows()/LESDSF.dimPose-1;
	last_state_id = num_states-1;
}

void SLAM::odom_callback(const sensor_msgs::PointCloud2ConstPtr& c_msg, const nav_msgs::Odometry::ConstPtr& odom){
	tf::Quaternion q(odom->pose.pose.orientation.x,odom->pose.pose.orientation.y,odom->pose.pose.orientation.z,odom->pose.pose.orientation.w);
	tf::Matrix3x3 cRot;
	cRot.setRotation(q);
	Eigen::Matrix3d IMU_rot;
	Eigen::Matrix4d cIMU,dIMU;
	cIMU.setIdentity();
	tf::matrixTFToEigen(cRot,IMU_rot);

	cIMU.block(0,0,3,3) = IMU_rot;
	cIMU(0,3)=odom->pose.pose.position.x;
	cIMU(1,3)=odom->pose.pose.position.y;
	cIMU(2,3)=odom->pose.pose.position.z;

	odom_data tmp_data;
	pcl::fromROSMsg(*c_msg, tmp_data.cloud);
	if(!first_cloud){
		dIMU = T_imu.inverse()*cIMU;
		Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> Tinit;
		Tinit.matrix() = dIMU;
		bool ret = matcherD2D.match(l_cloud,tmp_data.cloud,Tinit,use_odom_init);
		tmp_data.Tf = Tinit.matrix();
		//		Eigen::MatrixXd cov(6,6); //column vectors, pose_increment_v(6,1)
		//		matcherD2D.covariance(l_cloud,tmp_data.cloud, Tinit, cov);

		odom_queue.push(tmp_data);
		if(Callback_ready){
			Callback_ready = false;
			while(!odom_queue.empty()){
				callback_queue.push(odom_queue.front());
				odom_queue.pop();

			}
			int pm = pthread_create(&thread_update_map, NULL, slam_callback_thread, (void *)this);
		}
		T_c = T_c*tmp_data.Tf;
	}else{
		first_cloud = false;
		tmp_data.Tf.setIdentity();
		callback_queue.push(tmp_data);
		Callback_ready=false;
		int pm = pthread_create(&thread_update_map, NULL, slam_callback_thread, (void *)this);
		T_c.setIdentity();
	}
	T_imu = cIMU;

	nav_msgs::Odometry odometry;
	odometry.header.stamp = ros::Time::now();
	odometry.pose.pose.position.x = T_c(0,3);
	odometry.pose.pose.position.y = T_c(1,3);
	odometry.pose.pose.position.z = T_c(2,3);
	odom_pub.publish(odometry);

	l_cloud = tmp_data.cloud;
}

void SLAM::callback(){

	odom_state_id++;
	LESDSF.MotionModel(u);
	set_num_states();

	if (AUGMENT_FLAG) {
		num_augment++;

		const clock_t begin_augment = clock();
		LESDSF.AugmentState();
		StateVector xt_1 = LESDSF.x.block(LESDSF.x.rows()-2*LESDSF.dimPose, 0, LESDSF.dimPose, 1);
		add_trajectory_state(xt_1);
		set_num_states();
		map_clouds.push_back(last_cloud);

		if(num_states>1){ // ukoliko postoji vise stanja pokusaj zatvoriti petlju
			match_pair curr_match_pair;
			curr_match_pair.id2 = last_state_id;
			curr_match_pair.is_coop=false;
			curr_match_pair.Pose.Reset();
			trajectory_pub.publish(slam_path);

			if(MATCH_NEIGHBOUR_STATES){
				curr_match_pair.id1 = last_state_id-1;
				matched_pairs.push(curr_match_pair);
			}

#ifdef USE_COOP
			scan_coop_loop();
#endif

			if(nema_blizu_zatvorenih(last_state_id)){
				AS_final_state_id = -1;
				AS_states.clear();
				priority_queue<node_blizu> moguci_cvorovi = active_slam.nadi_blizu(slam_trajectory);
				AS_max_dist = 0;
				ASLAM_states curr_AS;
				printf("AS: Found %ld states close enough\n",moguci_cvorovi.size());
				while(moguci_cvorovi.size()>0){
					curr_AS.dist = 0;
					curr_AS.state_id = moguci_cvorovi.top().getid();
					curr_AS.e_dist = moguci_cvorovi.top().getPriority();
					printf("	AS: State %ld  ID: %d  E_DIST: %d mm\n",moguci_cvorovi.size(),curr_AS.state_id,curr_AS.e_dist);
#ifdef ACTIVE_SEARCH
					curr_AS.dist = last_state_id-curr_AS.state_id;
					AS_states.push_back(curr_AS);
#endif
#ifdef ACTIVE_SLAM
					loop_path curr_loop = active_slam.pathFind(curr_AS.state_id,slam_path);
					printf("	AS: State %ld  ID: %d  T_DIST: %d mm\n",moguci_cvorovi.size(),curr_AS.state_id,curr_loop.top_distance);
					if (curr_loop.top_distance > AS_min_topological){
						curr_AS.dist = curr_loop.top_distance;
						curr_AS.t_dist = curr_loop.top_distance;
						AS_states.push_back(curr_AS);
						printf("	AS: State %ld  ID: %d  ADDED %d\n",moguci_cvorovi.size(),curr_AS.state_id,curr_loop.top_distance);
					}
#endif
					if(curr_AS.dist>AS_max_dist){
						AS_max_dist = curr_AS.dist;
						AS_final_state_id = curr_AS.state_id;
					}
					moguci_cvorovi.pop();
				}
				std::sort(AS_states.begin(), AS_states.end(), compare_states());
				if(AS_states.size()>0 && (MATCH_AS_STATES)){
					zatvorena_stanja.push_back(last_state_id);
					curr_match_pair.id1 = AS_final_state_id;
					matched_pairs.push(curr_match_pair);
					printf("	AS: State %d was added to que!\n",AS_final_state_id);
				}
				ROS_WARN("AS: FS %d\n",AS_final_state_id);

			}
		}
	}else{
		num_predict++;
		LESDSF.Predict();
	}

	while(!matched_pairs.empty()){
		match_pair curr_match_pair = matched_pairs.front();
		matched_pairs.pop();

		bool successfull_match;
		if(!curr_match_pair.is_coop){
			Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> Tout;
			Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> Tinit;
			Tout.setIdentity();

			StateVector x1g = slam_trajectory[curr_match_pair.id1];
			StateVector x2g = slam_trajectory[curr_match_pair.id2];
			SE3Mat x1G = LESDSF.SEMap.exp(x1g);
			SE3Mat x2G = LESDSF.SEMap.exp(x2g);
			SE3Mat dT = x1G.inverse()*x2G;
			Tinit.matrix() = dT;

			successfull_match = matcherD2D.match(map_clouds[curr_match_pair.id1],map_clouds[curr_match_pair.id2],Tinit,true);
			Tout=Tinit;

			curr_match_pair.Pose.m_Rot = Tout.matrix().block(0,0,3,3);
			curr_match_pair.Pose.m_X = Tout.matrix().block(0,3,3,1);
			Eigen::MatrixXd cov(6,6); //column vectors, pose_increment_v(6,1)
			matcherD2D.covariance(map_clouds[curr_match_pair.id1],map_clouds[curr_match_pair.id2], Tinit, cov);

			curr_match_pair.Pose.translation_C = cov.block(0,0,3,3);
			curr_match_pair.Pose.angles_C = cov.block(3,3,3,3);
			curr_match_pair.Pose.t_a_C = cov.block(0,3,3,3);
			curr_match_pair.Pose.UpdatePTRLL();
		}else{
			successfull_match =true;
		}
		if(successfull_match){
			SLAM_update(curr_match_pair);
			num_update++;
		}
	}

	if((!Saving_pcd)&&(pcd_save_id<map_clouds.size())){
		Saving_pcd = true;
		snprintf(buffer, sizeof(buffer), "agent_%d/X.txt",agent_id);
		printTrajectoryGroup((temp_path+buffer).c_str());
		current_pcd_num = map_clouds.size();
		int pm = pthread_create(&thread_save_pcd, NULL, save_pcd_thread, (void *)this);
	}

	AUGMENT_FLAG = false;
	checkAugmentCondition();
	if(last_state_id==-1){
		AUGMENT_FLAG = true;
	}

	if(slam_trajectory.size()>0 && (agent_id==0)){
		StateVector xt_ = LESDSF.SEMap.logV(LESDSF.SEMap.exp(slam_trajectory[0]));
		tf::Quaternion map_0_quat = tf::createQuaternionFromRPY(xt_(5,0),xt_(3,0),xt_(4,0));
		tf::Vector3 map0_vec(xt_(0,0),xt_(1,0),xt_(2,0));
		tf::Transform tf_map0(map_0_quat, map0_vec);
		tf::StampedTransform tf_map0_final(tf_map0,ros::Time::now(), "/map", "/map_0");
		static tf::TransformBroadcaster odom_broadcaster, map_0_broadcaster;
		map_0_broadcaster.sendTransform(tf_map0_final);
	}

	printf("Done with odom %d : p a u %d %d %d\n",odom_state_id,num_predict,num_augment,num_update);
}


bool SLAM::SLAM_update(match_pair &corr_pose){
	bool marginalized_ok = false;
	printf("VISUAL UPDATE between indexes %d %d started\n",corr_pose.id1,corr_pose.id2);

	StateMat z_C,z_Cg;
	StateVector zg,zg_ut;
	SE3Mat zG;
	zG.setIdentity();
	zG.block(0,0,3,3)=corr_pose.Pose.m_Rot;
	zG.block(0,3,3,1) = corr_pose.Pose.m_X;

	zg.block(0,0,3,1) = corr_pose.Pose.m_X;
	zg(3,0) = corr_pose.Pose.m_Beta;
	zg(4,0) = corr_pose.Pose.m_Alpha;
	zg(5,0) = corr_pose.Pose.m_Theta;

	z_C.setZero();
	z_C.block(0,0,3,3)=corr_pose.Pose.translation_C;
	z_C.block(3,3,3,3)=corr_pose.Pose.angles_C;
	z_C.block(0,3,3,3)=(corr_pose.Pose.t_a_C);
	z_C.block(3,0,3,3)=(corr_pose.Pose.t_a_C).transpose();

	UT ut;
	ut.UTGroupToAlgebra(zg,z_C,zg_ut,z_Cg);

	LESDSF.Update(corr_pose.id1,corr_pose.id2,zG,z_Cg, slam_trajectory);
	ros::param::set("/do_map_update", 1);

	if(do_loop_marg){
		if(last_loop_id>=0){
			if(!(LESDSF.total_updates%sparse_every)){
				LESDSF.MarginalizeO(last_loop_id,false);
			}else
				LESDSF.MarginalizeO(last_loop_id,true);
			marginalized_ok = true;
		}
	}

	slam_trajectory.clear();
	slam_path.poses.clear();
	set_num_states();

	for(int i=0; i<num_states;i++){
		StateVector xt_ = LESDSF.X.block(i*LESDSF.dimPose,0,LESDSF.dimPose,1);
		add_trajectory_state(xt_);
	}

	int total_removed,current_removed;
	total_removed = current_removed = 0;
	if((last_loop_id>0) && do_marg){
		vector<block_marg> moguci_cvorovi;
		if(marginalized_ok)
			active_slam.find_between(slam_trajectory,last_loop_id,corr_pose.id2-2,moguci_cvorovi);
		else{
			//if((corr_pose.id2-1-last_loop_id-1)<=0)
			//ROS_ERROR("wrong indexes: %d %d",corr_pose.id2-1,last_loop_id+1);
			active_slam.find_between(slam_trajectory,last_loop_id+1,corr_pose.id2-1,moguci_cvorovi);
		}
		if((moguci_cvorovi.size()>0)){

			for(int i = 0; i < moguci_cvorovi.size(); i++){
				printf("marginalize block %d  %d %d\n",i,moguci_cvorovi[i].begin-total_removed,moguci_cvorovi[i].end-total_removed);
				//LESDSF.MarginalizeO(false,moguci_cvorovi[i].getid()-i,slam_trajectory,true);
				LESDSF.MarginalizeS(false,moguci_cvorovi[i].begin-total_removed,moguci_cvorovi[i].end-total_removed,slam_trajectory);
				current_removed = moguci_cvorovi[i].end-moguci_cvorovi[i].begin + 1;
				total_removed = total_removed + moguci_cvorovi[i].end-moguci_cvorovi[i].begin + 1;
			}
			slam_trajectory.clear();
			slam_path.poses.clear();
			set_num_states();
			for(int i=0; i<num_states;i++){
				StateVector xt_ = LESDSF.X.block(i*LESDSF.dimPose,0,LESDSF.dimPose,1);
				add_trajectory_state(xt_);
			}
		}
	}

	trajectory_pub.publish(slam_path);

	last_loop_id = corr_pose.id2-total_removed;
	if(marginalized_ok)
		last_loop_id = last_loop_id - 1;

	return true;
}

void SLAM::checkAugmentCondition() {
	int N = LESDSF.x.rows();
	StateVector xt_1;
	if (N > LESDSF.dimPose){ // at least two states
		xt_1 = LESDSF.x.block(N-2*LESDSF.dimPose, 0, LESDSF.dimPose, 1);
		xt_1 = LESDSF.SEMap.logV(LESDSF.SEMap.exp(xt_1));
	}else{
		xt_1 << 0, 0, 0; // initial state in SLAM
	}
	StateVector xt_ = LESDSF.x.block(N-LESDSF.dimPose, 0, LESDSF.dimPose, 1);
	xt_ = LESDSF.SEMap.logV(LESDSF.SEMap.exp(xt_));

	Vector3d deltaX_a = (xt_-xt_1).block(0, 0, 3, 1);
	Vector3d deltaTheta_a = (xt_-xt_1).block(3, 0, 3, 1);

	AUGMENT_FLAG = deltaX_a.norm() >= AUG_S_THRESHOLD || deltaTheta_a.norm() >= AUG_TH_THRESHOLD;
	//ROT_FLAG = fabs(deltaTheta_a) >= AUG_TH_THRESHOLD;
	//AUGMENT_FLAG = true;
}
