#include "ProcessDefinitions.h"
#define VELODYNE64
//C++ headers
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <stdio.h>
#include <time.h>
#include <vector>
#include <list>
#include <cstdio>
#include <typeinfo>
#include <ctime>
//#include "UnscentedTransform.h"

#include <flann/flann.hpp>

#include "RVLCore.h"
#include "RVLPCS.h"
#include "RVLRLM.h"
#include "RVLPSuLMBuilder.h"
#include "RVLPSuLMGroundTruth.h"
#include "RVLPSuLMVS.h"

#include <Eigen/Eigen> // whole Eigen matrix library (dense and sparse)

#include "PCD_add.h"
#include "clipper.hpp"
#include <visualization_msgs/Marker.h>
#include "CommonStruct.h"

//Additional Headers
#ifdef USE_VTK
#include "VTK.h"
#endif

//namespaces
using namespace std;
using Eigen::MatrixXd;
using namespace boost::interprocess;
using namespace Eigen;
using namespace ClipperLib;

#ifndef PI
#define PI	3.14159265358979323846
#endif

#ifndef _2PI
#define _2PI	6.28318530717959
#endif

struct MODEL{
	CRVLPSuLM *pModel;
};

struct LOOP_MATCHES{
	int index1, index2;
};

class PlanarUpdate{
public:
	int dimPose,broj_pozvanih,br_pog;
	int lok_rav_2,lok_rav,trazi_sve_id;
	bool bKinect;
	double point[2];
	Matrix3d Ri, R12, Rj;
	Vector3d ti, w12, tj, varqi, varqj;

	std::string config_path, temp_path;

	CRVLPSuLMVS VS;
	CRVL3DPose PoseSMC;
	CRVL3DSurface2* currentS;
	RVLPSULM_HYPOTHESIS *CHypothesis;
	vector<MODEL> local_models;

	PlanarUpdate(){
		printf("Initialization...");

		broj_pozvanih=0;
		dimPose = 4;
		VS.CreateParamList();

		// set concache_pathom ROS parameter
		if(!ros::param::get("/config_path", config_path)){
			config_path  = "/home/josip/catkin_ws/src/SLAM_git/cooperation_server/"; //boost::filesystem::current_path().string();
			ROS_INFO("Param /config_path not set, current dir %s will be used for configuration.", config_path.c_str());
		}
		config_path += "RVLPSuLMdemo.cfg";
		ROS_INFO("CONFIG read from FILE %s", config_path.c_str());
		VS.Init(const_cast<char*>(config_path.c_str()));

		// set path for debug dump from ROS parameter
		if(!ros::param::get("/temp_path", temp_path)){
			temp_path  = "/home/kruno/catkin_ws/TEST/rvl/";//boost::filesystem::current_path().string();
			ROS_INFO("Param /temp_path not set, current dir %s will be used as temp.", temp_path.c_str());
		}
		br_pog = 0;

		bKinect = false;

		// get the pointer to the depth image

		VS.m_CameraL.m_PanRange = 240;
		VS.m_CameraL.m_TiltRange = 125;
		VS.m_CameraL.m_PixPerDeg = 3;
		VS.m_CameraL.InitSpherical();
		VS.m_PSuLMBuilder.m_ROI.right = 2 * (VS.m_CameraL.m_wSpherical - 1) + 1;
		VS.m_PSuLMBuilder.m_ROI.bottom = 2 * (VS.m_CameraL.m_hSpherical - 1) + 1;
		local_models.clear();

		if(VS.m_Flags & RVLSYS_FLAGS_CREATE_GLOBAL_MESH)
			VS.CreateMeshFile("Mesh.obj");
		RVLGetFirstValidFileName(VS.m_ImageFileName, "00000-sl.bmp", 10000);
		PoseSMC.m_q=(double *) malloc(4*sizeof(double));
		PoseSMC.m_C=(double *) malloc(27*sizeof(double));
		printf(" Union OK\n");
	}

	void set_relative_pose_cov(M_POSE_PLANE& Pose, CRVL3DPose *Pose_rel){
		for (int i = 0; i < 3; i++){
			for(int j = 0; j < 3; j++){
				Pose_rel->m_Rot[i*3+j]=Pose.m_Rot(i,j);
				Pose_rel->m_C[18+i*3+j] = Pose.translation_C(i,j);
				Pose_rel->m_C[i*3+j] = Pose.angles_C(i,j);
				Pose_rel->m_C[9+i*3+j] = Pose.t_a_C(i,j);
			}
			Pose_rel->m_X[i]=Pose.m_X(i);

		}
		Pose_rel->UpdatePTRLL();
		Pose_rel->m_sa = sin(Pose.m_Alpha);
		Pose_rel->m_ca = cos(Pose.m_Alpha);
	}

	bool match_models(int id1, int id2, M_POSE_PLANE& Pose){
		ROS_INFO("\n ADD: Matching request received %d %d\n", id2,id1);
		bool wrong_match;
		double corr_error;
		CRVLPSuLM *pMPSuLMC, *psulm1, *psulm2;
		CRVL3DPose PoseSMInit;

		wrong_match = false;
		PoseSMInit.Reset();
		PoseSMInit.m_q=(double *) malloc(4*sizeof(double));
		PoseSMInit.m_C=(double *) malloc(27*sizeof(double));
		set_relative_pose_cov(Pose,&PoseSMInit);
		psulm1 = local_models[id1].pModel;
		psulm2 = local_models[id2].pModel;

		printf("angles_C\n");
		for(int i = 0; i < 9; i=i+3){
			printf("%f %f %f\n",PoseSMInit.m_C[i],PoseSMInit.m_C[i+1],PoseSMInit.m_C[i+2]);
		}
		printf("t_a_C\n");
		for(int i = 0; i < 9; i=i+3){
			printf("%f %f %f\n",PoseSMInit.m_C[9+i],PoseSMInit.m_C[9+i+1],PoseSMInit.m_C[9+i+2]);
		}
		printf("translation_C\n");
		for(int i = 0; i < 9; i=i+3){
			printf("%f %f %f\n",PoseSMInit.m_C[18+i],PoseSMInit.m_C[18+i+1],PoseSMInit.m_C[18+i+2]);
		}

		printf("	ADD: Relative position SLAM: %f %f %f\n",PoseSMInit.m_X[0],PoseSMInit.m_X[1],PoseSMInit.m_X[2]);
		printf("	ADD: Relative rotation SLAM: %f %f %f\n",PoseSMInit.m_Alpha*180/PI,PoseSMInit.m_Beta*180/PI,PoseSMInit.m_Theta*180/PI);

		VS.m_PSuLMBuilder.Localization(psulm2, &PoseSMInit,psulm1);

		RVLSetFileNumber(VS.m_ImageFileName, "00000-PC.pcd", id2);

		CHypothesis = VS.m_PSuLMBuilder.m_HypothesisArray[0];

		printf("	ADD: Relative position RVL: %f %f %f\n",CHypothesis->PoseSM.m_X[0],CHypothesis->PoseSM.m_X[1],CHypothesis->PoseSM.m_X[2]);
		printf("	ADD: Relative rotation RVL: %f %f %f\n",CHypothesis->PoseSM.m_Alpha*180/PI,CHypothesis->PoseSM.m_Beta*180/PI,CHypothesis->PoseSM.m_Theta*180/PI);

		double odom_p[3], rvl_p[3], d_p[3],rvl_a,odom_a,d_a;
		/*if(abs(id1-id2)==1){
			for(int i = 0; i < 3; i++){
				if((i==0)||(i==3)){
					odom_p[i] = PoseSMInit.m_X[i];
					rvl_p[i] = CHypothesis->PoseSM.m_X[i];
					d_p[i] = fabs(odom_p[i]-rvl_p[i]);
					if(d_p[i]>200){
						if((d_p[i]>400)){
							wrong_match=true;
						}
					}
				}
			}
			rvl_a = CHypothesis->PoseSM.m_Alpha;
			odom_a = PoseSMInit.m_Alpha;
			d_a = rvl_a-odom_a;
			if(d_a > PI){
				d_a = d_a - 2*PI;
			}else if(d_a<-PI){
				d_a = d_a+2*PI;
			}

			if(fabs(d_a)>0.005){
				if((fabs(d_a)>0.012) || ((odom_a>0)&&(rvl_a<0)) || ((odom_a<0)&&(rvl_a>0))){
					wrong_match=true;
					ROS_ERROR("rvl error rotation: %f %f %f\n",rvl_a,odom_a,fabs(d_a));
				}
			}
			printf("rotations: %f %f %f\n",rvl_a*RAD2DEG,odom_a*RAD2DEG,d_a*RAD2DEG);
		}*/

		if(wrong_match){
			ROS_ERROR("	ADD: RVL ERROR odom (%f %f %f) rvl (%f %f %f)\n",odom_p[0],odom_p[1],odom_p[2],rvl_p[0],rvl_p[1],rvl_p[2]);
		}else{
			corr_error = sqrt(pow(PoseSMInit.m_X[0]-CHypothesis->PoseSM.m_X[0],2)+pow(PoseSMInit.m_X[2]-CHypothesis->PoseSM.m_X[2],2));
			if(corr_error<50000){
				printf("	ADD: Attempting to update SLAM\n");
				for(int bri = 0; bri<3; bri++){
					for(int brj=0;brj<3;brj++){
						Pose.m_Rot(bri,brj)=CHypothesis->PoseSM.m_Rot[bri*3+brj];
						Pose.translation_C(bri,brj)=CHypothesis->PoseSM.m_C[18+bri*3+brj]/1000.0/1000.0;
						Pose.angles_C(bri,brj)=CHypothesis->PoseSM.m_C[bri*3+brj];
						Pose.t_a_C(bri,brj)=CHypothesis->PoseSM.m_C[9+bri*3+brj]/1000.0;
					}
					Pose.m_X(bri) = CHypothesis->PoseSM.m_X[bri]/1000.0;
				}
				Pose.m_Alpha = CHypothesis->PoseSM.m_Alpha;
				Pose.m_Beta = CHypothesis->PoseSM.m_Beta;
				Pose.m_Theta = CHypothesis->PoseSM.m_Theta;
				Pose.m_sa = sin(Pose.m_Alpha);
				Pose.m_ca = cos(Pose.m_Alpha);
			}else{
				wrong_match=true;
				ROS_WARN("Too big corr_error, aborting update %f",corr_error);
			}
		}
		printf("MATCH: matching between models completed %f \n",corr_error);
		return(!wrong_match);
	}

	bool add_model(sensor_msgs::PointCloud2 &pcd, int pcd_id, int agent_id){

		ros::Time t_add_start = ros::Time::now();
		//ROS_INFO("	ADD MODEL called v1.1\n");

		int num_models;
		char buffer[200];
		MODEL tmp_model;
		FILE *pFile;

#ifdef VELODYNE32
		std::vector<PCLPointFieldMY> my_fields;
		std::vector<uint8_t> my_data;
		my_fields.resize(pcd.fields.size());
		for(int i = 0; i < pcd.fields.size(); i++) {
			my_fields[i].name = pcd.fields[i].name;
			my_fields[i].offset = pcd.fields[i].offset;
			my_fields[i].datatype = pcd.fields[i].datatype;
			my_fields[i].count = pcd.fields[i].count;
		}
		my_data = pcd.data;

		std::vector<FieldMapping> field_map;
		FieldMapperMY<pcl::PointXYZ> mapper2 (my_fields,field_map);
		for_each_type_moj< typename pcl::traits::fieldList<pcl::PointXYZ>::type > (mapper2);

		// Coalesce adjacent fields into single memcpy's where possible
		if (field_map.size() > 1){
			std::sort(field_map.begin(), field_map.end(), fieldOrderingMY);
			MsgFieldMapMY::iterator i = field_map.begin(), j = i + 1;
			while (j != field_map.end())
			{
				if (j->serialized_offset - i->serialized_offset == j->struct_offset - i->struct_offset){
					i->size += (j->struct_offset + j->size) - (i->struct_offset + i->size);
					j = field_map.erase(j);
				}else{
					++i;
					++j;
				}
			}
		}
#endif
		// Copy point data
		uint32_t num_points = pcd.width * pcd.height;
		std::vector<PointXYZ_MY, Eigen::aligned_allocator<PointXYZ_MY> > pcd_points;
		pcd_points.resize (num_points);
		uint8_t* cloud_data = reinterpret_cast<uint8_t*>(&pcd_points[0]);

#ifdef VELODYNE32
		for (uint32_t row = 0; row < pcd.height; ++row)
		{
			const uint8_t* row_data = &my_data[row * pcd.row_step];
			for (uint32_t col = 0; col < pcd.width; ++col)
			{
				const uint8_t* msg_data = row_data + col * pcd.point_step;
				BOOST_FOREACH (const FieldMapping& mapping, field_map){
					memcpy (cloud_data + mapping.struct_offset, msg_data + mapping.serialized_offset, mapping.size);
				}
				cloud_data += sizeof (PointXYZ_MY);
			}
		}
#endif

#ifdef VELODYNE64
		const uint8_t* msg_data = &pcd.data[0];
		memcpy (cloud_data, msg_data, pcd.data.size ());
#endif

		double *PC_new = new double[3 * VS.m_PSuLMBuilder.m_pPSD->m_Width * VS.m_PSuLMBuilder.m_pPSD->m_Height];
		double **pX = &PC_new;
		double *X = *pX;
		if(X)
			delete[] X;
		X = new double[3 * num_points];
		*pX = X;
		double *X_ = X;
#ifdef VELODYNE64
#ifdef BRLLASER
		for(int i = 0; i < num_points; i++,X_ += 3){
			X_[0] = -1000.0 * pcd_points[i].y;
			X_[1] = -1000.0 * pcd_points[i].z;
			X_[2] = 1000.0 * pcd_points[i].x;
		}
#else
		/*for(int i = 0; i < num_points; i++,X_ += 3){
			X_[0] = 1000.0 * pcd_points[i].x;
			X_[1] = -1000.0 * pcd_points[i].z;
			X_[2] = 1000.0 * pcd_points[i].y;
		}*/

		for(int i = 0; i < num_points; i++,X_ += 3){
			X_[0] = -1000.0 * pcd_points[i].y;
			X_[1] = -1000.0 * pcd_points[i].z;
			X_[2] = 1000.0 * pcd_points[i].x;
		}
#endif
#endif
#ifdef VELODYNE32
		for(int i = 0; i < num_points; i++,X_ += 3){
			X_[0] = -1000.0 * pcd_points[i].y;
			X_[1] = -1000.0 * pcd_points[i].z;
			X_[2] = 1000.0 * pcd_points[i].x;
		}
#endif

		const clock_t begin_t = clock();
		VS.m_PSuLMBuilder.m_pPSD->GetOrgPC(PC_new, num_points);

		delete[] PC_new;

		RVLSetFileNumber(VS.m_ImageFileName, "00000-PC.pcd", pcd_id);
		VS.m_PoseA0.Reset();
		VS.Update(bKinect ? 0x00000000 : RVLPSULMBUILDER_CREATEMODEL_IMAGE_FROM_FILE);
		double elapsed_secs = double(clock() - begin_t) / CLOCKS_PER_SEC;
#ifdef DO_LOG
		snprintf(buffer, sizeof(buffer), "times/segment.txt");
		pFile = fopen((temp_path + buffer).c_str(),"a");
		fprintf(pFile,"%lf\n",elapsed_secs);
		fclose(pFile);
#endif
		VS.m_PSuLMBuilder.m_PSuLMList.Start();
		CRVLPSuLM *pMPSuLMC;
		num_models=0;
		while(VS.m_PSuLMBuilder.m_PSuLMList.m_pNext){
			pMPSuLMC = (CRVLPSuLM *)(VS.m_PSuLMBuilder.m_PSuLMList.GetNext());
			num_models++;
		}
		printf("	ADD: Total models after addition: %d\n",num_models);

		snprintf(buffer, sizeof(buffer), "server_models/agent_%d/%d_model.txt",agent_id,br_pog);
		pFile = fopen((temp_path + buffer).c_str(),"wb");
		pMPSuLMC->Save(pFile);
		fclose(pFile);
		br_pog++;

		tmp_model.pModel = pMPSuLMC;
		local_models.push_back(tmp_model);

		VS.m_pPSuLM->m_Index = 0xffffffff;
		VS.m_Mem.Clear();

		printf("ADD: Model add OK %d\n",pMPSuLMC->m_n3DSurfacesTotal);
		printf("ADD: Duration %f\n",(ros::Time::now()-t_add_start).toSec()*1000);
		return true;
	}

	~PlanarUpdate() {
		printf("Shutting Down\n");
	}


};
