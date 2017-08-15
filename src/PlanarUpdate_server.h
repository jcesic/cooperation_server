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

#define USE_VTK

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

struct match_data{
	double angle,distance;
	int plane_id1,plane_id2,model_id1,model_id2;
};

struct PLANE{
	int id;
	int model_id;
	int global_plane_id;
};

struct PLANE_POINT{
	double x,y;
};

struct BOUNDARY_PLANE_PARAMS{
	PLANE_POINT max_x,min_x,max_y,min_y;
	double area;
};

struct f_mec{
	int pair_group_id,final_glob_id;
	vector<PLANE> parovi;
	Paths polygons,final_polygons;
	int ref_plane_id,ref_model_id,max_area;
	Matrix3d refRot,globRot;
	Vector3d refTran,globTran;
	vector<BOUNDARY_PLANE_PARAMS> params1,params2;
};

struct TRAN{
	Matrix3d Rotation;
	Vector3d Normal, Centar;
};


struct CONTOUR_S{
	bool bHole;
	Path poly_contour;
	vector<Vector3d> P3D;
};

struct LOCAL_PLANE{
	int global_id;
	PLANE ref_plane;
	bool inserted;
	bool wall,floor,roof;
	bool big;
	double area,m_d;
	Vector3d m_varq;
	bool deleted;
	vector<BOUNDARY_PLANE_PARAMS> params;
	TRAN old_position;
	M_POSE_PLANE m_Pose;
	bool paired;
	vector<CONTOUR_S> contours;
};

struct MODEL{
	bool changed,is_new;
	bool map_changed;
	CRVLPSuLM *pModel;
	vector<int> global_planes;
	int SurfacesTotal;
	vector<LOCAL_PLANE> local_planes;
	int agent_id;
	int model_agent_id;
};

struct INTERSECTION_MATCH{
	double int_x, int_y;
};

struct correction_pose{
	int index1, index2;
	double Alpha,Beta,Theta,x,y,z;
	//MeasurementVector h_k;
	double covariance[27];
};

struct LOOP_MATCHES{
	int index1, index2;
};

struct PLANE_PARAMS {
	Matrix3d R;
	Vector3d t;
	Vector3d n;
	double d;
	Matrix3d varq;
};

class PlanarUpdate{
public:
	int broj_pozvanih,total_models;
	int lok_rav_2,lok_rav,trazi_sve_id,agent_id;
	bool bKinect;
	double point[2];
	double N_THRESHOLD_local;
	double mD_THRESHOLD_local;
	Matrix3d Ri, R12, Rj;
	Vector3d ti, w12, tj, varqi, varqj;

	std::string config_path, temp_path;

	CRVLPSuLMVS VS;
	CRVL3DPose PoseSMC;
	CRVL3DSurface2* currentS;
	RVLPSULM_HYPOTHESIS *CHypothesis;
	Vector3d T_laser_odom;
	vector<MODEL> local_models;
	vector<int> last_model_id,total_models_agents;
	vector<vector<int > > all_model_ids;

	PlanarUpdate(){
		printf("Initialization...");

		broj_pozvanih=0;
		VS.CreateParamList();

		// set concache_pathom ROS parameter
		if(!ros::param::get("/config_path", config_path)){
			config_path  = "/home/kruno/map_ws/src/SLAM_git/cooperation_server/"; //boost::filesystem::current_path().string();
			ROS_INFO("Param /config_path not set, current dir %s will be used for configuration.", config_path.c_str());
		}
		config_path += "RVLPSuLMdemo.cfg";
		ROS_INFO("CONFIG read from FILE %s", config_path.c_str());
		VS.Init(const_cast<char*>(config_path.c_str()));

		// set path for debug dump from ROS parameter
		if(!ros::param::get("/temp_path", temp_path)){
			temp_path  = "/media/kruno/DATA/TEST/SLAM/";//boost::filesystem::current_path().string();
			ROS_INFO("Param /temp_path not set, current dir %s will be used as temp.", temp_path.c_str());
		}

		bKinect = false;

		// get the pointer to the depth image

		VS.m_CameraL.m_PanRange = 240;
		VS.m_CameraL.m_TiltRange = 125;
		VS.m_CameraL.m_PixPerDeg = 3;
		VS.m_CameraL.InitSpherical();
		VS.m_PSuLMBuilder.m_ROI.right = 2 * (VS.m_CameraL.m_wSpherical - 1) + 1;
		VS.m_PSuLMBuilder.m_ROI.bottom = 2 * (VS.m_CameraL.m_hSpherical - 1) + 1;

		local_models.clear();

#ifdef VELODYNE32
		N_THRESHOLD_local = 2;//4,8;
		mD_THRESHOLD_local = 400;//200;
#endif
#ifdef VELODYNE64
		N_THRESHOLD_local = 3;//4,8;
		mD_THRESHOLD_local = 800;//1000;//200;
#endif

		if(VS.m_Flags & RVLSYS_FLAGS_CREATE_GLOBAL_MESH)
			VS.CreateMeshFile("Mesh.obj");
		RVLGetFirstValidFileName(VS.m_ImageFileName, "00000-sl.bmp", 10000);
		PoseSMC.m_q=(double *) malloc(4*sizeof(double));
		PoseSMC.m_C=(double *) malloc(27*sizeof(double));
		printf(" Union OK\n");
	}

	bool check_interval_intersection(double x1_max, double x1_min, double x2_max, double x2_min, double &intersection_score){
		double maxv=x2_max,minv=x2_min, dist1 = fabs(x1_max-x1_min), dist2=fabs(x2_max-x2_min);
		double dist = dist2;
		if(dist>dist1)
			dist = dist1;

		if((x2_max>=x1_min) && (x2_min<=x1_max)){
			if(x2_max>x1_max)
				maxv = x1_max;
			if(x1_min>x2_min)
				minv = x1_min;
			intersection_score = fabs(maxv-minv)/1.0/dist;
			return true;
		}
		return false;
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

	double match_models(int id1, int id2, M_POSE_PLANE& Pose, double &prob_match){
		ROS_INFO("\n ADD: Matching request received %d %d\n", id2,id1);
		bool wrong_match;
		double corr_error;
		CRVLPSuLM *psulm1, *psulm2;
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
		prob_match = CHypothesis->Probability;

		printf("	ADD: Relative position RVL: %f %f %f\n",CHypothesis->PoseSM.m_X[0],CHypothesis->PoseSM.m_X[1],CHypothesis->PoseSM.m_X[2]);
		printf("	ADD: Relative rotation RVL: %f %f %f\n",CHypothesis->PoseSM.m_Alpha*180/PI,CHypothesis->PoseSM.m_Beta*180/PI,CHypothesis->PoseSM.m_Theta*180/PI);

		corr_error = sqrt(pow(PoseSMInit.m_X[0]-CHypothesis->PoseSM.m_X[0],2)+pow(PoseSMInit.m_X[2]-CHypothesis->PoseSM.m_X[2],2));
		if(corr_error<50000){
			for(int bri = 0; bri<3; bri++){
				for(int brj=0;brj<3;brj++){
					Pose.m_Rot(bri,brj)=CHypothesis->PoseSM.m_Rot[bri*3+brj];
					Pose.translation_C(bri,brj)=CHypothesis->PoseSM.m_C[18+bri*3+brj]/1000.0/1000.0;
					Pose.angles_C(bri,brj)=CHypothesis->PoseSM.m_C[bri*3+brj];
					Pose.t_a_C(bri,brj)=CHypothesis->PoseSM.m_C[9+bri*3+brj]/1000.0;
				}
				Pose.m_X(bri) = CHypothesis->PoseSM.m_X[bri]/1000.0;
			}
			cout << "mRot" << endl << Pose.m_Rot << endl;
			cout << "mX" << endl << Pose.m_X << endl;

			Pose.m_Alpha = CHypothesis->PoseSM.m_Alpha;
			Pose.m_Beta = CHypothesis->PoseSM.m_Beta;
			Pose.m_Theta = CHypothesis->PoseSM.m_Theta;
			Pose.m_sa = sin(Pose.m_Alpha);
			Pose.m_ca = cos(Pose.m_Alpha);
		}else{
			wrong_match=true;
			ROS_WARN("Too big corr_error, aborting update %f",corr_error);
		}

		printf("MATCH: matching between models completed %f \n",corr_error);
		return(!wrong_match);
	}

	~PlanarUpdate() {
		printf("Shutting Down\n");
	}


};
