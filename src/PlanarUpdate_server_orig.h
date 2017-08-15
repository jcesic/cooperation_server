#include "ProcessDefinitions.h"

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
	int broj_pozvanih,total_models,br_pog,num_changed_models;
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

	PlanarUpdate(){
		printf("Initialization...");

		broj_pozvanih=0;
		num_changed_models = 0;
		VS.CreateParamList();

		// set concache_pathom ROS parameter
		if(!ros::param::get("/config_path", config_path)){
			config_path  = "/home/josip/map_ws/src/SLAM_git/cooperation_server/"; //boost::filesystem::current_path().string();
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
		N_THRESHOLD_local = 4;//8;
		mD_THRESHOLD_local = 400;//200;
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

	bool add_model(sensor_msgs::PointCloud2 &pcd, int pcd_id){

		ros::Time t_add_start = ros::Time::now();
		ROS_INFO("	ADD MODEL called\n");

		int num_models;
		char buffer[200];
		vector<LOCAL_PLANE> currPlanes;
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

		double *PC_new = new double[3 * VS.m_PSuLMBuilder.m_pPSD->m_Width * VS.m_PSuLMBuilder.m_pPSD->m_Height];
		double **pX = &PC_new;
		double *X = *pX;
		if(X)
			delete[] X;
		X = new double[3 * num_points];
		*pX = X;
		double *X_ = X;

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

		for(int k=0; k<pMPSuLMC->m_n3DSurfacesTotal;k++){
			LOCAL_PLANE currPlane;
			currPlane.global_id = -1;
			currPlane.paired = false;

			currentS = pMPSuLMC->m_3DSurfaceArray[k];
			for(int i = 0; i < 3; i++){
				for(int j = 0; j < 3; j++){
					currPlane.old_position.Rotation(i,j)=currentS->m_Pose.m_Rot[i*3+j];
				}
				currPlane.old_position.Centar(i) = currentS->m_Pose.m_X[i];
				currPlane.old_position.Normal(i) = currentS->m_N[i];
			}

			currPlane.paired = false;
			tmp_model.local_planes.push_back(currPlane);
		}

		tmp_model.changed = false;
		tmp_model.map_changed = false;
		tmp_model.is_new = true;
		tmp_model.pModel = pMPSuLMC;
		tmp_model.SurfacesTotal = pMPSuLMC->m_n3DSurfacesTotal;
		local_models.push_back(tmp_model);

		VS.m_pPSuLM->m_Index = 0xffffffff;
		VS.m_Mem.Clear();

		printf("ADD: Model add OK %d\n",tmp_model.SurfacesTotal);
		printf("ADD: Duration %f\n",(ros::Time::now()-t_add_start).toSec()*1000);
		return true;
	}

	~PlanarUpdate() {
		printf("Shutting Down\n");
	}


};
