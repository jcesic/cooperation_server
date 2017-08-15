#include "PlanarUpdate_server.h"

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/adapted/c_array.hpp>
#include <boost/geometry/geometries/adapted/boost_tuple.hpp>
#include <boost/geometry/geometries/ring.hpp>
using namespace boost::geometry;
BOOST_GEOMETRY_REGISTER_C_ARRAY_CS(cs::cartesian)
BOOST_GEOMETRY_REGISTER_BOOST_TUPLE_CS(cs::cartesian)
typedef boost::geometry::model::d2::point_xy<double> boost_points;
typedef model::polygon<boost_points, false, false > polygon;
typedef boost::geometry::model::ring< boost_points, false, false > ring;
typedef boost::geometry::model::linestring<boost_points> lines;

struct local_model_update{
	int index;
	bool check_area;
};

struct MAP_PARAMETERS{
	double angle;
	double maxx,minx,maxy,miny,maxz,minz;
	bool changed;
	bool roof;
	bool floor;
	bool wall;
};

struct GLOBAL_PLANE{
	int ID;

	PLANE ref_plane;
	vector<PLANE> local_planes;
	bool active;
	bool global;
	bool added;
	int global_id;
	int num_called;
	bool changed;
	bool reformat;
	double area;
	VectorXd ref_local_planes;
	LOCAL_PLANE m_plane,t_plane;
	MAP_PARAMETERS map_params;
	void copy_to_mplane(LOCAL_PLANE &local_plane){
		for(int k = 0; k < 3; k++){
			for(int l = 0; l < 3; l++){
				m_plane.m_Pose.m_Rot(k,l) = local_plane.m_Pose.m_Rot(k,l);
			}
			m_plane.m_Pose.m_X(k) = local_plane.m_Pose.m_X(k);
			m_plane.m_Pose.m_N(k) = local_plane.m_Pose.m_N(k);
			m_plane.m_varq(k) = local_plane.m_varq(k);
		}
	}
};

class MapTrajectory{
public:
	SE3Mappings SEMap;
	void PoseBetweenStates(int i, int j, int agent_i, int agent_j, SE3Mat& h_k) {
		StateVector x1g = trajectories[agent_i][i];
		StateVector x2g = trajectories[agent_j][j];
		SE3Mat x1G = SEMap.exp(x1g);
		SE3Mat x2G = SEMap.exp(x2g);
		h_k = x1G.inverse()*x2G;
	}
	StateVector get_group(StateVector& x){
		return SEMap.logV(SEMap.exp(x));
	}
	vector<vector<StateVector > > trajectories, old_trajectories;

};

class PlanarMapping: public PlanarUpdate{

public:
	double M_THRESHOLD;
	double H_THRESHOLD;
	double AREA_THRESHOLD;
	double CENTER_THRESHOLD;
	double m_epsilon1;
	double POSE_THRESHOLD;
	double ORIENTATION_THRESHOLD;
	double N_THRESHOLD_global;
	double mD_THRESHOLD_global;
	double MIN_TEST_SCORE;
	double sigma_multiply;
	double RES,total_time_union,RES_AREA;
	double wall_angle_THRESHOLD;
	double roof_floor_angle_THRESHOLD;
	double min_roof_height;
	int REFINE_AND_REFORMAT_EVERY;
	int SEND_MAP_EVERY;
	bool SEND_NOW;
	bool new_map_;
	int models_in_trajectory;
	int total_planes_local;
	int do_map_update;

	MapTrajectory slam_trajectory;
	MatrixXi svi_mec;
	VectorXi redovi_dodani;
	VectorXi stupci_dodani;

	vector<f_mec> s_mec;
	vector<PLANE> plane_pairs;
	vector<vector<PLANE> > all_plane_pairs;
	vector<vector<PLANE> > all_reformated_pairs;
	vector<LOOP_MATCHES> loop_matches;
	vector<int> id_red;
	vector<GLOBAL_PLANE> global_planes;
	vector<double> paired_distances;

	visualization_msgs::Marker surface_marker,surface_marker2;

	Eigen::SimplicialCholesky<SparseMat, Eigen::Upper> solver;
	SparseMat L_SLAM;

	vector<int> roof_surfaces,floor_surfaces, wall_surfaces;

	PlanarMapping(){
		//PlanarMapping() {
		printf("Initialization...");
#ifdef VELODYNE64
#ifdef BRLLASER
		AREA_THRESHOLD = 2000000;//4000000;//4000000;
		CENTER_THRESHOLD = 15000;//50000; //20000;
		mD_THRESHOLD_global = 400;//1500; //200;
		N_THRESHOLD_global = 5;//5;
#else
		AREA_THRESHOLD = 500000;//4000000;//4000000;
		CENTER_THRESHOLD = 35000;//50000; //20000;
		mD_THRESHOLD_global = 800;//1000;//1500; //200;
		N_THRESHOLD_global = 3;//8;//5;
#endif
#endif
#ifdef VELODYNE32
		M_THRESHOLD=50000;
		H_THRESHOLD=4000;
		AREA_THRESHOLD = 1000000;//500000;
		CENTER_THRESHOLD = 15000;
		mD_THRESHOLD_global = 620;//200;
		N_THRESHOLD_global = 2;//4,10;
		MIN_TEST_SCORE = 20;
		sigma_multiply = 0.9;
#endif
		total_planes_local = 0;
		models_in_trajectory =0;
		m_epsilon1 = 11.34;
		POSE_THRESHOLD = 100;//150.0;
		ORIENTATION_THRESHOLD = 3*PI/180.0;//5
		total_time_union = 0;
		RES = 1.0;
		RES_AREA = RES*RES;
		wall_angle_THRESHOLD=20*PI/180.0;
		roof_floor_angle_THRESHOLD=50*PI/180.0;
		min_roof_height = 100;
		REFINE_AND_REFORMAT_EVERY = 1;
		SEND_MAP_EVERY = 80;
		SEND_NOW = false;
		new_map_=false;
		surface_marker.header.frame_id = "/map_0";
		surface_marker.ns = "basic_shapes";
		surface_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
		surface_marker.action = visualization_msgs::Marker::ADD;
		surface_marker.pose.position.x = 0;	surface_marker.pose.position.y = 0;	surface_marker.pose.position.z = 0;
		surface_marker.pose.orientation.x = 0.0; surface_marker.pose.orientation.y = 0.0; surface_marker.pose.orientation.z = 0.0; surface_marker.pose.orientation.w = 1.0;
		surface_marker.scale.x = 1.0; surface_marker.scale.y = 1.0; surface_marker.scale.z = 1.0;
		surface_marker2 = surface_marker;
		total_models = 0;
		ros::param::set("/do_map_update", 0);

		printf("Map OK\n");
	}

	double get_area(Path &poly){ //return the area of polygon
		return fabs(Area(poly))/RES_AREA;
	}

	double calculate_polygon_area(f_mec* cMec){
		double area = 0.0;
		for(int c=0; c<cMec->final_polygons.size();c++){
			area = area + get_area(cMec->final_polygons[c]);
		}
		return area;
	}

	void detect_global_planes(int &num_global, int &num_local){ //returns the number of global planes in global model
		int num_global_planes=0;
		int num_local_planes=0;
		for(int i = 0; i<global_planes.size(); i++){
			if(global_planes[i].active){
				num_global_planes++;
				num_local_planes = num_local_planes + global_planes[i].local_planes.size();
			}
		}
		num_global = num_global_planes;
		num_local = num_local_planes;
	}

	void delete_global_plane(int id){
		global_planes[id].active = false;
		global_planes[id].global = false;
		global_planes[id].local_planes.clear();
	}

	int add_global_plane(f_mec* cMec){
		GLOBAL_PLANE plane;
		int tmp_ref_plane;
		plane.active = true;
		plane.added = false;
		plane.changed = true;
		plane.global = false;
		plane.map_params.floor = false;
		plane.map_params.roof = false;
		plane.map_params.wall = false;

		plane.global_id = -1;
		plane.ID = global_planes.size();
		plane.num_called = 2;
		PLANE tmp_plane;
		plane.ref_local_planes.resize(total_models);
		plane.ref_local_planes.setConstant(-1);
		for(int j=0; j<cMec->parovi.size();j++){
			tmp_plane.id = cMec->parovi[j].id;
			tmp_plane.model_id = cMec->parovi[j].model_id;
			local_models[cMec->parovi[j].model_id].local_planes[cMec->parovi[j].id].global_id = plane.ID;
			plane.local_planes.push_back(tmp_plane);
			tmp_ref_plane = plane.ref_local_planes[tmp_plane.model_id];
			if(tmp_ref_plane==-1){
				plane.ref_local_planes[tmp_plane.model_id] = tmp_plane.id;
				tmp_ref_plane = plane.ref_local_planes[tmp_plane.model_id];
			}
			local_models[tmp_plane.model_id].local_planes[tmp_plane.id].ref_plane.id=tmp_ref_plane;
		}

		plane.ref_plane.model_id = cMec->ref_model_id;
		plane.ref_plane.id = cMec->ref_plane_id;
		plane.ref_plane.global_plane_id = plane.ID;
		plane.copy_to_mplane(local_models[cMec->ref_model_id].local_planes[cMec->ref_plane_id]);
		global_planes.push_back(plane);
		return plane.ID;
	}

	void merge_global_planes(int id1, int id2){
		int tmp_model_id, tmp_ref_plane;
		global_planes[id1].num_called = global_planes[id1].num_called + global_planes[id2].num_called;
		global_planes[id1].changed = true;
		global_planes[id1].active = true;
		global_planes[id2].global_id = id1;

		for(int i = 0; i<global_planes[id2].local_planes.size(); i++){
			global_planes[id1].local_planes.push_back(global_planes[id2].local_planes[i]);
		}
		global_planes[id1].ref_local_planes.resize(total_models);
		global_planes[id1].ref_local_planes.setConstant(-1);
		for(int k = 0; k<global_planes[id1].local_planes.size(); k++){
			tmp_model_id = global_planes[id1].local_planes[k].model_id;
			tmp_ref_plane = global_planes[id1].ref_local_planes[tmp_model_id];
			if(tmp_ref_plane==-1){
				global_planes[id1].ref_local_planes[tmp_model_id] = global_planes[id1].local_planes[k].id;
				tmp_ref_plane = global_planes[id1].ref_local_planes[tmp_model_id];
			}
			local_models[tmp_model_id].local_planes[global_planes[id1].local_planes[k].id].ref_plane.id=tmp_ref_plane;
			local_models[tmp_model_id].local_planes[global_planes[id1].local_planes[k].id].global_id = id1;
		}
		delete_global_plane(id2);
	}

	void simple_update_global_planes(int id1){
		int tmp_model_id, tmp_ref_plane;
		global_planes[id1].num_called = global_planes[id1].num_called + 1;
		global_planes[id1].changed = true;
		global_planes[id1].active = true;

		global_planes[id1].ref_local_planes.resize(total_models);
		global_planes[id1].ref_local_planes.setConstant(-1);
		for(int k = 0; k<global_planes[id1].local_planes.size(); k++){
			tmp_model_id = global_planes[id1].local_planes[k].model_id;
			tmp_ref_plane = global_planes[id1].ref_local_planes[tmp_model_id];
			if(tmp_ref_plane==-1){
				global_planes[id1].ref_local_planes[tmp_model_id] = global_planes[id1].local_planes[k].id;
				tmp_ref_plane = global_planes[id1].ref_local_planes[tmp_model_id];
			}
			local_models[tmp_model_id].local_planes[global_planes[id1].local_planes[k].id].ref_plane.id=tmp_ref_plane;
			local_models[tmp_model_id].local_planes[global_planes[id1].local_planes[k].id].global_id = id1;
		}
	}

	bool add_polygon_from_plane(bool transform, f_mec* cMec, Matrix3d R_final, Vector3d T_final, LOCAL_PLANE &tmp_plane){
		int num_holes;
		bool is_hole,added;
		Path poly;
		Vector3d Tp_tf;
		for (int brC = 0; brC < tmp_plane.contours.size(); brC++){
			if (tmp_plane.contours[brC].bHole){
				continue;
			}
			if(0){
				cMec->polygons.push_back(tmp_plane.contours[brC].poly_contour);
			}else{
				poly.clear();
				for(int brP = 0; brP < tmp_plane.contours[brC].poly_contour.size(); brP++){
					Tp_tf = R_final*P2D23D(tmp_plane.contours[brC].poly_contour[brP]) +T_final;
					if(!is_hole){
						poly.push_back(IntPoint((int)(Tp_tf(0)),(int)(Tp_tf(1))));
					}else{
						num_holes++;
					}
				}
				cMec->polygons.push_back(poly);
			}
		}
		return true;
	}

	void polygon_union(f_mec* cMec, int id, LOCAL_PLANE &tmp_plane){
		Clipper c;
		Paths solution;
		solution.clear();
		c.AddPaths(cMec->polygons, ptSubject, true);
		const clock_t begin_t = clock();
		c.Execute(ctUnion, solution, pftNonZero, pftNonZero);
		double elapsed_secs = double(clock() - begin_t) / CLOCKS_PER_SEC;
		total_time_union = total_time_union + elapsed_secs;
		cMec->final_polygons.clear();
		tmp_plane.contours.clear();
		for(int i = 0; i < solution.size(); i++){
			if(get_area(solution[i])>AREA_THRESHOLD){
				CONTOUR_S tmp_contour;
				tmp_contour.bHole=false;
				tmp_contour.poly_contour = solution[i];
				tmp_plane.contours.push_back(tmp_contour);
				cMec->final_polygons.push_back(solution[i]);
			}
		}

	}

	void setRefglob(f_mec *cMec){
		int plane_id;
		cMec->max_area = -1;
		cMec->ref_plane_id = all_plane_pairs[cMec->pair_group_id][0].id;
		for (int i=0;i<all_plane_pairs[cMec->pair_group_id].size();i++){
			plane_id = all_plane_pairs[cMec->pair_group_id][i].id;
			if((global_planes[plane_id].area>cMec->max_area) || (cMec->ref_plane_id > plane_id)){
				cMec->ref_plane_id = plane_id;
				cMec->max_area = global_planes[plane_id].area;
				for(int k = 0; k < 3; k++){
					for(int l = 0; l < 3; l++){
						cMec->refRot(k,l) = global_planes[plane_id].m_plane.m_Pose.m_Rot(k,l);
					}
					cMec->refTran(k) = global_planes[plane_id].m_plane.m_Pose.m_X(k);
				}
			}
		}
	}

	void setRefPlane(f_mec *cMec, double area, int model_id, int plane_id){
		if(model_id<cMec->ref_model_id || area>cMec->max_area){
			cMec->ref_plane_id = plane_id;
			cMec->max_area = area;
			cMec->ref_model_id = model_id;
		}
	}

	void dodaj_par(int i, int j, int id1, int id2){
		PLANE tmp_plane;
		tmp_plane.id = i;
		tmp_plane.model_id = j;
		tmp_plane.global_plane_id = -1;
		if(j==id2 && stupci_dodani[i]==0){
			plane_pairs.push_back(tmp_plane);
			stupci_dodani[i]=1;
		}
		if(j==id1 && redovi_dodani[i]==0){
			plane_pairs.push_back(tmp_plane);
			redovi_dodani[i]=1;
		}
	}

	void trazi_sve(int i, int j, int id1, int id2){
		trazi_sve_id++;
		for(int n= 0; n<svi_mec.cols();n++){
			if(svi_mec(i,n)==1){
				svi_mec(i,n)=0;
				dodaj_par(n,id2,id1,id2);
				trazi_sve(i,n,id1,id2);
			}
		}
		for(int m= 0; m<svi_mec.rows();m++){
			if(svi_mec(m,j)==1){
				svi_mec(m,j)=0;
				dodaj_par(m,id1,id1,id2);
				trazi_sve(m,j,id1,id2);
			}
		}
	}

	void nadi_parove(int id1, int id2){
		for(int i = 0; i < svi_mec.rows(); i++){
			for(int j = 0; j<svi_mec.cols(); j++){
				if(svi_mec(i,j)==1){
					plane_pairs.clear();
					svi_mec(i,j)=0;
					dodaj_par(j,id2,id1,id2);
					dodaj_par(i,id1,id1,id2);
					trazi_sve_id=0;
					trazi_sve(i,j,id1,id2);
					all_plane_pairs.push_back(plane_pairs);
				}
			}
		}
	}

	void transform_to_glob(LOCAL_PLANE &m_plane, int id1, int model_id, LOCAL_PLANE &t_plane){ //takes mplane and transforms it to global map (tplane)
		Vector3d trans_col;
		M_POSE_PLANE global_pose;

		set_glob_rot_tranM(id1,model_id,global_pose);
		t_plane.m_Pose.m_N = global_pose.m_Rot*m_plane.m_Pose.m_N;
		t_plane.m_Pose.m_Rot=global_pose.m_Rot*m_plane.m_Pose.m_Rot;
		t_plane.m_Pose.m_X = global_pose.m_Rot*m_plane.m_Pose.m_X + global_pose.m_X;
		t_plane.contours.clear();

		for(int brC = 0; brC < m_plane.contours.size(); brC++){
			CONTOUR_S tmp_contour;
			tmp_contour.bHole=m_plane.contours[brC].bHole;
			tmp_contour.P3D.clear();
			for(int brP = 0; brP < m_plane.contours[brC].poly_contour.size(); brP++){
				trans_col = t_plane.m_Pose.m_Rot*P2D23D(m_plane.contours[brC].poly_contour[brP])+t_plane.m_Pose.m_X;
				tmp_contour.P3D.push_back(trans_col);
			}
			t_plane.contours.push_back(tmp_contour);
		}
	}

	void detect_roof_floor(){
		double plane_angle;
		for (int i = 0; i < global_planes.size(); i++){
			if(global_planes[i].global){
				plane_angle = acos(global_planes[i].t_plane.m_Pose.m_N(1));//global_planes[i].map_params.angle;
				if(fabs(plane_angle-PI/2)<wall_angle_THRESHOLD){
					global_planes[i].map_params.wall=true;
					global_planes[i].map_params.floor=false;
					global_planes[i].map_params.roof=false;
					//here call tlocrt
				}else if(((fabs(plane_angle)<roof_floor_angle_THRESHOLD) || (fabs(plane_angle-PI)<roof_floor_angle_THRESHOLD))){
					if(global_planes[i].map_params.maxy<min_roof_height){
						global_planes[i].map_params.roof=true;
						global_planes[i].map_params.wall=false;
						global_planes[i].map_params.floor=false;
					}else{
						global_planes[i].map_params.floor=true;
						global_planes[i].map_params.wall=false;
						global_planes[i].map_params.roof=false;
					}
				}
			}
		}
#ifdef TRANSFORM_LOCAL_PLANES
		for (int i = 0; i < models_local_planes.size(); i++){
			for(int j = 0; j < models_local_planes[i].size();j++){
				if(local_models[i].changed || local_models[i].is_new){
					if(models_local_planes[i][j].big && (models_local_planes[i][j].global_id==-1)){
						plane_angle = acos(models_local_planes[i][j].t_plane.m_Pose.m_N(1));//global_planes[i].map_params.angle;
						if(fabs(plane_angle-PI/2)<wall_angle_THRESHOLD){
							models_local_planes[i][j].wall=true;
							models_local_planes[i][j].floor=false;
							models_local_planes[i][j].roof=false;
						}else{
							models_local_planes[i][j].floor=true;
							models_local_planes[i][j].wall=false;
							models_local_planes[i][j].roof=false;

						}
					}
				}
			}
		}
#endif
	}

	void tlocrt(){
		double maxx,minx,maxxy,minxy,maxxz,minxz,maxz,minz,miny,maxy,brPmin,brPmax,mean_y,minyx,minyz,maxyx,maxyz;
		polygon hole_n;
		Matrix3d GlobRot,Iref;
		Vector3d Tp,Tp_tmp,Tp_tmp2,GlobTran,Tpl1,Tpl2,Tplt1,Tplt2;
		vector<lines> output;
		vector<lines> output_tmp;
		FILE *pFile;
		FILE *pFile2;
		lines line1;
		char buffer[100];
		snprintf(buffer, sizeof(buffer), "tlocrt/%d_tlocrt_%ld_%ld.txt",broj_pozvanih,slam_trajectory.trajectories[0].size(),slam_trajectory.trajectories[1].size());
		M_POSE_PLANE Pose;

		double visina = -1000.0;
		bool krov;
		pFile = fopen((temp_path + buffer).c_str(),"w");

		roof_surfaces.clear(); floor_surfaces.clear(); wall_surfaces.clear();
		for (int i = 0; i < global_planes.size(); i++){
			if(global_planes[i].global){
				if(global_planes[i].map_params.wall){
					wall_surfaces.push_back(i);
				}
			}
		}

		for (int br_planes = 0; br_planes < wall_surfaces.size(); br_planes++){
			int i = wall_surfaces[br_planes];
			set_glob_rot_tranM(0,global_planes[i].ref_plane.model_id,Pose);
			GlobRot = Pose.m_Rot*global_planes[i].m_plane.m_Pose.m_Rot;
			GlobTran = Pose.m_Rot*global_planes[i].m_plane.m_Pose.m_X + Pose.m_X;
			Iref = global_planes[i].m_plane.m_Pose.m_Rot.inverse();
			snprintf(buffer, sizeof(buffer), "model/%d_model_%d.txt",broj_pozvanih,br_planes);
			//pFile2 = fopen((temp_path + buffer).c_str(),"w");
			//fprintf(pFile2,"%ld %ld %ld %f %f %f %f %f %f\n",9999999999,9999999999,9999999999,0.0,0.0,0.0,0.0,0.0,0.0);
			for(int brC = 0; brC < global_planes[i].m_plane.contours.size(); brC++){
				output.clear();
				hole_n.clear();
				mean_y = 0.0;
				for(int brP = 0; brP < global_planes[i].m_plane.contours[brC].poly_contour.size(); brP++){
					point[0]=(double)global_planes[i].m_plane.contours[brC].poly_contour[brP].X;
					point[1]=(double)global_planes[i].m_plane.contours[brC].poly_contour[brP].Y;
					append(hole_n,point);
					Tp = global_planes[i].m_plane.m_Pose.m_Rot*P2D23D(global_planes[i].m_plane.contours[brC].poly_contour[brP])+global_planes[i].m_plane.m_Pose.m_X;
					Tp_tmp = GlobRot*P2D23D(global_planes[i].m_plane.contours[brC].poly_contour[brP])+GlobTran;
					Tp_tmp2 = P2D23D(global_planes[i].m_plane.contours[brC].poly_contour[brP]);

					//fprintf(pFile2,"%f %f %f %f %f %f %f %f %f\n",Tp(0),Tp(1),Tp(2),Tp_tmp(0),Tp_tmp(1),Tp_tmp(2),Tp_tmp2(0),Tp_tmp2(1),Tp_tmp2(2));
					if(brP==0){
						maxy=miny=maxxy=minxy=Tp(1);
						maxx=minx=maxyx = minyx=Tp(0);
						maxxz = minxz = maxyz = minyz = Tp(2);
						brPmin=brPmax = brP;
					}
					mean_y = mean_y + Tp(1);
					if(Tp(1)>maxy){
						maxy=Tp(1);
						maxyx = Tp(0);
						maxyz = Tp(2);
					}

					if(Tp(1)<miny){
						miny=Tp(1);
						minyz = Tp(2);
						minyx = Tp(0);
					}
					if(Tp(0)>=maxx){
						maxx=Tp(0);
						maxxy=Tp(1);
						maxxz=Tp(2);
						brPmax = brP;
					}

					if(Tp(0)<minx){
						minx=Tp(0);
						minxy=Tp(1);
						minxz=Tp(2);
						brPmin = brP;
					}
				}
				mean_y = mean_y/global_planes[i].m_plane.contours[brC].poly_contour.size();
				double mean_ymax,mean_ymin;
				mean_ymax = mean_ymin = 0.0;
				double d_x = (maxx - minx)/2.0+minx;
				int bminy,bmaxy;
				bminy=bmaxy=0;
				for(int brP = 0; brP < global_planes[i].m_plane.contours[brC].poly_contour.size(); brP++){
					Tp = global_planes[i].m_plane.m_Pose.m_Rot*P2D23D(global_planes[i].m_plane.contours[brC].poly_contour[brP])+global_planes[i].m_plane.m_Pose.m_X;
					if(Tp(0)>d_x){
						mean_ymax = mean_ymax+Tp(1);
						bmaxy = bmaxy + 1;
					}else{
						mean_ymin = mean_ymin+Tp(1);
						bminy = bminy+1;
					}
				}
				mean_ymax = mean_ymax/bmaxy;
				mean_ymin = mean_ymin/bminy;

				if(global_planes[i].m_plane.contours[brC].poly_contour.size()>=2){
					boost::geometry::correct(hole_n);

					Tpl1(0) = maxx; Tpl1(1) = mean_ymax;Tpl1(2) = maxxz;
					Tpl2(0) = minx;	Tpl2(1) = mean_ymin; Tpl2(2) = minxz;
					Tp_tmp = Pose.m_Rot*Tpl1+Pose.m_X;
					Tp_tmp2 = Pose.m_Rot*Tpl2+Pose.m_X;

					Tplt1 = Iref*(Tpl1-global_planes[i].m_plane.m_Pose.m_X);
					Tplt2 = Iref*(Tpl2-global_planes[i].m_plane.m_Pose.m_X);

					//fprintf(pFile2,"%ld %ld %ld %f %f %f %f %f %f\n",9999999999,9999999999,9999999999,Tplt1(0),Tplt2(0),Tplt1(1),Tplt2(1),Tplt1(2),Tplt2(2));


					line1.clear();
					point[0]=Tplt1(0); point[1]=Tplt1(1); append(line1,point);
					point[0]=Tplt2(0); point[1]=Tplt2(1); append(line1,point);

					bool b = boost::geometry::intersection(line1, hole_n,output);

/*					Tpl1(0) = maxyx; Tpl1(1) = maxxy;Tpl1(2) = maxyz;
					Tpl2(0) = minyx;	Tpl2(1) = minxy; Tpl2(2) = minyz;
					Tplt1 = Iref*(Tpl1-global_planes[i].m_plane.m_Pose.m_X);
					Tplt2 = Iref*(Tpl2-global_planes[i].m_plane.m_Pose.m_X);
					Tpl1 = GlobRot*Tplt1+GlobTran;
					Tpl2 = GlobRot*Tplt2+GlobTran;*/

					if((maxy>visina)&&(output.size()>0)){
						for(int j=0; j<output.size(); j++){
							int brojac_outputa = 0;
							for(lines::iterator it1 = boost::begin(output[j]); it1 != boost::end(output[j]); ++it1){
								brojac_outputa++;
							}
							if(brojac_outputa==2){
								for(lines::iterator it1 = boost::begin(output[j]); it1 != boost::end(output[j]); ++it1){
									Tp(0)=it1->get<0>();
									Tp(1)=it1->get<1>();
									Tp(2)=0;
									Tp = GlobRot*Tp+GlobTran;
									fprintf(pFile,"%f %f %f\n",(float)Tp(0),(float)Tp(1),(float)Tp(2));
								}
							}else{
								ROS_ERROR("Tlocrt kriv... %d\n",brojac_outputa);
							}
							fprintf(pFile,"%d %d %d\n",999999999,999999999,999999999);
						}
					}
				}
			}
			//fclose(pFile2);
		}
		fclose(pFile);
	}

	void format_global_plane(f_mec& cMec, bool simple){
		int glob_id, tmp_id,tmp_model_id;
		bool is_ref_plane, is_ref_model;
		double x_,y_,z_,c_,s_,m_d_,normE;
		//UT ut;
		M_POSE_PLANE RelPose;

		VectorXd eVec(3),Ne_(3);
		MatrixXd CovMat(3,3),R_(3,3),tmp_1(3,3), tmp_2(3,3);
		VectorXd normal_z(4),eVecZ(4),final_e(4),E_(4);
		MatrixXd normal_z_C(4,4), ICovMatZ(4,4),final_cov(4,4),Cov_(4,4);

		eVec.setZero();
		CovMat.setIdentity();
		final_cov.setZero();
		final_e.setZero();
		glob_id = cMec.final_glob_id;

		for(int i = 0; i < 3; i++){
			for(int j = 0; j < 3; j++){
				cMec.refRot(i,j) = local_models[cMec.ref_model_id].local_planes[cMec.ref_plane_id].m_Pose.m_Rot(i,j);
			}
			cMec.refTran(i) = local_models[cMec.ref_model_id].local_planes[cMec.ref_plane_id].m_Pose.m_X(i);
		}

		if(simple){
			global_planes[cMec.final_glob_id].copy_to_mplane(local_models[cMec.ref_model_id].local_planes[cMec.ref_plane_id]);
		}
	}

	void combine_pairs(){
		bool dobar_poligon,transform;
		Matrix3d Iref,R_final;
		Vector3d Tref,T_final;
		M_POSE_PLANE global_pose;
		int tmp_model_id,tmp_plane_id,tmp_glob_id;

		//printf("	MAP: Combining planes... %ld\n",s_mec.size());

		for(int j = 0; j < s_mec.size(); j++){
			for (int i=0;i<s_mec[j].parovi.size();i++){
				transform = true;
				tmp_model_id = s_mec[j].parovi[i].model_id;
				tmp_plane_id = s_mec[j].parovi[i].id;
				tmp_glob_id = s_mec[j].parovi[i].global_plane_id;
				if((tmp_model_id == s_mec[j].ref_model_id) && (tmp_plane_id == s_mec[j].ref_plane_id)){
					transform = false;
				}/*else{*/
				Iref=s_mec[j].refRot.inverse();
				Tref=s_mec[j].refTran;
				set_glob_rot_tranM(s_mec[j].ref_model_id,tmp_model_id,global_pose);
				R_final = Iref*global_pose.m_Rot*local_models[tmp_model_id].local_planes[tmp_plane_id].m_Pose.m_Rot;
				T_final = Iref*(global_pose.m_Rot*local_models[tmp_model_id].local_planes[tmp_plane_id].m_Pose.m_X+global_pose.m_X-Tref);
				//}
				if(tmp_glob_id!=-1){
					dobar_poligon=add_polygon_from_plane(transform,&s_mec[j],R_final,T_final,global_planes[tmp_glob_id].m_plane);
				}else{
					dobar_poligon=add_polygon_from_plane(transform,&s_mec[j],R_final,T_final,local_models[tmp_model_id].local_planes[tmp_plane_id]);
				}
			}
			polygon_union(&s_mec[j],j,global_planes[s_mec[j].final_glob_id].m_plane);
			global_planes[s_mec[j].final_glob_id].area=calculate_polygon_area(&s_mec[j]);
		}
	}

	bool reformat_global_plane(int id){
		//printf("	MAP: Starting to reformat plane %d\n",id);
		//printf("	MAP: Total number of planes: %ld\n",global_planes[id].local_planes.size());

		int model_id_o,model_id_u,plane_id_o,plane_id_u,final_glob_id;
		double distance_,angle_;
		bool match_initial,match_intersect;
		VectorXd dodane;
		f_mec cMec;
		PLANE tmp_plane;
		M_POSE_PLANE global_pose;
		vector<vector<PLANE> > all_reformat_pairs;

		svi_mec.resize(global_planes[id].local_planes.size(),global_planes[id].local_planes.size()); svi_mec.setIdentity();
		redovi_dodani.resize(global_planes[id].local_planes.size()); redovi_dodani.setZero();
		dodane.resize(global_planes[id].local_planes.size()); dodane.setZero();
		stupci_dodani.resize(global_planes[id].local_planes.size()); stupci_dodani.setZero();
		all_plane_pairs.clear();

		for (int o = 0; o<global_planes[id].local_planes.size(); o++){
			model_id_o = global_planes[id].local_planes[o].model_id;
			plane_id_o = global_planes[id].local_planes[o].id;
			for(int u = o+1; u < global_planes[id].local_planes.size(); u++){
				model_id_u = global_planes[id].local_planes[u].model_id;
				plane_id_u = global_planes[id].local_planes[u].id;
				set_glob_rot_tranM(model_id_u,model_id_o,global_pose);
				match_initial = MatchAlignment(local_models[model_id_o].local_planes[plane_id_o],local_models[model_id_u].local_planes[plane_id_u], global_pose, distance_, angle_, N_THRESHOLD_local, mD_THRESHOLD_local, true);
				if(match_initial){
					svi_mec(u,o) = 1;
				}
			}
		}

		nadi_parove(0,1);
		//printf("	MAP: Number of pairs found: %ld\n",all_plane_pairs.size());
		/*if(all_plane_pairs.size()==1){
			global_planes[id].changed = true;
			printf("	MAP: No changes required.\n");
		}else{*/
		s_mec.clear();
		for(int i = 0; i < all_plane_pairs.size();i++){
			vector<PLANE> tmp_planes;
			for(int j=0; j<all_plane_pairs[i].size();j++){
				if(!dodane(all_plane_pairs[i][j].id)){
					dodane(all_plane_pairs[i][j].id)=1;
					tmp_plane.global_plane_id = -1;
					tmp_plane.model_id = global_planes[id].local_planes[all_plane_pairs[i][j].id].model_id;
					tmp_plane.id = global_planes[id].local_planes[all_plane_pairs[i][j].id].id;
					tmp_planes.push_back(tmp_plane);
					local_models[tmp_plane.model_id].local_planes[tmp_plane.id].global_id = -1;
					local_models[tmp_plane.model_id].local_planes[tmp_plane.id].ref_plane.model_id = tmp_plane.model_id;
					local_models[tmp_plane.model_id].local_planes[tmp_plane.id].ref_plane.id = tmp_plane.id;
				}
			}
			if(tmp_planes.size()>1)
				all_reformat_pairs.push_back(tmp_planes);
		}
		for(int i = 0; i < all_reformat_pairs.size();i++){
			cMec.polygons.clear();
			cMec.pair_group_id = i;
			cMec.parovi.clear();
			cMec.max_area = -1;
			cMec.ref_model_id = total_models+10;
			for(int j=0; j<all_reformat_pairs[i].size();j++){
				setRefPlane(&cMec, local_models[all_reformat_pairs[i][j].model_id].local_planes[all_reformat_pairs[i][j].id].area, all_reformat_pairs[i][j].model_id, all_reformat_pairs[i][j].id);
				cMec.parovi.push_back(all_reformat_pairs[i][j]);
			}
			final_glob_id = add_global_plane(&cMec);
			global_planes[final_glob_id].reformat = true;
			//printf("	MAP: added glob plane: %d\n",final_glob_id);
			cMec.final_glob_id = final_glob_id;
			format_global_plane(cMec,true);
			s_mec.push_back(cMec);
		}
		combine_pairs();
		delete_global_plane(id);
		//}
		global_planes[id].reformat = true;
		return true;
	}

	bool update_planes(int model_id1, int model_id2){
		//printf("	MAP: Updating planes %d %d\n",model_id1,model_id2);
		int num_initial,num_paired,tmp_glob_id,final_glob_id,tmp_model_id,tmp_plane_id,sigma_rejected, passed_mean_test;
		bool match_initial,match_intersect,novi_red;
		double test_score, distance_, angle_, total_format_time, mean, sigma, max_mean, min_mean, sigma_mul;

		M_POSE_PLANE PoseSMCMap;

		INTERSECTION_MATCH intersections;
		total_format_time = 0;
		FILE *pFIle; char buffer[100];

		//printf("setting rotation\n");
		set_glob_rot_tranM(model_id1,model_id2,PoseSMCMap);
#ifdef DO_LOG
		snprintf(buffer, sizeof(buffer), "psulm/%d_%d_paired_planes.txt",broj_pozvanih,model_id1);
		pFIle = fopen((temp_path + buffer).c_str(),"w");
		for(int i = 0; i < models_local_planes[model_id1].size(); i++){
			if(models_local_planes[model_id1][i].global_id!=-1){
				fprintf(pFIle,"%d\n",models_local_planes[model_id1][i].ID.id);
			}
		}
		fclose(pFIle);

		save_psulm(psulm1,model_id1,broj_pozvanih,&NullPose);
		save_psulm(psulm2,model_id2,broj_pozvanih,&PoseSMCMap);
		snprintf(buffer, sizeof(buffer), "psulm/%d_%d_paired_planes2.txt",broj_pozvanih,model_id2);
		pFIle = fopen((temp_path + buffer).c_str(),"w");
#endif

		svi_mec.resize(local_models[model_id1].SurfacesTotal,local_models[model_id2].SurfacesTotal); svi_mec.setZero();
		redovi_dodani.resize(local_models[model_id1].SurfacesTotal); redovi_dodani.setZero();
		stupci_dodani.resize(local_models[model_id2].SurfacesTotal); stupci_dodani.setZero();
		all_plane_pairs.clear();
		id_red.clear();
		vector<match_data> final_match_data;
		match_data tmp_match_data;

		num_paired=num_initial = sigma_rejected = 0;
		//printf("	MAP: Planes in scene: %d, model: %d\n",local_models[model_id2].SurfacesTotal,local_models[model_id1].SurfacesTotal);
		for (int o = 0; o<local_models[model_id2].SurfacesTotal; o++){
			lok_rav_2 = local_models[model_id2].local_planes[o].ref_plane.id;
			tmp_match_data.distance=100000;
			tmp_match_data.angle = 100000;
			for(int u = 0; u < local_models[model_id1].SurfacesTotal; u++){
				if(local_models[model_id1].local_planes[u].big && local_models[model_id2].local_planes[o].big){
					match_initial = MatchAlignment(local_models[model_id2].local_planes[o], local_models[model_id1].local_planes[u], PoseSMCMap, distance_, angle_, N_THRESHOLD_local, mD_THRESHOLD_local, true);
					lok_rav = local_models[model_id1].local_planes[u].ref_plane.id;
					if(match_initial){
						num_initial++;
						match_intersect=MatchIntersect(local_models[model_id2].local_planes[o], local_models[model_id1].local_planes[u], PoseSMCMap, intersections);
						if(match_intersect){
							num_paired++;
							svi_mec(lok_rav,lok_rav_2) = 1;
							tmp_match_data.distance = distance_;
							tmp_match_data.angle = angle_;
							tmp_match_data.plane_id1 = lok_rav;
							tmp_match_data.plane_id2 = lok_rav_2;
							final_match_data.push_back(tmp_match_data);
#ifdef DO_LOG
							fprintf(pFIle,"%d %d %d %d %d  %d %d %d %d  %f %f\n",o,u,lok_rav_2,lok_rav,models_local_planes[model_id1][u].global_id,distance_, angle_);
#endif
						}
					}
				}
			}
		}
#ifdef DO_LOG
		fclose(pFIle);
#endif

		paired_distances.clear();
		for(int i = 0; i < final_match_data.size();i++){
			paired_distances.push_back(final_match_data[i].distance);
		}
		for(int i = 0; i < 1; i++){
			setMeanSigma(mean, sigma, max_mean, min_mean, 1.0);
		}
		//printf("	MAP: Mean %f Sigma %f Max_mean %f Min_mean %f\n",mean,sigma,max_mean,min_mean);
		passed_mean_test = 0;
		for(int j = 0; j < paired_distances.size(); j++){
			if(paired_distances[j]!=-1){
				passed_mean_test++;
			}else{
				svi_mec(final_match_data[j].plane_id1,final_match_data[j].plane_id2) = 0;
			}
		}
		//printf("	MAP: Of %ld starting %d passed\n",paired_distances.size(),passed_mean_test);

		nadi_parove(model_id1,model_id2);

		//printf("	MAP: Number of: paired initial %d,  paired: %d,  test %ld\n",num_initial,num_paired,all_plane_pairs.size());
		f_mec cMec;
		s_mec.clear();
		for(int i = 0; i < all_plane_pairs.size();i++){
			cMec.polygons.clear();
			cMec.pair_group_id = i;
			cMec.parovi.clear();
			id_red.clear();
			novi_red=true;

			//find all local planes that already belong to global planes and add only reference plane for that global plane in group
			cMec.max_area = -1;
			cMec.ref_model_id = total_models+10;
			for(int k=0; k<id_red.size(); k++){
				global_planes[id_red[k]].added = false;
			}

			for(int j=0; j<all_plane_pairs[i].size();j++){
				tmp_model_id = all_plane_pairs[i][j].model_id;
				tmp_plane_id = all_plane_pairs[i][j].id;
				tmp_glob_id = local_models[tmp_model_id].local_planes[tmp_plane_id].global_id;
				if(tmp_glob_id!=-1){
					if(global_planes[tmp_glob_id].added==false){
						global_planes[tmp_glob_id].added = true;
						global_planes[tmp_glob_id].num_called=global_planes[tmp_glob_id].num_called+1;
						id_red.push_back(tmp_glob_id);
						cMec.parovi.push_back(global_planes[tmp_glob_id].ref_plane);
						novi_red = false;
						setRefPlane(&cMec,global_planes[tmp_glob_id].area, global_planes[tmp_glob_id].ref_plane.model_id, global_planes[tmp_glob_id].ref_plane.id);
					}
				}else{
					setRefPlane(&cMec, local_models[tmp_model_id].local_planes[tmp_plane_id].area, tmp_model_id, tmp_plane_id);
					cMec.parovi.push_back(all_plane_pairs[i][j]);
				}
			}

			//reset global planes flags
			for(int k=0; k<id_red.size(); k++){
				global_planes[id_red[k]].added = false;
			}

			if(novi_red){ //add new global plane
				final_glob_id = add_global_plane(&cMec);
			}else{ // this executes if local plane belonged to global planes
				final_glob_id = id_red[0];
				for(int k=0; k<all_plane_pairs[i].size();k++){ //add local planes that did not belong to global planes to merged global plane
					tmp_glob_id = local_models[all_plane_pairs[i][k].model_id].local_planes[all_plane_pairs[i][k].id].global_id;
					if(tmp_glob_id==-1){
						global_planes[id_red[0]].local_planes.push_back(all_plane_pairs[i][k]);
					}
				}
				if(id_red.size()==1)
					simple_update_global_planes(id_red[0]);
				else{
					for(int k=1; k<id_red.size();k++){
						merge_global_planes(id_red[0],id_red[k]); //merge all detected global planes
					}
				}
				//set parameter of merged plane
				global_planes[final_glob_id].ref_plane.model_id = cMec.ref_model_id;
				global_planes[final_glob_id].ref_plane.id = cMec.ref_plane_id;
			}
			cMec.final_glob_id = final_glob_id;
			ros::Time t_format = ros::Time::now();
			format_global_plane(cMec,true);
			ros::Duration d_format = ros::Time::now() - t_format;
			total_format_time = total_format_time + d_format.toSec();
			s_mec.push_back(cMec);
		}

		//printf("	MAP: Total format duration: %f\n",total_format_time);
		combine_pairs();
		//printf("	MAP: Updating planes %d %d OK\n",model_id1,model_id2);
		return true;
	}

	bool refine_glob(){
		int broj_sparenih=0, tmp_model_id, tmp_plane_id,tmp_glob_id,num_in_pairs;
		bool dobar_poligon,match_initial,match_intersect,matched_one_new,matched;
		double distance_,angle_,mean_distance,sigma_distance,max_mean,min_mean,intersect_score;
		VectorXd dodane;
		Matrix3d R_final,Iref;
		Vector3d T_final;
		INTERSECTION_MATCH intersections;
		PLANE tmp_plane;
		M_POSE_PLANE global_pose;
		global_pose.Reset();

		svi_mec.resize(global_planes.size(),global_planes.size());
		svi_mec.setIdentity();
		redovi_dodani.resize(global_planes.size()); redovi_dodani.setZero();
		dodane.resize(global_planes.size()); dodane.setZero();
		stupci_dodani.resize(global_planes.size()); stupci_dodani.setZero();
		all_plane_pairs.clear();
		id_red.clear();

		//printf("Planes in global: %ld\n",global_planes.size());
		for (int o = 0; o<global_planes.size(); o++){
			if(global_planes[o].global && global_planes[o].changed){
				for(int u = 0; u < global_planes.size(); u++){
					if((o!=u) && (global_planes[u].global)){
						match_initial = MatchAlignment(global_planes[o].t_plane, global_planes[u].t_plane, global_pose, distance_, angle_, N_THRESHOLD_global, mD_THRESHOLD_global, true);
						if(match_initial){
							match_intersect=MatchIntersect(global_planes[o].t_plane, global_planes[u].t_plane, global_pose, intersections);
							if(match_intersect){
								broj_sparenih++;
								svi_mec(u,o) = 1;
							}
						}
					}
				}
			}
		}

		const clock_t begin_t = clock();
		nadi_parove(0,1);
		double elapsed_secs = double(clock() - begin_t) / CLOCKS_PER_SEC;

		//printf("Number of paired in global: %d, pairs: %ld %f\n",broj_sparenih,all_plane_pairs.size(),elapsed_secs);

		f_mec cMec;
		s_mec.clear();
		for(int j = 0; j < all_plane_pairs.size(); j++){
			cMec.polygons.clear();
			cMec.pair_group_id = j;
			setRefglob(&cMec);
			Iref = cMec.refRot.inverse();
			for (int i=0;i<all_plane_pairs[j].size();i++){
				set_glob_rot_tranM(global_planes[cMec.ref_plane_id].ref_plane.model_id,global_planes[all_plane_pairs[j][i].id].ref_plane.model_id, global_pose);
				R_final = Iref*global_pose.m_Rot*global_planes[all_plane_pairs[j][i].id].m_plane.m_Pose.m_Rot;
				T_final = Iref*(global_pose.m_Rot*global_planes[all_plane_pairs[j][i].id].m_plane.m_Pose.m_X+global_pose.m_X-cMec.refTran);
				dobar_poligon=add_polygon_from_plane(true,&cMec,R_final,T_final,global_planes[all_plane_pairs[j][i].id].m_plane);
				if(all_plane_pairs[j][i].id != cMec.ref_plane_id){
					merge_global_planes(cMec.ref_plane_id,all_plane_pairs[j][i].id);
				}
			}
			polygon_union(&cMec,-1,global_planes[cMec.ref_plane_id].m_plane);
			global_planes[cMec.ref_plane_id].area =calculate_polygon_area(&cMec);
			transform_to_glob(global_planes[cMec.ref_plane_id].m_plane,0,global_planes[cMec.ref_plane_id].ref_plane.model_id,global_planes[cMec.ref_plane_id].t_plane);
		}

		return true;
	}

	void gen_glob_map(){
		int num_global_planes,num_local_planes,num_global_planes_added;
		FILE *pFile; char buffer[100];
		num_global_planes_added = 0;
		//printf("______________________________________________________________________________\n"); printf("\n");
		//printf("Generating global model\n");
		for(int i=0; i<global_planes.size();i++){
			if((global_planes[i].active)){
				num_global_planes_added++;
				global_planes[i].global = true;
				if(global_planes[i].changed){
					transform_to_glob(global_planes[i].m_plane, 0,global_planes[i].ref_plane.model_id,global_planes[i].t_plane);
				}

			}
		}
#ifdef TRANSFORM_LOCAL_PLANES
		printf("	MAP: Total global planes changed %d\n",num_changed);
		for (int i = 0; i < models_local_planes.size(); i++){
			for(int j = 0; j < models_local_planes[i].size();j++){
				if(local_models[i].is_new || local_models[i].changed){
					if(models_local_planes[i][j].big && (models_local_planes[i][j].global_id==-1)){
						currentS = models_local_planes[i][j].pModel->m_3DSurfaceArray[j];
						transform_to_glob(0,i,models_local_planes[i][j].t_plane);
						num_added_local++;
					}
				}
			}
		}

		printf("	MAP: Total local planes added to global: %d\n",num_added_local);
#endif
		//printf("Global model succesfully generated: %d %d\n",num_global_planes_added,broj_pozvanih);
		detect_roof_floor();

		ros::param::get("/do_map_update", do_map_update);
		detect_global_planes(num_global_planes,num_local_planes);
		if((!(broj_pozvanih%REFINE_AND_REFORMAT_EVERY))||SEND_NOW||do_map_update){
			ros::param::set("/do_map_update", 0);

			//printf("______________________________________________________________________________\n"); printf("\n");
			//printf("Refinig global model %d %d\n",num_global_planes,num_local_planes);
			const clock_t begin_t = clock();
			refine_glob();
			double elapsed_secs = double(clock() - begin_t) / CLOCKS_PER_SEC;
			detect_global_planes(num_global_planes,num_local_planes);
			//printf("Model refined successfully %d %d\n",num_global_planes,num_local_planes);
			//printf("______________________________________________________________________________\n"); printf("\n");
			//printf("Duration refine: %f ms\n",elapsed_secs*1000);
#ifdef USE_VTK
			if((!(broj_pozvanih%SEND_MAP_EVERY))||SEND_NOW||do_map_update){
				printf("Building stl map and sending to RVIz\n");
				send_map_rviz(broj_pozvanih);
				new_map_ = true;
			}else{
				new_map_ = false;
			}
#endif
			//tlocrt();
		}

		for (int o = 0; o<global_planes.size(); o++){
			global_planes[o].changed = false;
			global_planes[o].reformat = false;
		}
		for (int o = 0; o<local_models.size(); o++){
			local_models[o].is_new=false;
			local_models[o].changed = false;
		}
		//printf("Total local planes %d Total planes: %d\n",num_local_planes,num_global_planes);
		broj_pozvanih++;
	}

	void load_models(bool &new_map){

		//ROS_INFO("MAP UPDATE REQUEST %d %d\n",models_in_trajectory,total_models);
		//printf("______________________________________________________________________________\n"); printf("\n");
		/*	compute_L();*/

		int new_models_in_trajectory,num_deleted,num_models,num_changed_models, first_model_id, deleted_center,deleted_area,num_local_planes,num_global_planes,tmp_glob_id;
		double center_distance,plane_angle,orientation_diff,pose_diff,area_local,area_global;
		bool res_changed;
		char buffer[200];
		FILE *pFile;

		Vector3d Tp,tmp_col;
		RVL3DCONTOUR* currentC;
		RVL3DPOINT3* currentP;
		Path poly;
		Paths poly_simplified;

		VectorXd tmp_x;
		MatrixXd tmp_P,Iref;
		double trace_P;

		deleted_center = deleted_area = 0;
		first_model_id = local_models.size();
		//printf("%d  %d  %d\n",first_model_id,total_models_agents[agent_id],last_model_id[agent_id]);
		for (int i = total_models_agents[agent_id]; i < models_in_trajectory; i++){
			int brloc = local_models.size();
			MODEL tmp_model;
			num_deleted = 0;
			tmp_model.changed = false;
			tmp_model.is_new = true;
			tmp_model.map_changed = false;
			tmp_model.agent_id = agent_id;
			tmp_model.model_agent_id = i;
			snprintf(buffer, sizeof(buffer), "server_models/agent_%d/%d_model.txt",agent_id,i);
			pFile = fopen((temp_path + buffer).c_str(),"r");
			tmp_model.pModel = VS.m_PSuLMBuilder.Load(pFile);
			fclose(pFile);
			printf("Load OK\n");
			tmp_model.SurfacesTotal = tmp_model.pModel->m_n3DSurfacesTotal;
			printf("tmp %d\n",tmp_model.SurfacesTotal);
			printf("local %d\n",tmp_model.pModel->m_n3DSurfacesTotal);
			local_models.push_back(tmp_model);
			printf("Local OK\n");
			all_model_ids[agent_id].push_back(brloc);
			for(int k=0; k<local_models[brloc].pModel->m_n3DSurfacesTotal;k++){
				LOCAL_PLANE tmp_plane;
				tmp_plane.global_id = -1;
				tmp_plane.deleted = false;
				tmp_plane.ref_plane.id = k;
				tmp_plane.ref_plane.model_id = brloc;
				tmp_plane.contours.clear();
				local_models[brloc].local_planes.push_back(tmp_plane);

				currentS = local_models[brloc].pModel->m_3DSurfaceArray[k];
				for(int bri = 0; bri < 3; bri++){
					for(int brj = 0; brj < 3; brj++){
						local_models[brloc].local_planes[k].m_Pose.m_Rot(bri,brj)=currentS->m_Pose.m_Rot[bri*3+brj];
					}
					local_models[brloc].local_planes[k].m_Pose.m_X(bri) = currentS->m_Pose.m_X[bri];
					local_models[brloc].local_planes[k].m_Pose.m_N(bri) = currentS->m_N[bri];
					local_models[brloc].local_planes[k].m_varq(bri) = currentS->m_varq[bri];
				}
				local_models[brloc].local_planes[k].m_d = currentS->m_d;

				center_distance=sqrt(pow(currentS->m_Pose.m_X[0],2)+pow(currentS->m_Pose.m_X[1],2)+pow(currentS->m_Pose.m_X[2],2));
				plane_angle = acos(currentS->m_N[1]);
				if ((center_distance>CENTER_THRESHOLD) || (fabs(plane_angle-PI/2)>wall_angle_THRESHOLD)){
					local_models[brloc].local_planes[k].deleted = true;
					if((center_distance>CENTER_THRESHOLD)&&(fabs(plane_angle-PI/2)<=wall_angle_THRESHOLD)){
						deleted_center++;
					}
				}else{
					currentC = (RVL3DCONTOUR*)currentS->m_BoundaryContourList.pFirst;
					area_global=0;
					while (currentC){
						if (currentC->bHole){
							currentC = (RVL3DCONTOUR*)currentC->pNext;
							continue;
						}
						poly.clear();

						currentP = (RVL3DPOINT3*)currentC->PtList.pFirst;
						Iref = local_models[brloc].local_planes[k].m_Pose.m_Rot.inverse();
						while (currentP){
							Tp(0)=currentP->P3D[0];	Tp(1)=currentP->P3D[1];	Tp(2)=currentP->P3D[2];
							tmp_col = Iref*(Tp-local_models[brloc].local_planes[k].m_Pose.m_X);
#ifdef VELODYNE32
							if((fabs(tmp_col(0))>M_THRESHOLD) || (fabs(tmp_col(1))>M_THRESHOLD) || (fabs(tmp_col(2))>M_THRESHOLD) || (fabs(Tp(1))>H_THRESHOLD)){
								local_models[brloc].local_planes[k].deleted = true;
							}
#endif
							poly.push_back(IntPoint((int)tmp_col(0),(int)tmp_col(1)));
							currentP = (RVL3DPOINT3*)currentP->pNext;
						}
						currentC = (RVL3DCONTOUR*)currentC->pNext;
						if(local_models[brloc].local_planes[k].deleted){
							break;
						}else{
							area_local = get_area(poly);
							if(area_local<AREA_THRESHOLD){
								local_models[brloc].local_planes[k].deleted = true;
								deleted_area++;
								break;
							}else{
								area_global = area_global+area_local;
								SimplifyPolygon(poly,poly_simplified);
								for(int brarea = 0; brarea<poly_simplified.size(); brarea++){
									if(get_area(poly_simplified[brarea])>AREA_THRESHOLD){
										CONTOUR_S tmp_contour;
										tmp_contour.bHole = false;
										tmp_contour.poly_contour=poly_simplified[brarea];
										local_models[brloc].local_planes[k].contours.push_back(tmp_contour);
									}
								}
								if(local_models[brloc].local_planes[k].contours.size()==0){
									local_models[brloc].local_planes[k].deleted = true;
									deleted_area++;
									break;
								}
							}
						}
					}
				}
				if(!local_models[brloc].local_planes[k].deleted){
					local_models[brloc].local_planes[k].area = area_global;
					local_models[brloc].local_planes[k].big = true;
					local_models[brloc].local_planes[k].inserted = false;
					local_models[brloc].local_planes[k].deleted = false;
					total_planes_local++;
				}else{
					local_models[brloc].local_planes[k].deleted = true;
					local_models[brloc].local_planes[k].big = false;
					num_deleted++;
				}

			}
			//printf("Deleted %d of %d planes. Model: %d  %d  %d\n",num_deleted, local_models[brloc].pModel->m_n3DSurfacesTotal,brloc,deleted_center,deleted_area);
		}
		total_models = local_models.size();

		//printf("Total models after addition: %ld\n",local_models.size());

		const clock_t begin_t = clock();

#ifdef DO_LOG
		for (int i = br_pog_old; i < br_pog; i++){
			save_psulm_no_pose(local_models[i].pModel,i);
		}
#endif

		//check and reformat planes that have changed locations
		num_changed_models = 0;
		for(int i =0; i < total_models_agents[agent_id];i++){
			res_changed = detect_changed_models(slam_trajectory.trajectories[agent_id][i], slam_trajectory.old_trajectories[agent_id][i],pose_diff,orientation_diff);
			if(res_changed){
				local_models[all_model_ids[agent_id][i]].changed=true;
				num_changed_models++;
				slam_trajectory.old_trajectories[agent_id][i]=slam_trajectory.trajectories[agent_id][i];
				for(int j = 0; j < local_models[all_model_ids[agent_id][i]].local_planes.size();j++){
					tmp_glob_id = local_models[all_model_ids[agent_id][i]].local_planes[j].global_id;
					if((tmp_glob_id!=-1) && !(global_planes[tmp_glob_id].reformat) && global_planes[tmp_glob_id].active){
						ROS_WARN("reformating GLOBAL PLANE %d\n",tmp_glob_id);
						reformat_global_plane(tmp_glob_id);
					}
				}
			}
		}
		//printf("MAP:	Number of models changed: %d\n",num_changed_models);

		//connect neighbouring modele
		if(last_model_id[agent_id]>=0){
			ROS_INFO("Sequential update %d %d\n", last_model_id[agent_id],first_model_id);
			update_planes(last_model_id[agent_id],first_model_id);
		}
		for (int i = first_model_id; i < total_models-1; i++){
			ROS_INFO("Sequential update %d %d\n", i,i+1);
			update_planes(i,i+1);
		}

		//connect loop closed ones
		for(int i = 0; i < loop_matches.size(); i++){
			ROS_INFO("Loop update %d %d\n", loop_matches[i].index1,loop_matches[i].index2);
			update_planes(all_model_ids[agent_id][loop_matches[i].index1],all_model_ids[agent_id][loop_matches[i].index2]);
		}

		gen_glob_map(); //generate global odel and do refine
		double elapsed_secs = double(clock() - begin_t) / CLOCKS_PER_SEC;

#ifdef DO_LOG
		snprintf(buffer, sizeof(buffer), "times/map.txt");
		pFile = fopen((temp_path + buffer).c_str(),"a");
		fprintf(pFile,"%lf %lf\n",elapsed_secs);
		fclose(pFile);


		detect_global_planes(num_global_planes,num_local_planes);
		snprintf(buffer, sizeof(buffer), "times/stanja.txt");
		pFile = fopen((temp_path + buffer).c_str(),"a");
		fprintf(pFile,"%d %d\n",num_global_planes,num_local_planes);
		fclose(pFile);
#endif
		last_model_id[agent_id]=local_models.size()-1;
		total_models_agents[agent_id] = models_in_trajectory;
		new_map = new_map_;
		printf("______________________________________________________________________________\n"); printf("\n");
		printf("MAP UPDATE DONE\n");
	}

	bool detect_changed_models(StateVector elem_new, StateVector elem_old, double &pose_diff, double &orientation_diff){
		StateVector pose_new,pose_old;
		pose_new = slam_trajectory.get_group(elem_new);
		pose_old = slam_trajectory.get_group(elem_old);
		for (int i = 0; i < 3; i++){
			pose_new(i+3,0)=pose_old(i+3,0)=0.0;
		}
		pose_diff = (pose_new-pose_old).norm();
		orientation_diff = fabs(elem_new(3,0) - elem_old(3,0));
		if (orientation_diff > PI) {
			orientation_diff = fabs(orientation_diff - _2PI);
		}
		if(pose_diff>POSE_THRESHOLD || orientation_diff>ORIENTATION_THRESHOLD){
			return true;
		}else{
			return false;
		}
	}

	void setMeanSigma(double &mean, double &sigma, double &max_mean, double &min_mean, double sigma_mul){
		double mean_distance,sigma_distances,set_distances;

		mean_distance = 0.0;
		sigma_distances = 0.0;
		sigma_distances = 0.0;
		for (int o = 0; o<paired_distances.size(); o++){
			if(paired_distances[o]!=-1){
				mean_distance = mean_distance+paired_distances[o];
				set_distances++;
			}
		}
		mean = mean_distance/set_distances;
		for (int o = 0; o<paired_distances.size(); o++){
			if(paired_distances[o]!=-1){
				sigma_distances = pow((mean-paired_distances[o]),2)+sigma_distances;
			}
		}
		sigma = sqrt(sigma_distances/(set_distances-1));
		max_mean = mean+sigma_mul*sigma;
		min_mean = mean-sigma_mul*sigma;
		for(int o=0; o < paired_distances.size(); o++){
			if((paired_distances[o]>max_mean) || (paired_distances[o]<min_mean))
				paired_distances[o]=-1;
		}
	}

	bool MatchAlignment(LOCAL_PLANE &Surface2, LOCAL_PLANE &Surface1, M_POSE_PLANE &pPose, double &distance_, double &angle_, double N_THRESHOLD, double mD_THRESHOLD, bool check_distance){
		double mD1, mD2, diff_mD, normal_angle;
		bool match_res;

		Vector3d N2,C2t;
		Matrix3d inv_local_Rot1;

		inv_local_Rot1 = Surface1.m_Pose.m_Rot.inverse();
		N2=pPose.m_Rot*Surface2.m_Pose.m_N;

		if(check_distance){
			C2t = inv_local_Rot1*pPose.m_Rot*Surface2.m_Pose.m_X + inv_local_Rot1*(pPose.m_X-Surface1.m_Pose.m_X);
			diff_mD = fabs(C2t(2));
			/*C2 = Rot1*C2nt+Tran1;

			mD1 = fabs(N1(0))*C1(0)+fabs(N1(1))*C1(1)+fabs(N1(2))*C1(2);
			mD2 = fabs(N2(0))*C2(0)+fabs(N2(1))*C2(1)+fabs(N2(2))*C2(2);

			diff_mD = fabs(mD1-mD2);*/
		}
		//normal_angle = acos(N2(2)*N1(2)+N2(1)*N1(1)+N2(0)*N1(0))*180/3.14;
		normal_angle = N2(2)*Surface1.m_Pose.m_N(2)+N2(1)*Surface1.m_Pose.m_N(1)+N2(0)*Surface1.m_Pose.m_N(0);

		match_res=false;
		distance_ = diff_mD;
		angle_ = acos(normal_angle)*RAD2DEG;
		if(fabs(normal_angle)>cos(N_THRESHOLD*DEG2RAD)){
			if(check_distance){
				if(diff_mD<mD_THRESHOLD){
					match_res = true;
				}else{
					match_res=false;
				}
			}else{
				match_res=true;
			}
		}else{
			match_res=false;
		}

		return match_res;
	}

	Vector3d P2D23D(IntPoint &P2D){
		Vector3d ret_vec;
		ret_vec(2)=0.0;
		ret_vec(0)=(double)P2D.X;
		ret_vec(1)=(double)P2D.Y;
		return ret_vec;
	}

	void find_plane_boundaries(bool transform, Matrix3d R_final, Vector3d T_final, LOCAL_PLANE &tmp_plane, vector<BOUNDARY_PLANE_PARAMS> &params){
		Vector3d tmp_col;
		params.clear();
		bool prva;
		BOUNDARY_PLANE_PARAMS tmp_params;
		for (int brC = 0; brC < tmp_plane.contours.size(); brC++){
			if (tmp_plane.contours[brC].bHole){
				continue;
			}
			prva = true;
			for(int brP = 0; brP < tmp_plane.contours[brC].poly_contour.size(); brP++){
				if(transform){
					tmp_col = R_final*P2D23D(tmp_plane.contours[brC].poly_contour[brP])+T_final;
				}else{
					tmp_col(0) = tmp_plane.contours[brC].poly_contour[brP].X;
					tmp_col(1) = tmp_plane.contours[brC].poly_contour[brP].Y;
				}
				point[0]=tmp_col(0);
				point[1]=tmp_col(1);
				if(prva){
					tmp_params.max_x.x = tmp_params.min_x.x = point[0]; tmp_params.max_y.y = tmp_params.min_y.y = point[1];
					tmp_params.max_x.y = tmp_params.min_x.y = point[1]; tmp_params.max_y.x = tmp_params.min_x.x = point[0];
					prva = false;
				}else{
					if(tmp_params.max_x.x < point[0]){
						tmp_params.max_x.x = point[0];
						tmp_params.max_x.y = point[1];
					}
					if(tmp_params.min_x.x > point[0]){
						tmp_params.min_x.x = point[0];
						tmp_params.min_x.y = point[1];
					}
					if(tmp_params.max_y.y < point[1]){
						tmp_params.max_y.x = point[0];
						tmp_params.max_y.y = point[1];
					}
					if(tmp_params.min_y.y > point[1]){
						tmp_params.min_y.x = point[0];
						tmp_params.min_y.y = point[1];
					}

				}
			}
			tmp_params.area = fabs((tmp_params.max_x.x-tmp_params.min_x.x)*(tmp_params.max_y.y-tmp_params.min_y.y));
			params.push_back(tmp_params);
		}
	}

	bool MatchIntersect(LOCAL_PLANE &Surface2, LOCAL_PLANE &Surface1, M_POSE_PLANE &pPose, INTERSECTION_MATCH &intersections){
		f_mec cMec;
		Vector3d T_final;
		Matrix3d R_final,Iref;
		bool res_intersect;
		double intersection_scorex, intersection_scorey;

		BOUNDARY_PLANE_PARAMS tmp_params1, tmp_params2;
		for(int i = 0; i < 3; i++){
			for(int j = 0; j < 3; j++){
				cMec.globRot(i,j) = pPose.m_Rot(i,j);
			}
			cMec.globTran(i) = pPose.m_X(i);
		}

		Iref = Surface1.m_Pose.m_Rot.inverse();
		R_final = Iref*cMec.globRot*Surface2.m_Pose.m_Rot;
		T_final = Iref*(cMec.globRot*Surface2.m_Pose.m_X+cMec.globTran-Surface1.m_Pose.m_X);
		find_plane_boundaries(true,R_final, T_final, Surface2, cMec.params2);
		find_plane_boundaries(false,R_final, T_final, Surface1, cMec.params1);
		for(int z=0; z<cMec.params2.size();z++){
			tmp_params1 = cMec.params2[z];
			for(int k=0; k<cMec.params1.size();k++){
				tmp_params2 = cMec.params1[k];
				res_intersect = check_interval_intersection(tmp_params1.max_x.x,tmp_params1.min_x.x,tmp_params2.max_x.x,tmp_params2.min_x.x,intersection_scorex);
				intersections.int_x = intersection_scorex;
				if(res_intersect){
					res_intersect = check_interval_intersection(tmp_params1.max_y.y,tmp_params1.min_y.y,tmp_params2.max_y.y,tmp_params2.min_y.y,intersection_scorey);
					intersections.int_y = intersection_scorey;
				}
				if(res_intersect)
					return true;
			}
		}
		return false;
	}

	void set_glob_rot_tranM(int id1, int id2, M_POSE_PLANE &Pose_rel){
		SE3Mat pose;
		slam_trajectory.PoseBetweenStates(local_models[id1].model_agent_id,local_models[id2].model_agent_id,local_models[id1].agent_id,local_models[id2].agent_id,pose);

		Pose_rel.Reset();
		for(int i = 0; i < 3; i++){
			for (int j = 0; j<3; j++){
				Pose_rel.m_Rot(i,j) = pose(i,j);
			}
			Pose_rel.m_X(i,0)=pose(i,3)*1000.0;
		}
		Pose_rel.UpdatePTRLL();
		Pose_rel.m_sa = sin(Pose_rel.m_Alpha);
		Pose_rel.m_ca = cos(Pose_rel.m_Alpha);
	}

#ifdef USE_VTK
	bool SurfaceToVTKPolygon(int id, int id1, vector<int> &global_planes_vec){
		bool inserted;
		int num_points, total_points = 0;
		double x,y,z;
		char buffer[100];

		vtkSmartPointer<vtkCellArray> polygons =vtkSmartPointer<vtkCellArray>::New();
		vtkSmartPointer<vtkPoints> points =vtkSmartPointer<vtkPoints>::New();

		inserted = false;

		for (int i = 0; i < global_planes_vec.size(); i++){
			for(int brC = 0; brC < global_planes[global_planes_vec[i]].t_plane.contours.size(); brC++){
				if (global_planes[global_planes_vec[i]].t_plane.contours[brC].bHole){
					continue;
				}
				num_points = 0;
				for(int brP = 0; brP < global_planes[global_planes_vec[i]].t_plane.contours[brC].P3D.size(); brP++){
					x = global_planes[global_planes_vec[i]].t_plane.contours[brC].P3D[brP](0);
					y = global_planes[global_planes_vec[i]].t_plane.contours[brC].P3D[brP](1);
					z = global_planes[global_planes_vec[i]].t_plane.contours[brC].P3D[brP](2);
					points->InsertNextPoint(z/1000.0,-x/1000.0,-y/1000.0);
					num_points++;
				}
				if(num_points>0){
					vtkSmartPointer<vtkPolygon> polygon =vtkSmartPointer<vtkPolygon>::New();
					polygon->GetPointIds()->SetNumberOfIds(num_points); //make a quad

					for(int j=0; j<num_points;j++){
						polygon->GetPointIds()->SetId(j,j+total_points);
					}
					inserted = true;
					polygons->InsertNextCell(polygon);
					total_points = total_points + num_points;
				}
			}
		}
		if(inserted){
			vtkSmartPointer<vtkPolyData> polygonPolyData =vtkSmartPointer<vtkPolyData>::New();
			vtkSmartPointer<vtkTriangleFilter> triFilter= vtkSmartPointer<vtkTriangleFilter>::New();
			vtkSmartPointer<vtkSTLWriter> stlWriter = vtkSmartPointer<vtkSTLWriter>::New();
			vtkSmartPointer<vtkPolyDataNormals> normalGenerator = vtkSmartPointer<vtkPolyDataNormals>::New();

			polygonPolyData->SetPoints(points);
			polygonPolyData->SetPolys(polygons);

			triFilter->SetInputData(polygonPolyData);
			triFilter->PassVertsOn();
			triFilter->Update();

			stlWriter->SetFileTypeToBinary();
			snprintf(buffer, sizeof(buffer), "/home/kruno/map_ws/src/SLAM_git/cooperation_server/models/%d_%d_mod.stl",id,id1);
			stlWriter->SetFileName(buffer);
			stlWriter->SetInputData(triFilter->GetOutput());
			stlWriter->Write();

			normalGenerator->SetInputData(polygonPolyData);
			normalGenerator->SetComputePointNormals(1);
			normalGenerator->ComputePointNormalsOn();
			normalGenerator->ComputeCellNormalsOff();
			normalGenerator->SetFlipNormals(1);

			normalGenerator->Update();

			polygonPolyData = normalGenerator->GetOutput();

			triFilter->SetInputData(polygonPolyData);
			triFilter->PassVertsOn();
			triFilter->Update();
			stlWriter->SetFileTypeToBinary();
			snprintf(buffer, sizeof(buffer), "/home/josip/map_ws/src/SLAM_git/cooperation_server/models/%d_%d_mod.stl",id,id1+1);
			stlWriter->SetFileName(buffer);
			stlWriter->SetInputData(triFilter->GetOutput());
			stlWriter->Write();
		}
		return inserted;

	}

	void PrepareMarkerSTL(visualization_msgs::Marker &marker, int id, int id1, float r_color, float g_color, float b_color){
		char buffer[100];
		marker.color.r = r_color; marker.color.g = g_color; marker.color.b = b_color; marker.color.a = 1.0;
		snprintf(buffer, sizeof(buffer), "package://cooperation_server/models/%d_%d_mod.stl",id,id1);
		marker.id = id1;
		marker.lifetime = ros::Duration();
		marker.mesh_resource = buffer;
		marker.header.stamp = ros::Time::now();
	}

	void send_map_rviz(int id){
		roof_surfaces.clear(); floor_surfaces.clear(); wall_surfaces.clear();
		for (int i = 0; i < global_planes.size(); i++){
			if(global_planes[i].global){
				if(global_planes[i].map_params.roof){
					roof_surfaces.push_back(i);
				}
				if(global_planes[i].map_params.wall){
					wall_surfaces.push_back(i);
				}
				if(global_planes[i].map_params.floor){
					floor_surfaces.push_back(i);
				}
			}
		}
		bool draw_wall = SurfaceToVTKPolygon(id, 0, wall_surfaces);

		if(draw_wall){
			PrepareMarkerSTL(surface_marker, broj_pozvanih, 0, 0.0,0.0,1.0);
			PrepareMarkerSTL(surface_marker2, broj_pozvanih, 1, 0.0,0.0,1.0);
		}
		/*bool draw_wall_local = Local_SurfaceToVTKPolygon(id, 6);
		if(draw_wall_local){
			ros::Rate r(100);
			for(int i=0; i<2; i++){
				PrepareMarkerSTL(surface_marker, broj_pozvanih, 6+i, 0.5,1.0,0.0);
				marker_pub_wall_local.publish(surface_marker);
				r.sleep();
			}
		}*/
		/*bool draw_floor = SurfaceToVTKPolygon(id, 2, floor_surfaces);
		if(draw_floor){
			ros::Rate r(100);
			for(int i=0; i<2; i++){
				PrepareMarkerSTL(surface_marker, broj_pozvanih, 2+i, 0.0,0.0,1.0);
				marker_pub_floor.publish(surface_marker);
				r.sleep();
			}
		}
		bool draw_roof = SurfaceToVTKPolygon(id, 4, roof_surfaces);
		if(draw_roof){
			ros::Rate r(100);
			for(int i=0; i<2; i++){
				PrepareMarkerSTL(surface_marker, broj_pozvanih, 4+i, 1.0,0.0,0.0);
				marker_pub_roof.publish(surface_marker);
				r.sleep();
			}
		}*/
	}
#endif


	~PlanarMapping() {
		printf("Shutting Down\n");
	}

};
