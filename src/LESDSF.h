#define EIGEN_SUPERLU_SUPPORT 1
#define EIGEN_MATRIXBASE_PLUGIN "/home/josip/catkin_ws/src/cooperation_server/src/EigenMatrixBaseAddons.h"
#define EIGEN_SPARSEMATRIX_PLUGIN "/home/josip/catkin_ws/src/cooperation_server/src/EigenSparseMatrixBaseAddons.h"
#include <fstream>
#include <iostream>
#include <Eigen/Eigen> // whole Eigen matrix library (dense and sparse)

#include "Common.h"
#include "ProcessDefinitions.h"
#include "SE3Mappings.h"
#include "CommonStruct.h"
#include "stopwatch.h"
#include "Kruskal.h"

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>

#include <ros/ros.h>
typedef Eigen::Triplet<double> EigenT;

using namespace Eigen;
using namespace std;

struct state_loops{
	int ID;
	vector<int> loops;
};

struct gain_ret{
	double Inf,DLii,IDLjj,DLij;
};

class LESDSF_class{
public:
	StateMat Pxx, F, Q, Qu_test; // linearized process model
	SE3Mat xtG;
	Eigen::SimplicialCholesky<SparseMat, Eigen::Upper> solver;
	VectorXd n,x,X;
	SparseMat L;
	vector<state_loops> states_loops;
	state_loops tmp_state_data;
	StateVector xt;
	int dimPose,num_p,marg_id,total_updates;
	double minQnoise,minQnoiseLoop,t_glob_solve;
	SE3Mappings SEMap;
	double Kssx,Kssy,Kssz,Kalpha,Kbeta,Ktheta;
	void Init();
	void MotionModel(StateVector& Om);
	void write_to_shared(int agent_id);
	void AugmentState();
	void Predict();
	void MarginalizeO(int state_id, bool no_sparse);
	gain_ret InfGainLxx(int src, int dest, SparseMat &LXS);
	gain_ret InfGainRS(int src, int dest, SparseMat &LXS, SparseMat &RS, SparseMat &LZS);

	void MarginalizeS(bool recover_state, int state_IDm, int state_IDn, vector<StateVector> &slam_trajectory);
	void Update(int id1, int id2, SE3Mat& zG, StateMat& R, vector<StateVector> &slam_trajectory);
	void RecoverState();
	void getRelPose(int id1, int id2,StateVector& vPose);
	void RelativeCov(int id1, int id2, MatrixXd& P_12 );
	void StateCov(int id, StateMat& P);
	int num_poses, num_angles;
};
