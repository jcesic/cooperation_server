#include <queue>
#include <string>
#include <math.h>
#include <stdio.h>
#include <vector>
#include <new>

#include "Common.h"
#include "SE3Mappings.h"

#include <Eigen/Eigen> // whole Eigen matrix library (dense and sparse)
using namespace Eigen;
using namespace std;

//typedef Eigen::Matrix<double,6,1> StateVector;

#ifndef _2PI
#define _2PI	6.28318530717959
#endif
#ifndef PI
#define PI 3.14159265
#endif

struct block_marg{
	int begin,end;
};

struct loop_path{
	vector<int> states_path;
	int start_path;
	int end_path;
	int top_distance;
};

class node_blizu{
    int priority;  // smaller: higher priority
    int state_id;
    int close_state;
    public:
        node_blizu(int p, int id, int close_state);
        int getPriority() const {return priority;}
        int getid() const {return state_id;}
        int getClose() const {return close_state;}
};

class node{
    int level;
    int priority;
    int state_id;
    SE3Mat nX;
    double yaw_SCALE;
    public:
        node(SE3Mat X, int d, int p, int id, double angle_scale);
        int getLevel() const {return level;}
        int getPriority() const {return priority;}
        int getid() const {return state_id;}
        void updatePriority(SE3Mat X, int razmak);
        void nextLevel(SE3Mat X, int razmak);
        int estimate(SE3Mat X, int razmak);
        SE3Mat getX() const {return nX;};
};

bool operator<(const node_blizu & a, const node_blizu & b);

class a_slam{
	SE3Mappings SEMap;

	public:
		Eigen::MatrixXi G;
		double angle_scale;
		int  min_index_diff;
		double max_euclidian_distance;
		void init(double AS_max_euclidean_dist, int AS_min_index_diff, double angleScale);
		int get_size();
		int get_cols();
		int get_rows();
		double get_distance(StateVector& x1, StateVector& x2);

		loop_path pathFind(int finish_id, vector<StateVector> &slam_trajectory);
		vector<int> ubaci_ispis();
		vector<double> ubaci_stanja(VectorXd X);
		priority_queue<node_blizu> nadi_blizu(vector<StateVector> &slam_trajectory);
		void find_between(vector<StateVector> &slam_trajectory, int id_start, int id_end, vector<block_marg> &moguci_cvorovi);
};







