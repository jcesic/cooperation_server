#include <queue>
#include <string>
#include <math.h>
#include <stdio.h>
#include <vector>
#include <new>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Eigen> // whole Eigen matrix library (dense and sparse)
using namespace Eigen;
using namespace std;
#include <tf/tf.h>

typedef Eigen::Matrix<double,6,1> StateVector;

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
    int xPos;
    int yPos;
    int level;
    int priority;
    int state_id;
    double yawPos;
    double yaw_SCALE;
    public:
        node(int xp, int yp, double yaw, int d, int p, int id);
        int getxPos() const {return xPos;}
        int getyPos() const {return yPos;}
        double getYaw() const {return yawPos;}

        int getLevel() const {return level;}
        int getPriority() const {return priority;}
        int getid() const {return state_id;}
        void updatePriority(int xDest, int yDest, double yawDest, int razmak);
        void nextLevel(int x, int y, double yaw, int razmak);
        int estimate(int xDest, int yDest, double yawDest, int razmak);
};

bool operator<(const node_blizu & a, const node_blizu & b);

class a_slam{
	MatrixXi G;
	public:
		double initial_pose_x,initial_pose_y,initial_pose_theta;
		bool first_augment;
		int max_euclidian_distance;
		int min_topological_distance;
		int min_index_diff;
		int dimPose;
		void init(int e_distance_active_slam, int MIN_INDEX_DISTANCE_THRESHOLD);
		int prvi;
		void dodaj_augment();
		void dodaj_update(int index1, int index2);
		void ispis();
		double get_yaw(double w, double z);
		double get_distance(geometry_msgs::PoseStamped& pose1, geometry_msgs::PoseStamped& pose2);
		vector<int> ubaci_ispis();
		vector<double> ubaci_stanja(VectorXd X);
		int get_size();
		int get_cols();
		int get_rows();
		loop_path pathFind(int finishid, nav_msgs::Path &slam_path);

		priority_queue<node_blizu> nadi_blizu(nav_msgs::Path &slam_path);
		void find_between(nav_msgs::Path &slam_path, int id_start, int id_end, vector<block_marg> &moguci_cvorovi);

		void ispis_file();
		priority_queue<node_blizu> moguci_cvorovi;
};







