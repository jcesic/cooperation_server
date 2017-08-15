#include "ASLAM.h"
#include <stdio.h>

using namespace std;

void a_slam::init(int AS_max_euclidean_dist, int AS_min_index_diff){
	first_augment = true;
	max_euclidian_distance = AS_max_euclidean_dist;
	//min_topological_distance = t_distance_active_slam;
	initial_pose_x = 0;
	initial_pose_y = 0;
	initial_pose_theta = 0;
	dimPose =4;
	min_index_diff=AS_min_index_diff;

}

void a_slam::dodaj_augment() {
	if(first_augment){
		G.resize(1,1);
		G(0,0)=1;
		first_augment = false;
	}else{
		int i = G.rows();
		G.conservativeResize(i+1,i+1);
		G(i,i)=1;
		G(i-1,i)=1;
		G(i,i-1)=1;
	}
}

void a_slam::dodaj_update(int index1, int index2) {
	G(index1,index2)=1;
	G(index2,index1)=1;
}

vector<int> a_slam::ubaci_ispis() {
	int i;
	int j;
	vector<int> vrati;
	for(i=0;i<G.cols();i++){
		for(j=0;j<G.rows();j++){
			vrati.push_back(G(i,j));
		}
	}
	return vrati;
}

vector<double> a_slam::ubaci_stanja(VectorXd X){
	vector<double> vrati;
	for (int i =0;i<G.cols();i++){
		VectorXd x_current = X.block(i*dimPose, 0, dimPose, 1);
		vrati.push_back(x_current(0));
		vrati.push_back(x_current(1));
	}
	return vrati;
}

int a_slam::get_size(){
	return G.cols()*G.rows();
}
int a_slam::get_cols(){
	return G.cols();
}
int a_slam::get_rows(){
	return G.rows();
}

double a_slam::get_yaw(double w, double z){
	tf::Quaternion quat; quat.setW(w);quat.setZ(z);quat.setX(0);quat.setY(0);
	return tf::getYaw(quat);
}

double a_slam::get_distance(geometry_msgs::PoseStamped& pose1, geometry_msgs::PoseStamped& pose2){
	return 1000*sqrt(pow((pose1.pose.position.x-pose2.pose.position.x),2)+pow((pose1.pose.position.y-pose2.pose.position.y),2)+pow((pose1.pose.position.z-pose2.pose.position.z),2));
}

loop_path a_slam::pathFind(int finish_id, nav_msgs::Path &slam_path){
	priority_queue<node> pq[2]; // list of open (not-yet-tried) nodes
	node* n0;
	node* m0;
	loop_path final_loop;

	int i, x, y, new_x, new_y, id, new_id, pqi;
	double new_yaw,yaw;

	int size = slam_path.poses.size();
	int start_id = size - 1;
	int *open_nodes_map = new int[size];
	int *closed_nodes_map = new int[size];
	int *dir_map = new int[size];

	int xStart = slam_path.poses[start_id].pose.position.x;
	int yStart = slam_path.poses[start_id].pose.position.y;
	double yaw_start = tf::getYaw(slam_path.poses[start_id].pose.orientation);

	int xFinish = slam_path.poses[finish_id].pose.position.x;
	int yFinish = slam_path.poses[finish_id].pose.position.y;
	double yaw_finish = tf::getYaw(slam_path.poses[finish_id].pose.orientation);

	pqi=0;
	for(y=0;y<size;y++){
		closed_nodes_map[y]=0;
		open_nodes_map[y]=0;
	}
	final_loop.states_path.clear();
	// create the start node and push into list of open nodes
	n0=new node(xStart, yStart, yaw_start, 0, 0, start_id);
	n0->updatePriority(xFinish, yFinish, yaw_finish,0);
	pq[pqi].push(*n0);
	open_nodes_map[start_id]=n0->getPriority(); // mark it on the open nodes map
	// A* search
	while(!pq[pqi].empty()){ 	// get the current node with the highest priority from the list of open nodes
		n0=new node( pq[pqi].top().getxPos(), pq[pqi].top().getyPos(), pq[pqi].top().getYaw(), pq[pqi].top().getLevel(), pq[pqi].top().getPriority(), pq[pqi].top().getid());
		x=n0->getxPos(); y=n0->getyPos(); id=n0->getid(); yaw=n0->getYaw();
		pq[pqi].pop(); // remove the node from the open list
		open_nodes_map[id]=0;
		// mark it on the closed nodes map
		closed_nodes_map[id]=1;
		// quit searching when the goal state is reached
		if(id==finish_id){
			// generate the path from finish to star by following the directions
			while(!(id==start_id)){
				final_loop.states_path.push_back(dir_map[id]);
				id=dir_map[id];
			}
			final_loop.start_path=start_id;
			final_loop.end_path=finish_id;
			final_loop.top_distance = n0->getLevel();
			// garbage collection
			delete n0;
			delete [] open_nodes_map;
			delete [] closed_nodes_map;
			delete [] dir_map;
			// empty the leftover nodes
			while(!pq[pqi].empty())
				pq[pqi].pop();
			return final_loop;
		}
		// generate moves (child nodes) in all possible directions
		for(i=0;i<G.cols();i++){
			if(G(i,id)==1&&i!=id&&!closed_nodes_map[i]){
				int razmak = 1;
				if(abs(i-id)>1)
					razmak=0;
				new_x = slam_path.poses[i].pose.position.x;
				new_y = slam_path.poses[i].pose.position.y;
				new_yaw = tf::getYaw(slam_path.poses[i].pose.orientation);

				new_id=i;
				m0=new node(new_x, new_y, new_yaw, n0->getLevel(), n0->getPriority(),new_id);
				m0->nextLevel(x,y,yaw,razmak);
				m0->updatePriority(xFinish, yFinish, yaw_finish, 0);
				// if it is not in the open list then add into that
				if(open_nodes_map[new_id]==0){
					open_nodes_map[new_id]=m0->getPriority();
					pq[pqi].push(*m0);
					// mark its parent node direction
					dir_map[new_id]=id;
				}else if(open_nodes_map[new_id]>m0->getPriority()){
					// update the priority info
					open_nodes_map[new_id]=m0->getPriority();
					// update the parent direction info
					dir_map[new_id]=id;
					// replace the node by emptying one pq to the other one except the node to be replaced will be ignored and the new node will be pushed in instead
					while(!(pq[pqi].top().getid()==new_id)){
						pq[1-pqi].push(pq[pqi].top());
						pq[pqi].pop();
					}
					pq[pqi].pop(); // remove the wanted node empty the larger size pq to the smaller one
					if(pq[pqi].size()>pq[1-pqi].size())
						pqi=1-pqi;
					while(!pq[pqi].empty()){
						pq[1-pqi].push(pq[pqi].top());
						pq[pqi].pop();
					}
					pqi=1-pqi;
					pq[pqi].push(*m0); // add the better node instead
				}
				else{
					delete m0; // garbage collection
				}
			}
		}
		delete n0; // garbage collection
	}
	delete [] open_nodes_map;
	delete [] closed_nodes_map;
	delete [] dir_map;
	return final_loop; // no route found

}

priority_queue<node_blizu> a_slam::nadi_blizu(nav_msgs::Path &slam_path){
	node_blizu *n0;
	priority_queue<node_blizu> vrati;
	int num_states = slam_path.poses.size();
	double distance;
	for (int x=0;x<(num_states-min_index_diff);x++){
		distance = get_distance(slam_path.poses[num_states-1],slam_path.poses[x]);
		if (distance<max_euclidian_distance){
			n0 =new node_blizu(distance,x,num_states-1);
			vrati.push(*n0);
			delete n0;
		}
	}
	return vrati;
}

void a_slam::find_between(nav_msgs::Path &slam_path, int id_start, int id_end, vector<block_marg> &moguci_cvorovi){
	int num_states = slam_path.poses.size();
	double distance;
	bool first = true;
	block_marg tmp_block;
	int prev_y;
	for (int y=id_start;y<id_end;y++){
		for (int x=0;x<num_states;x++){
			if(abs(x-y)>min_index_diff){
				distance = get_distance(slam_path.poses[y],slam_path.poses[x]);
				if (distance<max_euclidian_distance){
					if(first){
						tmp_block.begin=y;
						first = false;
					}else if(y-prev_y>1){
						tmp_block.end = prev_y;
						moguci_cvorovi.push_back(tmp_block);
						tmp_block.begin = y;
					}
					prev_y = y;
					break;
				}
			}
		}
	}
	if(!first){
		tmp_block.end = prev_y;
		moguci_cvorovi.push_back(tmp_block);
	}
}

bool operator<(const node_blizu & a, const node_blizu & b)
{
	return a.getPriority() > b.getPriority();
}

bool operator<(const node & a, const node & b)
{
	return a.getPriority() > b.getPriority();
}

node::node(int x, int y, double yaw, int d, int p, int id){
	xPos=x; yPos=y; yawPos = yaw; level=d; priority=p; state_id=id, yaw_SCALE=0.0;
}

void node::updatePriority(int xDest, int yDest, double yawDest, int razmak){
	priority = level + estimate(xDest, yDest, yawDest, razmak); //A*
}

void node::nextLevel(int x, int y, double yaw, int razmak){
	level = level + estimate(x, y, yaw, razmak);
}

int node::estimate(int xDest, int yDest, double yawDest, int razmak){
	int xd, yd, d;
	double yawd;

	yawd = (yawDest-yawPos);
	if (yawd > PI) {
		yawd = yawd - _2PI;
	}else if(yawd < -PI) {
		yawd= yawd + _2PI;
	}
	xd=xDest-xPos;
	yd=yDest-yPos;
	d=(int)(sqrt(xd*xd+yd*yd))+(int)(fabs(razmak*yawd*yaw_SCALE));
	return(d);
}

node_blizu::node_blizu(int p, int id, int close_id){
	priority=p;
	state_id=id;
	close_state = close_id;
}

