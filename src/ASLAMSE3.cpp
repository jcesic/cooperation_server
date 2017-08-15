#include "ASLAMSE3.h"
#include <stdio.h>

using namespace std;

void a_slam::init(double AS_max_euclidean_dist, int AS_min_index_diff, double angleScale){
	max_euclidian_distance = AS_max_euclidean_dist;
	min_index_diff=AS_min_index_diff;
	angle_scale = angleScale;
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

/*double a_slam::get_distance(geometry_msgs::PoseStamped& pose1, geometry_msgs::PoseStamped& pose2){
	return 1000*sqrt(pow((pose1.pose.position.x-pose2.pose.position.x),2)+pow((pose1.pose.position.y-pose2.pose.position.y),2)+pow((pose1.pose.position.z-pose2.pose.position.z),2));
}*/

double a_slam::get_distance(StateVector& x1, StateVector& x2){
	SE3Mat X1 = SEMap.exp(x1);
	SE3Mat X2 = SEMap.exp(x2);

	SE3Mat dT = X1.inverse()*X2;
	double eTrans = sqrt(pow(dT(0,3),2)+pow(dT(1,3),2)+pow(dT(2,3),2));
	double eRot = acos((dT.trace()/2)-1);
	return (eTrans + angle_scale*eRot);
}

loop_path a_slam::pathFind(int finish_id, vector<StateVector> &slam_trajectory){
	priority_queue<node> pq[2]; // list of open (not-yet-tried) nodes
	node* n0;
	node* m0;
	loop_path final_loop;

	int i, id, new_id, pqi;

	int size = slam_trajectory.size();
	int start_id = size - 1;
	int *open_nodes_map = new int[size];
	int *closed_nodes_map = new int[size];
	int *dir_map = new int[size];

	SE3Mat XStart = SEMap.exp(slam_trajectory[start_id]);
	SE3Mat XFinish = SEMap.exp(slam_trajectory[finish_id]);
	SE3Mat new_X, tmpX;

	pqi=0;
	for(i=0;i<size;i++){
		closed_nodes_map[i]=0;
		open_nodes_map[i]=0;
	}
	final_loop.states_path.clear();
	// create the start node and push into list of open nodes
	n0=new node(XStart, 0, 0, start_id, angle_scale);
	n0->updatePriority(XFinish,0);
	pq[pqi].push(*n0);
	open_nodes_map[start_id]=n0->getPriority(); // mark it on the open nodes map
	// A* search
	while(!pq[pqi].empty()){ 	// get the current node with the highest priority from the list of open nodes
		n0=new node( pq[pqi].top().getX(), pq[pqi].top().getLevel(), pq[pqi].top().getPriority(), pq[pqi].top().getid(), angle_scale);
		tmpX = n0->getX();
		id=n0->getid();
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
				new_X = SEMap.exp(slam_trajectory[i]);
				new_id=i;
				m0=new node(new_X, n0->getLevel(), n0->getPriority(),new_id,angle_scale);
				m0->nextLevel(tmpX,razmak);
				m0->updatePriority(XFinish, 0);
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

priority_queue<node_blizu> a_slam::nadi_blizu(vector<StateVector> &slam_trajectory){
	node_blizu *n0;
	priority_queue<node_blizu> vrati;
	int num_states = slam_trajectory.size();
	double distance;
	for (int x=0;x<(num_states-min_index_diff);x++){
		distance = get_distance(slam_trajectory[num_states-1],slam_trajectory[x]);
		if (distance<max_euclidian_distance){
			n0 =new node_blizu(distance,x,num_states-1);
			vrati.push(*n0);
			delete n0;
		}
	}
	return vrati;
}

void a_slam::find_between(vector<StateVector> &slam_trajectory, int id_start, int id_end, vector<block_marg> &moguci_cvorovi){
	int num_states = slam_trajectory.size();
	double distance;
	bool first = true;
	block_marg tmp_block;
	int prev_y;
	printf("finding between: %d %d %ld\n",id_start,id_end,slam_trajectory.size());
	for (int y=id_start;y<=id_end;y++){
		for (int x=0;x<num_states;x++){
			if(abs(x-y)>min_index_diff){
				distance = get_distance(slam_trajectory[y],slam_trajectory[x]);
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

node::node(SE3Mat X, int d, int p, int id, double angleScale){
	nX = X; level=d; priority=p; state_id=id, yaw_SCALE=angleScale;
}

void node::updatePriority(SE3Mat X, int razmak){
	int d = estimate(X, razmak);
	priority = level + d; //A*
}

void node::nextLevel(SE3Mat X, int razmak){
	int d = estimate(X, razmak);
	level = level + d;

}

int node::estimate(SE3Mat X, int razmak){
	SE3Mat dT = nX.inverse()*X;
	double distance = sqrt(pow(X(0,3)-nX(0,3),2)+pow(X(1,3)-nX(1,3),2)+pow(X(2,3)-nX(2,3),2));
	double eTrans = sqrt(pow(dT(0,3),2)+pow(dT(1,3),2)+pow(dT(2,3),2));
	double eRot = acos((dT.trace()/2)-1);
	int d = (int)(100*(eTrans + yaw_SCALE*eRot));
	return d;
}

node_blizu::node_blizu(int p, int id, int close_id){
	priority=p;
	state_id=id;
	close_state = close_id;
}


