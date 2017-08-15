#include "Kruskal.h"

void CLT_graph::Init(int V){
    this->V=V;
    CLT_Subset tmp_subset;
    for(int i=0;i<V;i++){
        tmp_subset.parent=i;
        tmp_subset.rank=0;
        subset.push_back(tmp_subset);
    }
}

void CLT_graph::addEdge(int v,int w,double wt){
	CLT_Edge temp;
    temp.src=v;
    temp.dest=w;
    temp.wt=wt;
    edge.push_back(temp);
}

void CLT_graph::setEdgeWt(int k, double wt){
	edge[k].wt = wt;
}
 
int CLT_graph::find(int i){
    if(subset[i].parent!=i)
        subset[i].parent=find(subset[i].parent);
    return subset[i].parent;
}
 
void CLT_graph::Union(int x,int y){
    int xroot=find(x);
    int yroot=find(y);
    if(subset[xroot].rank<subset[yroot].rank){
        subset[xroot].parent=yroot;
    }else if(subset[yroot].rank<subset[xroot].rank){
        subset[yroot].parent=xroot;
    }else{
        subset[yroot].parent=xroot;
        subset[xroot].rank++;
    }
}
 
void CLT_graph::MST(){
    int e=0;
    sort(edge.begin(),edge.end());
    vector<CLT_Edge>::iterator i;
    for(i=edge.begin();i!=edge.end() && e!=V-1;i++){
        int x=find(i->src);
        int y=find(i->dest);
        if(x!=y){
            result.push_back(*i);
        	e++;
            Union(x,y);
        }
    }
}
