#include <iostream>
#include <vector>
#include <algorithm>
#include<bits/stdc++.h>
using namespace std;
 
struct CLT_Edge{
    int src;
    int dest;
    double wt;
    bool operator<(const CLT_Edge &rhs) const { return wt > rhs.wt; }
};
 
struct CLT_Subset{
    int parent;
    int rank;
};

class CLT_graph{
    vector<CLT_Subset> subset;
    int find(int i);
    void Union(int x,int y);
public:
    int V;
    vector<CLT_Edge> edge;
    vector<CLT_Edge> result;
    void Init(int V);
    void addEdge(int v,int w,double wt);
    void setEdgeWt(int k, double wt);
    void MST();
};
