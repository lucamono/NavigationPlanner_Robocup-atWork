#ifndef ASTAR_H
#define ASTAR_H

#include <iostream>
#include <vector>
#include <list>
#include "graph.h"



using namespace std;
struct node {
    graphNode g_node;
    node* parent;

    double f, g, h;
    
    node(){};
    node(graphNode& n){
      g_node = n;
      f=0;
      g=0;
      h=0;
    }
};

class Astar {

private:
  pGraph _graph;
  list<node> _open_list;
  vector<int> _closed_list;
  vector<int> _path;

private:  
  vector<node> getSuccessors(node n);
  double getDistance(node& p1, node& p2);
  void addToOpenList(node n);
  bool alreadyVisited(int id);



public:
    Astar(pGraph graph_);
    std::vector<int> compute(int start, int goal);

};


#endif
