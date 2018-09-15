#include "astar.h"

Astar::Astar(pGraph graph_){
  _graph = graph_;
}

vector<node> Astar::getSuccessors(node parent){
    vector<node> successors = *(new vector<node>);
    std::vector<int> successors_id = parent.g_node.list_id;
    for(int i=0; i < successors_id.size();i++){
      if(!alreadyVisited(successors_id[i])){
	node child;
	child.g_node = _graph.at(successors_id[i]);
	child.g = parent.g + getDistance(child, parent);
	successors.push_back(child);
      }
    }
    return successors;
}

double Astar::getDistance(node& p1, node& p2){
    return sqrt(pow( p1.g_node.pos_x - p2.g_node.pos_x, 2) + pow( p1.g_node.pos_y - p2.g_node.pos_y, 2));
}


void Astar::addToOpenList(node n){
  
  for(list<node>::iterator it = _open_list.begin(); it != _open_list.end(); it++){
    if(it->g_node.id == n.g_node.id && it->f <= n.f){
      return;
    }
    if(it->f >= n.f){
      _open_list.insert(it,n);
      return;
    }
  }
  _open_list.push_back(n);
}

bool Astar::alreadyVisited(int id){
  return find(_closed_list.begin(), _closed_list.end(),id) != _closed_list.end();
}




std::vector<int> Astar::compute(int start_, int goal_){
  if( start_ == -1 || goal_ == -1){
    return _path;
  }
  node start = node(_graph.at(start_));
  node goal  = node(_graph.at(goal_));
  

  _open_list.push_back(start);

  while (  _open_list.size() != 0){
    //wv take the first element and delete it from _open_list
    node current = _open_list.front();	
    _open_list.pop_front();
    
    //wv get the successors of th current node
    vector<node> successors = getSuccessors(current); 
    list<node>::iterator i;

    for (int i=0; i< successors.size(); i++){
      
	node* successor_i = new node;
	*successor_i = successors[i];
	
	node* tmp_node = new node;
	*tmp_node = current;

	//wv update the current successor
	successor_i->h = getDistance(*successor_i,goal);
	successor_i->f = successor_i->h + successor_i->g;
	successor_i->parent = tmp_node;
	
	//wv if successor has distance 0 from the goal
	if(successor_i->h == 0){
	  //inserting the goal
	  _path.insert(_path.begin(), successor_i->g_node.id);
	  node* last = successor_i->parent;
	  
	  //wv go back until start node
	  while(last->g_node.id != start_){
	    
	    _path.insert(_path.begin(),last->g_node.id);
	    last = last->parent;
	  }
	  //inserting the start
	  _path.insert(_path.begin(),last->g_node.id);
	  return _path;
	}

	//wv add the current node inside the open list
	addToOpenList(*successor_i);
    }
    
    // add the current node to the closed list , this means that it was already expanded
    _closed_list.push_back(current.g_node.id);
    if(_open_list.size() == 0 ) {
      return _path;
    }
  }
}




