/*
#ifndef DEBUG_HEUR
#define DEBUG_HEUR
#endif
*/
#ifdef EGRAPH_MANAGER_H
template <typename HeuristicType>
EGraphManager<HeuristicType>::EGraphManager(std::vector<EGraphPtr> egraphs,
					    EGraphablePtr egraph_env,
					    std::vector<std::vector<EGraphHeuristicPtr> > egraph_heurs,
					    int numgoals, int numagents):
  egraph_env_(egraph_env), numgoals_(numgoals), numagents_(numagents){
  //numgoals_  = numgoals; 
  //  numagents_ = numagents; 
  
  params_.feedback_path = true;
  params_.update_stats = true;
 
  int i = 0;
  egraph_heurs_.resize(numagents_);
  egraphperagent_.resize(numagents_);
  TSP_allagents_.reserve(numagents_);
  for(int agent_i = 0; agent_i < numagents_; agent_i++){
    egraph_heurs_[agent_i].resize(numgoals_);
    egraphperagent_[agent_i] = egraphs[agent_i];
    Matrix TSP_agenti(pow(2, numgoals_));
    TSP_allagents_.push_back(TSP_agenti);
    for(int goal_i = 0; goal_i < numgoals_; goal_i++){
      egraph_heurs_[agent_i][goal_i] = egraph_heurs[agent_i][goal_i];
      egraph_heurs_[agent_i][goal_i]->setAgentId(agent_i);
      i++;
    }
  }
  //updateManager();
}

template <typename HeuristicType>
void EGraphManager<HeuristicType>::updateHeuristicGrids(const std::vector<std::vector<bool> >& grid){
  // set up a heuristic grid for each agent-goal pair
  for(int agent_i = 0; agent_i < numagents_; agent_i++){
    for(int goal_i = 0; goal_i < numgoals_; goal_i++){
      egraph_heurs_[agent_i][goal_i]->setGrid(grid);
      }
  }  
}

template <typename HeuristicType>
void EGraphManager<HeuristicType>::setEpsE(double epsE){  
  for(int agent_i = 0; agent_i < numagents_; agent_i++){
    for(int goal_i = 0; goal_i < numgoals_; goal_i++){
      egraph_heurs_[agent_i][goal_i]->setEpsE(epsE);
    }
  }
}

template <typename HeuristicType>
void EGraphManager<HeuristicType>::updateManager(){
  setGoal();
  initEGraph(true);
  /*
  // Allocate edgecosts_agent_i ( [#numgoals+1 x #numgoals+1] vector )
  TSPEdgecosts_.clear();

  for(int agent_i = 0; agent_i < numagents_; agent_i++){
  Matrix TSPEdgecosts_agent_i;
  for(int i = 0; i < numgoals_ + 1; i++){
  std::vector<int> row(numgoals_ + 1, 0);
  TSPEdgecosts_agent_i.push_back(row);
  }
  TSPEdgecosts_.push_back(TSPEdgecosts_agent_i);
  }*/
  precomputeTSPcosts();
}

template <typename HeuristicType>
void EGraphManager<HeuristicType>::precomputeTSPcosts(){
  unsigned int num_TSPs = pow(2, numgoals_); // power set of set of goals
  // For each agent,
  // compute distances between goals for each agent,
  // and solve the open-loop TSP starting at each goal
  for(int agent_i = 0; agent_i < numagents_; agent_i++){
    Matrix goalDistances;
    computeDistanceBetweenGoals(agent_i, goalDistances); 
    const int maxnumgoals = MAXNUMGOALS;
    for(unsigned long int TSP_i = 0; TSP_i < num_TSPs; TSP_i ++){
      std::bitset<maxnumgoals> TSP_i_vertices (TSP_i);
      std::vector<int> goalindices;
      // find indices of goals corresponding to this assignment
      for(int goal_i = 0; goal_i < numgoals_; goal_i++){
	if(TSP_i_vertices.test(maxnumgoals - goal_i -1))
	  goalindices.push_back(goal_i);
      }
      std::vector<int> costV;
      solveTSP(goalDistances, goalindices, costV);
      TSP_allagents_[agent_i][TSP_i] = costV;
    }
  }
} 

template <typename HeuristicType>
std::vector<int> EGraphManager<HeuristicType>::getHeuristic(int state_id){
  std::vector<int> heurs (1+numagents_,0);
  if(egraph_env_->isGoal(state_id)){
    return heurs;
  }

  ContState cont_state;
  egraph_env_->getCoord(state_id, cont_state); // TODO: We'd like getcoord to return pose, goalsvisited

#ifdef DEBUG_HEUR  
  SBPL_INFO("[egraphManager]: \nComputing heuristic for state %d", state_id);
  int i;
  for(i = 0; i < numagents_; i++)
    printf("Agent %d: (%f %f %f %d) ", i,
	   cont_state[4*i], cont_state[4*i+1],
	   cont_state[4*i+2], (int) cont_state[4*i+3]);
  printf("\n");
  for(i = 0; i < numagents_;i++)
    printf("Agent %d: (%d, %d) ", i, heur_coord[2*i], heur_coord[2*i+1]);
  printf("\nGoalsVisited:");
  for(int goal_i = 0; goal_i < numgoals_; goal_i++)
    printf("%d ", (int) cont_state[4*numagents_ + goal_i]);
  printf("\nActiveAgents:");
  for(int agent_i = 0; agent_i < numagents_; agent_i++)
    printf("%d ", (int) cont_state[4*numagents_ + numgoals_ + agent_i]);
  printf("\n\n");  
#endif  

  // find number and indices of active agents
  std::vector<int> activeAgents_indices;
  for(int i = 0; i < numagents_; i++)
    if(cont_state[i + numagents_*4 + numgoals_]){
      activeAgents_indices.push_back(i);
    }
  bruteforceHeuristic(cont_state, activeAgents_indices, heurs);
  return heurs;
}

template <typename HeuristicType>
void EGraphManager<HeuristicType>::getGoalCoord(int i, std::vector<int>& coord){
  // TODO: Terrible!
  coord.clear();
  coord.push_back(allgoals_coord_[2*i]);
  coord.push_back(allgoals_coord_[2*i+1]);
}


template <typename HeuristicType>
bool EGraphManager<HeuristicType>::setGoal(){
  allgoals_coord_.clear();
  egraph_env_->projectGoalToHeuristicSpace(allgoals_coord_);
  stats_.heuristic_computation_time  = 0;
  stats_.combo_time  = 0;
  stats_.shortest_path_time  = 0;
  stats_.get_direct_shortcut_time = 0;
  stats_.shortcut_time = 0;
  stats_.snap_time = 0;
  stats_.num_snaps = 0;
  //clock_t time = clock();
  //ROS_INFO("egraph heuristic setGoal time %f", double(clock()-time)/CLOCKS_PER_SEC);
  //snaps_.clear();
  //snap_combo_cache_.clear();
  for(int agent_i = 0; agent_i < numagents_; agent_i++)
    egraphperagent_[agent_i]->clearShortestPathCache();
  return true;
}

template <typename HeuristicType>
void EGraphManager<HeuristicType>::computeDistanceBetweenGoals(int agent_i, 
							       std::vector<std::vector<int> > &goalDistances){
  std::vector<int> start(2,0);
  for(int goal_i = 0; goal_i < numgoals_; goal_i++){
    start[0] = allgoals_coord_[2*goal_i];
    start[1] = allgoals_coord_[2*goal_i+1];
    for(int goal_j = goal_i+1; goal_j < numgoals_; goal_j++){
      int distance = egraph_heurs_[agent_i][goal_j]->getHeuristic(start);
      goalDistances[goal_i][goal_j] = distance;
      goalDistances[goal_j][goal_i] = distance;
      //SBPL_INFO("computeDistanceBetweenGoals: start (%d,%d) goal (%d,%d)", start[0], start[1], goal[0], goal[1]); 
      //SBPL_INFO("computeDistanceBetweenGoals: heur is %d", heur);
    }
  }
}


/* Prim allocation */
/*template <typename HeuristicType>
void EgraphManager<HeuristicType>::primAllocationHeuristic(std::vector<double>& cont_state,
							   std::vector<int>& activeAgents_indices, 
							   std::vector<int>& heurs){
  HeuristicType heur_coord;
  egraph_env_->projectToHeuristicSpace(cont_state, heur_coord);    
}
*/

/* ------------------------------------------------------------------------------------
 solve TSP with brute force techniques */
template <typename HeuristicType>
void EGraphManager<HeuristicType>::bruteforceHeuristic(std::vector<double>& cont_state,
						       std::vector<int>& activeAgents_indices,
						       std::vector<int>& heurs){

  HeuristicType heur_coord;
  HeuristicType heur_coord_agent(2,0);
  egraph_env_->projectToHeuristicSpace(cont_state, heur_coord);    

  int numActiveAgents = activeAgents_indices.size();
  // look at all possible assignments of goals to agents
  int numassignments = pow(numActiveAgents, numgoals_);
  std::vector<int> heur_allassignments(numassignments, 0);
  std::vector<int> heur_allagents(numActiveAgents, -1);
  std::vector<int> assignment(numgoals_);
  std::vector<std::vector<int> > heur_allagents_allassignments;
  
  // compute distance from each agent to all unvisited goals
  std::vector<std::vector<int> > distance_to_goal(numagents_);
  for(int agent_i = 0; agent_i < numagents_; agent_i++ ){
    distance_to_goal[agent_i].resize(numgoals_, -1);
    heur_coord_agent[0] = heur_coord[2*agent_i];
    heur_coord_agent[1] = heur_coord[2*agent_i+1];
    for(int goal_i = 0; goal_i < numgoals_; goal_i++){
      if(cont_state[4*numagents_ + goal_i] >= 0) // don't care about already visited goals
	continue;
      EGraphHeuristicPtr egraph_heur = egraph_heurs_[agent_i][goal_i];
      distance_to_goal[agent_i][goal_i] = egraph_heur->getHeuristic(heur_coord_agent);
    }
  }
  
  for(int i = 0; i < numassignments; i ++){
    int index = i;
    // assignment is index in base numactiveagents
    for(int goal_i = 0; goal_i < numgoals_; goal_i++){
      int activeagent_index = index % numActiveAgents;
      // if goal is already visited at this state, don't assign to agent
      if(cont_state[4*numagents_ + goal_i] >= 0)
	assignment[goal_i] = -1;
      else
	assignment[goal_i] = activeAgents_indices[activeagent_index];
      index = (int) index/numActiveAgents;
    }
    /*
      #ifdef DEBUG_HEUR    
      SBPL_INFO("egraphManager: Assignment is");
      for(int j = 0; j < (int)assignment.size(); j ++)
      SBPL_INFO("%d", assignment[j]);
      #endif
    */
    // for this assignment, compute heuristic for every agent
    for(int agent_i = 0; agent_i < numagents_; agent_i++){
      heur_coord_agent[0] = heur_coord[2*agent_i];
      heur_coord_agent[1] = heur_coord[2*agent_i + 1];
      heur_allagents[agent_i] = bruteforceHeuristicPerAgent(agent_i, 
							    heur_coord_agent, assignment, distance_to_goal[agent_i]);
      /*
	#ifdef DEBUG_HEUR
	SBPL_INFO("heur_coord_agent is (%d,%d). Heuristic is %d", heur_coord_agent[0], heur_coord_agent[1], heur_allagents[agent_i]);
	#endif
      */
    }
    //for this assignment, heuristic is the sum of heuristics for all agents
    heur_allagents_allassignments.push_back(heur_allagents);
    heur_allassignments[i] = std::accumulate(heur_allagents.begin(), heur_allagents.end(), 0);
  }
  // Admissible heuristic is the min of all possible assignments 
  std::vector<int>::const_iterator it = std::min_element(heur_allassignments.begin(), 
						    heur_allassignments.end());
  int min_assignment_index = it - heur_allassignments.begin();
  int heur = *it;

  heurs[0] = heur;
#ifdef DEBUG_HEUR  
  SBPL_INFO("Final heuristic is %d", heur);
#endif
  for(int agent_i = 0; agent_i < numagents_; agent_i++)
    heurs[agent_i+1] = heur_allagents_allassignments[min_assignment_index][agent_i];
  //std::cin.get();
}


template <typename HeuristicType>
int EGraphManager<HeuristicType>::bruteforceHeuristicPerAgent(int agent_i, 
							      std::vector<int>& heur_coord_agent,
							      const std::vector<int>& assignment,
							      const std::vector<int>& distance_to_goalV) const {
  std::vector<int> goalindices;
  /*
#ifdef DEBUG_HEUR
  SBPL_INFO("Getting heuristic for agent at (%d,%d)", heur_coord_agent[0], heur_coord_agent[1]);
#endif
*/
  const int maxnumgoals = MAXNUMGOALS;
  std::bitset<maxnumgoals> TSP_i_vertices;
  for(int i = 0; i < numgoals_; i ++){
    if(assignment[i] == agent_i){
      goalindices.push_back(i);
      TSP_i_vertices[maxnumgoals-i-1] = 1;
    }
  }
  unsigned long int TSPindex = TSP_i_vertices.to_ulong();
  const std::vector<int>* TSPcosts_pergoal = &TSP_allagents_[agent_i][TSPindex]; 
  int bestheursofar = std::numeric_limits<int>::max();
  // for each goal, run 2d bfs
  for(int goal_i = 0; goal_i < (int)goalindices.size(); goal_i++){
    bestheursofar = std::min(bestheursofar, 
			     TSPcosts_pergoal->at(goal_i) + distance_to_goalV[goal_i]);
  }
  return bestheursofar;
  /*
    #ifdef DEBUG_HEUR
    std::vector<int> goalcoord;
    getGoalCoord(goalindices[i], goalcoord);
    SBPL_INFO("Goal is (%d,%d)", goalcoord[0], goalcoord[1]);
    SBPL_INFO("Heuristic is %d", heur_val);
    #endif
  */ 
}


template <typename HeuristicType>
void EGraphManager<HeuristicType>::solveTSP(const Matrix& edgeCosts, 
					   const std::vector<int>& vertex_indices,
					   std::vector<int>& costV) const{
  
  //TODO: Will call TSP solver. For now, enumerate
  costV.resize(vertex_indices.size(), 0);
  int bestheursofar = std::numeric_limits<int>::max();
  int heur = 0;
  
  // starting at each vertex, solve TSP optimally
  for(unsigned int start_i = 0; start_i < vertex_indices.size(); start_i++){
    int startvertex = vertex_indices[start_i];
    // consider all possible permutations of other goals
    std::vector<int> nonstart_vertex_indices = vertex_indices;
    nonstart_vertex_indices.erase(nonstart_vertex_indices.begin()+start_i);
    do{
	// cost from startvertex to first goal
	heur = edgeCosts[startvertex][nonstart_vertex_indices[0]];
     
	// add costs from each goal to the next
	for(int i = 0; i < (int) vertex_indices.size(); i++){
	  heur += edgeCosts[nonstart_vertex_indices[i]][nonstart_vertex_indices[i+1]];
	  if(heur > bestheursofar)
	    break;
	}
	if(heur < bestheursofar)
	  bestheursofar = heur;
    }while(std::next_permutation(nonstart_vertex_indices.begin(), nonstart_vertex_indices.end()));
    costV[start_i] = bestheursofar;
  }
}

/*
template <typename HeuristicType>
int EGraphManager<HeuristicType>::solveTSP(int agent_i, std::vector<int>& goalindices)
{
  //TODO: Will call TSP solver. For now, enumerate
  if(goalindices.empty())
    return 0;

  int bestheursofar = std::numeric_limits<int>::max();;
  int heur = 0;
  
  do{
    for(int i =0; i < (int) goalindices.size(); i++)
      //SBPL_INFO("solveTSP: goalindices =%d",goalindices[i]);
      // cost from agent to first goal
    heur = TSPEdgecosts_[agent_i][numgoals_][goalindices[0]];
    
#ifdef DEBUG_HEUR
    SBPL_INFO("solveTSP: heur is %d", heur);
#endif
    
    // add costs from each goal to the next
    for(int i = 0; i < (int)goalindices.size()-1; i++){
      heur += TSPEdgecosts_[agent_i][goalindices[i]][goalindices[i+1]];
      if(heur > bestheursofar)
	break;
    }
    if(heur < bestheursofar)
      bestheursofar = heur;
  }while(std::next_permutation(goalindices.begin(), goalindices.end()));
  return bestheursofar;
}
*/

template <typename HeuristicType>
void EGraphManager<HeuristicType>::segmentEGraph()
{
  //TODO: Split egraph by agent. for now just return egraph
  egraphperagent_.clear();
  for(int i = 0; i < numagents_; i++)
    egraphperagent_.push_back(egraph_);
}

// goes through each vertex in the egraph and makes sure it has valid
// edges. if update_egraph=true, it recomputes the egraph components
template <typename HeuristicType>
void EGraphManager<HeuristicType>::validateEGraph(bool update_egraph){
    clock_t time = clock();
    int num_invalid_edges = 0;

    for(int agent_i = 0; agent_i < numagents_;agent_i++){
      for(size_t i=0; i < egraphperagent_[agent_i]->id2vertex.size(); i++){
        EGraph::EGraphVertex* v = egraphperagent_[agent_i]->id2vertex[i];
	std::vector<double> coord;
        egraphperagent_[agent_i]->discToCont(v,coord);
        for(unsigned int j=0; j<v->neighbors.size(); j++){
	  EGraph::EGraphVertex* u = v->neighbors[j];
	  if(v->id < u->id){
	    std::vector<double> coord2;
	    egraphperagent_[agent_i]->discToCont(u,coord2);
	    int cost = v->costs[j];
	    bool change_cost;
	    bool valid = egraph_env_->isValidEdge(coord,coord2,change_cost,cost);
	    if (!valid){
	      num_invalid_edges++;
	    }
	    if (update_egraph){
	      if(change_cost)
		egraphperagent_[agent_i]->updateEdge(v,u,valid,cost);
	      else
		egraphperagent_[agent_i]->updateEdge(v,u,valid);
	    }
	  }
        }
      }
    
      for(size_t i=0; i < egraph_->id2vertex.size(); i++){
        EGraph::EGraphVertex* v = egraphperagent_[agent_i]->id2vertex[i];
	std::vector<double> coord;
        egraphperagent_[agent_i]->discToCont(v,coord);
        if (!egraph_env_->isValidVertex(coord)){
            egraphperagent_[agent_i]->invalidateVertex(v);
        }
      }
    }
    if (update_egraph){
        egraph_->computeComponents();
	for(int agent_i = 0; agent_i < numagents_; agent_i++){
	  for(int goal_i = 0; goal_i < numgoals_; goal_i++){
	  egraph_heurs_[agent_i][goal_i]->runPrecomputations();
	  }
	}
    }

    ROS_INFO("num invalid edges from full egraph check: %d", num_invalid_edges);
    stats_.egraph_validity_check_time = double(clock()-time)/CLOCKS_PER_SEC;
}

//-----------TODO: Copied from egraphManager.cpp

// computes if a snap is actually valid between two ids
template <typename HeuristicType>
int EGraphManager<HeuristicType>::getSnapTrueCost(int parentID, int childID){
    Edge edge(parentID, childID);

    ContState source_state;
    egraph_env_->getCoord(parentID, source_state);
    
    ContState successor_state;
    egraph_env_->getCoord(childID, successor_state);

    int successor_id;
    int cost_of_snap;
    bool is_snap_successful = egraph_env_->snap(source_state, 
                                                successor_state,
                                                successor_id,
                                                cost_of_snap);
    if (!is_snap_successful){
        return -1;
    }
    assert(successor_id == childID);
    //ROS_INFO("source %d successor_id %d childID %d", parentID, successor_id, childID);
    return cost_of_snap;
}

// lazily generates snaps. also updates the snaps_ cache which currently just
// stores whether a particular edge is a snap edge.
template <typename HeuristicType>
void EGraphManager<HeuristicType>::getSnapSuccessors(int stateID, std::vector<int>* SuccIDV, 
						     std::vector<int>* CostV, 
						     std::vector<bool>* isTrueCost,
						     std::vector<EdgeType>* edgeTypes){
  
  //ROS_INFO("looking for snaps");
    ContState source_state;
    egraph_env_->getCoord(stateID, source_state);
    HeuristicType heuristic_coord;
    egraph_env_->projectToHeuristicSpace(source_state,heuristic_coord);
    std::vector<EGraph::EGraphVertex*> equal_heur_vertices;
    //ROS_INFO("get verts with same heur...");
    egraph_heur_->getEGraphVerticesWithSameHeuristic(heuristic_coord,
                                                     equal_heur_vertices);
    //ROS_INFO("%d possible snaps",equal_heur_vertices.size());

    // for all vertices with the same heuristic value, let's add their
    // successors to the list
    stats_.num_snaps += equal_heur_vertices.size();

    for(auto& egraph_vertex : equal_heur_vertices){
      std::vector<double> successor_state;
      int cost_of_snap;

      egraph_->discToCont(egraph_vertex, successor_state);
      int successor_id = egraph_env_->getStateID(successor_state);
      bool is_snap_successful = successor_id != stateID;
      
      // sbpl secret sauce (tm)
      cost_of_snap = 1;
      
      bool is_unique = find(SuccIDV->begin(), SuccIDV->end(), successor_id) == SuccIDV->end();
      if(is_snap_successful && is_unique){
            assert(cost_of_snap > 0);
            //ROS_INFO("snap from %d to %d with cost %d\n",stateID,successor_id,cost_of_snap);
            SuccIDV->push_back(successor_id);
            Edge edge(stateID, successor_id);
            //snaps_.insert({edge, successor_id});
            CostV->push_back(cost_of_snap);
            isTrueCost->push_back(false);
            edgeTypes->push_back(EdgeType::SNAP);
      }
    }
}

// a combo snap is:
//      source->[snap]->snap_id->[shortcut]->successor
// the cost of this successor will be the cost of the snap + cost of
// shortcut. 
template <typename HeuristicType>
void EGraphManager<HeuristicType>::getComboSnapShortcutSuccessors(int stateID, 
								  std::vector<int>* SuccIDV, 
								  std::vector<int>* CostV, 
								  std::vector<bool>* isTrueCost){
    clock_t time = clock();
    //ROS_INFO("trying to find combo snaps - found %lu", snap_successors.size());
    int num_combo_snaps = 0;
    for (size_t i=0; i < snap_successors_cache_.size(); i++){
        int snap_id = snap_successors_cache_[i];
	std::vector<bool> shortcut_is_true_cost;
	std::vector<int> shortcut_successors;
	std::vector<int> shortcut_costs;
	std::vector<EdgeType> edgeTypes;

        //ROS_INFO("looking up shortcut for snap_id %d", snap_id);
        getDirectShortcutSuccessors(snap_id, &shortcut_successors, 
                                    &shortcut_costs, &shortcut_is_true_cost, &edgeTypes);

        // we want to ignore snaps that don't lead to shortcuts, because we
        // already have those successors
        bool snap_leads_to_shortcut = shortcut_successors.size() != 0;
        if (!snap_leads_to_shortcut){
            //ROS_INFO("snap %d dosen't lead to shortcut, skipping", snap_id);
            continue;
        }

        // update shortcut with snap cost
        for (auto& cost : shortcut_costs){
            cost += snap_costs_cache_[i];
        }
        
        assert(shortcut_successors.size() == 1);
        for (size_t j=0; i < shortcut_successors.size(); i++){
            SuccIDV->push_back(shortcut_successors[j]);
            // again, assumes there's only one shortcut successor
            assert(shortcut_costs[j] > 0);
            CostV->push_back(shortcut_costs[j]);
            isTrueCost->push_back(true);
            //ROS_INFO("computed combo snap from %d to %d through snap %d cost %d", 
            //          stateID, shortcut_successors[j], snap_id, shortcut_costs[j]);

            // store the snap state id used for this combo
            /*
            Edge key(stateID, shortcut_successors[j]);
            std::pair<int, int> value(snap_id, shortcut_costs[j]);
            snap_combo_cache_.insert({{stateID, shortcut_successors[j]},
                                      {snap_id, shortcut_costs[j]}});
                                      */

            num_combo_snaps++;
        }
    }
    stats_.combo_time += static_cast<double>(clock()-time)/CLOCKS_PER_SEC;
}

template <typename HeuristicType>
void EGraphManager<HeuristicType>::getSnapShortcuts(int stateID, 
						    std::vector<int>* SuccIDV, 
						    std::vector<int>* CostV, 
						    std::vector<bool>* isTrueCost,
						    std::vector<EdgeType>* edgeTypes,
						    std::vector<int>* snap_midpoints){
  return; //TODO
  ContState source_state;
  egraph_env_->getCoord(stateID, source_state);
  HeuristicType heuristic_coord;
  egraph_env_->projectToHeuristicSpace(source_state,heuristic_coord);
  std::vector<EGraph::EGraphVertex*> equal_heur_vertices;
  //ROS_INFO("get verts with same heur...");
  egraph_heur_->getEGraphVerticesWithSameHeuristic(heuristic_coord, equal_heur_vertices);
  //ROS_INFO("%d possible snaps",equal_heur_vertices.size());

  // for all vertices with the same heuristic value
  for(auto& egraph_vertex : equal_heur_vertices){
    std::vector<double> successor_state;
    egraph_->discToCont(egraph_vertex, successor_state);
    int egraph_state_id = egraph_env_->getStateID(successor_state);
    assert(egraph_state_id >= 0);
    if(egraph_state_id == stateID)
      continue;

    std::vector<bool> true_costs;
    std::vector<int> shortcuts;
    std::vector<int> shortcut_costs;
    std::vector<EdgeType> et;
    getDirectShortcutSuccessors(egraph_state_id, &shortcuts, &shortcut_costs, &true_costs, &et);
    if(shortcuts.size()==0)
      continue;

    assert(shortcuts.size() == 1);
    SuccIDV->push_back(shortcuts[0]);
    assert(shortcut_costs[0] > 0);
    CostV->push_back(shortcut_costs[0]);
    isTrueCost->push_back(false);
    snap_midpoints->push_back(egraph_state_id);
    edgeTypes->push_back(EdgeType::SNAP_DIRECT_SHORTCUT);

    //ROS_INFO("snap-shortcut from %d to %d (through %d) with lazy cost %d",stateID,SuccIDV->back(),egraph_state_id,CostV->back());
  }
}

// computes if a snap is actually valid between two ids
template <typename HeuristicType>
int EGraphManager<HeuristicType>::getSnapShortcutTrueCost(int parentID, int snap_midpoint, int childID){
  ContState pre_snap;
  egraph_env_->getCoord(parentID, pre_snap);
  
  ContState post_snap;
  egraph_env_->getCoord(snap_midpoint, post_snap);

  int snap_id;
  int cost_of_snap;
  bool is_snap_successful = egraph_env_->snap(pre_snap, 
                                              post_snap,
                                              snap_id,
                                              cost_of_snap);
  if(!is_snap_successful)
    return -1;
  assert(snap_id == snap_midpoint);
  //ROS_INFO("source %d successor_id %d childID %d", parentID, successor_id, childID);

  std::vector<bool> true_costs;
  std::vector<int> shortcuts;
  std::vector<int> shortcut_costs;
  std::vector<EdgeType> edgeTypes;
  getDirectShortcutSuccessors(snap_midpoint, &shortcuts, &shortcut_costs, &true_costs, &edgeTypes);
  assert(shortcuts.size() == 1);
  assert(shortcuts[0] == childID);
  assert(shortcut_costs[0] > 0);

  return cost_of_snap + shortcut_costs[0];
}

template <typename HeuristicType>
bool EGraphManager<HeuristicType>::reconstructSnapShortcut(LazyAEGState* state,
							   LazyAEGState*& next_state,
							   std::vector<int>* wholePathIds, 
							   std::vector<int>* costs,
							   int& totalCost){

  //state->expanded_best_parent arrived at state using a snap-shortcut
  //state->expanded_best_parent -> uses snap -> state->expanded_snap_midpoint (fake_snap_state) -> uses shortcut -> state
  //recall we are reconstructing in reverse so the shortcut happens before the snap

  LazyAEGState fake_snap_state;
  fake_snap_state.id = state->expanded_snap_midpoint;
  fake_snap_state.expanded_best_parent = state->expanded_best_parent;

  LazyAEGState state_reached_by_shortcut;
  state_reached_by_shortcut.id = state->id;
  state_reached_by_shortcut.expanded_best_parent = &fake_snap_state;
  int dummy_shortcut_count=0;
  int totalShortcutCost;
  //uses state->id and state->expanded_best_parent->id
  assert(reconstructDirectShortcuts(&state_reached_by_shortcut, next_state, 
                                    wholePathIds, costs, dummy_shortcut_count,
                                    totalShortcutCost));
  
  //uses state->id and state->expanded_best_parent->id
  //state->expanded_best_parent is used to set next_state
  assert(reconstructSnap(&fake_snap_state, next_state, 
                         wholePathIds, costs));

  totalCost = totalShortcutCost + costs->back();

  return true;
}

template <typename HeuristicType>
DiscState EGraphManager<HeuristicType>::getDiscStateFromID(int state_id){
    ContState source_state;
    DiscState disc_source_state;
    egraph_env_->getCoord(state_id,source_state);
    egraph_->contToDisc(source_state, disc_source_state);
    return disc_source_state;
}

// returns all available shortcuts for a particular state id. a shortcut is the
// heuristically closest state to the goal on source_state_id's component.  this
// assumes there's <= 1 shortcut per component. this does NOT do anything with
// the points that make up the shortcut - that's dealt with in
// reconstructDirectShortcuts
template <typename HeuristicType>
void EGraphManager<HeuristicType>::getDirectShortcutSuccessors(int source_state_id, 
							       std::vector<int>* SuccIDV, 
							       std::vector<int>* CostV, 
							       std::vector<bool>* isTrueCost,
							       std::vector<EdgeType>* edgeTypes){
    //ROS_INFO("looking for direct shortcuts for %d", source_state_id);
    DiscState disc_source_state = getDiscStateFromID(source_state_id);
    //printVector(disc_source_state);

    EGraph::EGraphVertex* equiv_eg_vert = egraph_->getVertex(disc_source_state);
    // if source state does not lie on the egraph, return
    if(!equiv_eg_vert){ return; }
    //ROS_INFO("shortcut source %d is ", source_state_id);
    ContState source_coord;
    egraph_->discToCont(equiv_eg_vert, source_coord);
    //printVector(source_coord);

    clock_t time = clock();
    stats_.shortcut_time += static_cast<double>(clock()-time)/CLOCKS_PER_SEC;

    // the following code was only tested with a heuristic that computes one
    // shortcut per component. if you've got multiple shortcuts, there may or
    // may not be a problem with retrieving the correct unique_goal_id for later
    // reconstruction
    std::vector<EGraph::EGraphVertex*> shortcuts;
    double gds_t0 = ros::Time::now().toSec();
    egraph_heur_->getDirectShortcut(equiv_eg_vert->component,shortcuts);
    double gds_t1 = ros::Time::now().toSec();
    stats_.get_direct_shortcut_time += gds_t1-gds_t0;
    assert(shortcuts.size() <= 1);
    for(auto& shortcut_successor_egraph : shortcuts){
        ContState shortcut_successor_state;
        egraph_->discToCont(shortcut_successor_egraph, shortcut_successor_state);
        //ROS_INFO("looking at shortcut state ");
        //printVector(shortcut_successor_state);
        int successor_id = egraph_env_->getStateID(shortcut_successor_state);

        //bool successor_equals_source = (successor_id == source_state_id);
        // i think i might have to do this for ben's planner because the start
        // state (id 0) is stored separately, and there's a possibility that you
        // could getStateID(start_vector) and get a different id out
        bool successor_equals_source = (disc_source_state == shortcut_successor_egraph->coord);
        if(successor_equals_source){ continue; }

        SuccIDV->push_back(successor_id);
        double t0 = ros::Time::now().toSec();
        //int shortcut_cost = egraph_->getShortestPath(equiv_eg_vert,
        //                                             shortcut_successor_egraph);
        //for egraph getShortestPath caching reasons, we have to put the shortcut state first
        int shortcut_cost = egraph_->getShortestPath(shortcut_successor_egraph, equiv_eg_vert);
        //ROS_INFO("direct shortcut %d->%d (%d)",source_state_id,successor_id,shortcut_cost);
        assert(shortcut_cost > 0);
        CostV->push_back(shortcut_cost);
        double t1 = ros::Time::now().toSec();
        isTrueCost->push_back(true);
        edgeTypes->push_back(EdgeType::DIRECT_SHORTCUT);
        stats_.shortest_path_time += t1-t0;
    }
}

// figures out if we've used a direct shortcut or not. if we have, it gets the
// entire path segment (including the last point) for the shortcut
template <typename HeuristicType>
bool EGraphManager<HeuristicType>::reconstructDirectShortcuts(LazyAEGState* state, 
							      LazyAEGState*& next_state, 
							      std::vector<int>* wholePathIds, 
							      std::vector<int>* costs,
							      int& shortcut_count,
							      int& totalCost){
  std::vector<int> SuccIDV;
  std::vector<int> CostV;
  std::vector<bool> isTrueCost;
  std::vector<EdgeType> edgeTypes;
  getDirectShortcutSuccessors(state->expanded_best_parent->id, &SuccIDV, &CostV, 
			      &isTrueCost, &edgeTypes);
    int actioncost = INFINITECOST;
    int shortcut_id = -1;
    // determine a shortcut was used. 
    for(size_t i=0; i<SuccIDV.size(); i++){
        bool used_shortcut = (SuccIDV[i] == state->id && CostV[i]<actioncost);
        if (used_shortcut){
            actioncost = CostV[i];
            shortcut_id = state->id;
        //ROS_INFO("reconstructing shortcut to %d (not %d), cost %d %d", 
        //         shortcut_id, goal_id, CostV[i], actioncost);
        }
    }
    bool used_shortcut = (actioncost < INFINITECOST);
    if (used_shortcut){
        assert(shortcut_id > -1);
        fillInDirectShortcut(state->expanded_best_parent->id,
                             shortcut_id, wholePathIds, costs, shortcut_count);
        next_state = state->expanded_best_parent;
        totalCost = actioncost;
        return true;
    }
    return false;
}

// this fills shortcuts in reverse. TODO this probably dosen't work with
// backwarsd search
template <typename HeuristicType>
void EGraphManager<HeuristicType>::fillInDirectShortcut (int parent_id, int shortcut_id,
							 std::vector<int>* wholePathIds, 
							 std::vector<int>* costs, 
							 int& shortcut_count){
  //ROS_INFO("get the direct shortcut path %d %d",parent_id, shortcut_id);
  std::vector<int> shortcut_costs;
  std::vector<int> shortcut_path = getDirectShortcutStateIDs(shortcut_id,
							     parent_id,
							     &shortcut_costs);

    // assert that the shortcut path we get back is in reverse order
    assert(shortcut_path.back() == parent_id);
    assert(shortcut_path.front() == shortcut_id);

    // we exclude the last point in the shortcut path because we assume
    // state->id is already in wholePathIds. state->id is the start and
    // shortcut_id is the end, and we only need to push the end point
    for(size_t j=1; j<shortcut_path.size(); j++){
        wholePathIds->push_back(shortcut_path[j]);
    }

    //ROS_INFO("used shortcut of size %lu between %d %d", shortcut_path.size(), 
    //                                                    parent_id, shortcut_id);
    assert(shortcut_path.size() > 1);
    for (auto cost : shortcut_costs){
        assert(cost > 0);
    }

    shortcut_count += shortcut_path.size();
    //ROS_INFO("shortcut count is %d", shortcut_count);
    costs->insert(costs->end(), shortcut_costs.begin(), 
            shortcut_costs.end());
    //ROS_INFO("costs is size %lu, path ids is size %lu", costs->size(), 
    //        wholePathIds->size());
    //assert(costs->size() == wholePathIds->size()-1);
}

template <typename HeuristicType>
void EGraphManager<HeuristicType>::storeLastPath(const std::vector<int>& path, 
                                  const std::vector<int>& costs){
  std::vector<EGraphPath> full_path_allagents(numagents_);
  assert(path.size()-1 == costs.size());
  for (auto& state_id : path){
    ContState coord;
    egraph_env_->getCoord(state_id, coord);
    //printVector(coord);
    for(int agent_i = 0; agent_i < numagents_; agent_i++){
      ContState coord_agent;
      coord_agent.push_back(coord[4*agent_i]);
      coord_agent.push_back(coord[4*agent_i + 1]);
      coord_agent.push_back(coord[4*agent_i + 2]);
      coord_agent.push_back(coord[4*agent_i + 3]);
      full_path_allagents[agent_i].push_back(coord_agent);
    }
  }
  update_eg_thread_data_.path_to_feedback.resize(numagents_);
  update_eg_thread_data_.costs.resize(numagents_);
  for(int agent_i = 0; agent_i < numagents_; agent_i++){
    update_eg_thread_data_.path_to_feedback[agent_i] = full_path_allagents[agent_i];
    update_eg_thread_data_.costs[agent_i] = costs;
    //todo: this will become cost per agent/ we just recompute
  }
}

// given a start and end id, we get all the egraph vertices in between (with
// start and end also included). the assumption here is that start and end are
// on the same component. getShortestPath will break if this is not the case
template <typename HeuristicType>
std::vector<int> EGraphManager<HeuristicType>::getDirectShortcutStateIDs(int start_id, int end_id,
									 std::vector<int>* costs){
    DiscState disc_start = getDiscStateFromID(start_id);
    DiscState disc_end = getDiscStateFromID(end_id);
    //printVector(disc_start);
    EGraph::EGraphVertex* start_vertex = egraph_->getVertex(disc_start);
    EGraph::EGraphVertex* end_vertex = egraph_->getVertex(disc_end);

    std::vector<EGraph::EGraphVertex*> path;
    costs->clear();
    egraph_->getShortestPath(start_vertex, end_vertex, &path, costs);

    std::vector<int> ids;
    for(auto& vertex : path){
        ContState coord;
        EGraph::EGraphVertex* egraph_vertex = vertex;
        egraph_->discToCont(egraph_vertex, coord);
        ids.push_back(egraph_env_->getStateID(coord));
    }
    return ids;
}

template <typename HeuristicType>
bool EGraphManager<HeuristicType>::reconstructSnap(LazyAEGState* state, 
						   LazyAEGState*& next_state, 
						   std::vector<int>* wholePathIds, 
						   std::vector<int>* costs){
  std::vector<int> SuccIDV;
  std::vector<int> CostV;
  std::vector<bool> isTrueCost;
  std::vector<EdgeType> edgeTypes;
    
    getSnapSuccessors(state->expanded_best_parent->id, &SuccIDV, &CostV, &isTrueCost, &edgeTypes);
    int actioncost = INFINITECOST;
    int snap_id = -1;
    for(size_t i=0; i<SuccIDV.size(); i++){
        bool successor_matches_next_state = SuccIDV[i] == state->id;
        bool better_cost = CostV[i] < actioncost;

        if(successor_matches_next_state && better_cost){
            actioncost = CostV[i];
            snap_id = SuccIDV[i];
        }
    }

    bool used_snap = (actioncost < INFINITECOST);
    if (used_snap){
        assert(actioncost > 0);
        assert(snap_id > -1);
        costs->push_back(actioncost);
        next_state = state->expanded_best_parent;
        wholePathIds->push_back(next_state->id);
        assert(costs->size() == wholePathIds->size()-1);
        return true;
    } 
    return false;
}

template <typename HeuristicType>
bool EGraphManager<HeuristicType>::reconstructComboSnapShortcut(LazyAEGState* successor_state, 
								LazyAEGState*& next_state, 
								std::vector<int>* wholePathIds, 
								std::vector<int>* costs, 
								int goal_id){
  std::vector<int> SuccIDV;
  std::vector<int> CostV;
  std::vector<bool> isTrueCost;

    ROS_INFO("using a tasty snap combo!");
    Edge key(successor_state->expanded_best_parent->id, successor_state->id);
    int snap_id = -1;
    int snap_cost = -1;
    /* Mike!
    if (snap_combo_cache_.find(key) == snap_combo_cache_.end()){ 
        // no combo found
        return false;
    } else {
        pair<int, int> value = snap_combo_cache_[key];
        snap_id = value.first;
        snap_cost = value.second;
    }
    */
    int shortcut_id = successor_state->id;

    // fill in shortcut
    int shortcut_count = 0;
    fillInDirectShortcut(snap_id, shortcut_id, wholePathIds, costs, 
                         shortcut_count);
    
    // rethink through how this happens. fillInDirectShortcut adds the parent in
    // at the end
    assert(false);
    // fill in snap
    wholePathIds->push_back(snap_id);
    costs->push_back(snap_cost);

    next_state = successor_state->expanded_best_parent;
    return true;
}

template <typename HeuristicType>
void EGraphManager<HeuristicType>::feedbackLastPath(){

    double t0 = ros::Time::now().toSec();
    for(int agent_i = 0; agent_i < numagents_; agent_i++){
      // only store last path as experience
      egraphperagent_[agent_i]->clearEGraph();
      egraphperagent_[agent_i]->search_iteration_ = 0;
      if(params_.update_stats){
        egraphperagent_[agent_i]->recordStats(update_eg_thread_data_.path_to_feedback[agent_i]);
      }
      
      egraphperagent_[agent_i]->addPath(update_eg_thread_data_.path_to_feedback[agent_i],
		       update_eg_thread_data_.costs[agent_i]);
      stats_.feedback_time = ros::Time::now().toSec() - t0;
      
      double t2 = ros::Time::now().toSec();
      /*
      for(unsigned int i=0; i < egraphperagent_[agent_i]->id2vertex.size(); i++){
	EGraph::EGraphVertex* v = egraphperagent_[agent_i]->id2vertex[i];
	//errorCheckEGraphVertex(v); //TODO
	}*/
      if (update_eg_thread_data_.path_to_feedback[agent_i].size()){
	for(int goal_i = 0; goal_i < numgoals_; goal_i++)
	  egraph_heurs_[agent_i][goal_i]->runPrecomputations(); 
      }
      double t3 = ros::Time::now().toSec();
      stats_.error_check_time = t3-t2;
      update_eg_thread_data_.path_to_feedback[agent_i].clear();
      update_eg_thread_data_.costs[agent_i].clear();
      stats_.precomp_time = 0;
    }
}

template <typename HeuristicType>
void EGraphManager<HeuristicType>::printVector(std::vector<double>& state){
    for (auto value : state){
        printf("%f ", value);
    }
    printf("\n");
}

template <typename HeuristicType>
void EGraphManager<HeuristicType>::printVector(std::vector<int>& state){
    for (auto value : state){
        printf("%d ", value);
    }
    printf("\n");
}


// initializes egraph by computing components, setting up the heuristic (down
// projecting, setting goal state) and resetting the shortcut cache.
template <typename HeuristicType>
void EGraphManager<HeuristicType>::initEGraph(bool set_goal){
    // this order is important because precomputations uses the number of
    // components
    clock_t time = clock();
    HeuristicType coord;
    if (set_goal){
      egraph_env_->projectGoalToHeuristicSpace(coord);
    }

    for(int agent_i = 0; agent_i < numagents_; agent_i++){
       egraphperagent_[agent_i]->computeComponents();
      for(int goal_i = 0; goal_i < numgoals_; goal_i++){
	if (set_goal){
	  HeuristicType coord_goal(2);
	  coord_goal[0] = coord[2*goal_i];
	  coord_goal[1] = coord[2*goal_i + 1];
	  /*#ifdef DEBUG_HEUR
	    SBPL_INFO("egraphManager: setting goal to (%d, %d)", coord_goal[0], coord_goal[1]);
	  cin.get();
	  #endif */
	  egraph_heurs_[agent_i][goal_i]->initialize(egraphperagent_[agent_i]);
	  egraph_heurs_[agent_i][goal_i]->setGoal(coord_goal);
	}
	egraph_heurs_[agent_i][goal_i]->runPrecomputations();
      }
    }
    stats_.precomp_time += static_cast<double>(clock()-time)/CLOCKS_PER_SEC;
     
}

// let's double check that we can transform between egraph vertices (continuous
// values) back into discrete graph coordinates, and then back into continuous
// values again
template <typename HeuristicType>
void EGraphManager<HeuristicType>::errorCheckEGraphVertex(EGraph::EGraphVertex* egv){
  std::vector<double> eg_coord;
  egraph_->discToCont(egv,eg_coord);
  //printVector(egv->coord);
  //printVector(eg_coord);
  int env_id = egraph_env_->getStateID(eg_coord);
  std::vector<double> env_coord;
  egraph_env_->getCoord(env_id, env_coord);
  std::vector<int> env_dcoord;
  egraph_->contToDisc(env_coord, env_dcoord);
  EGraph::EGraphVertex* egv2 = egraph_->getVertex(env_dcoord);
  if(egv==NULL){
    ROS_ERROR("[AEG] ErrorCheckEGraph: The vertex is NULL!\n");
    assert(false);
    }
    if(egv2 != egv){
        ROS_ERROR("[AEG] ErrorCheckEGraph: We didn't get back the egraph vertex we started with (memory addresses don't match)!\n");
        printf("Original E-Graph Vertex id: %d\n",egv->id);
        printf("Original disc: ");
        printVector(egv->coord);
        printf("Original cont: ");
        printVector(eg_coord);
        printf("getStateID=%d\n",env_id);
        printf("getCoord cont: ");
        printVector(env_coord);
        printf("getCoord disc: ");
        printVector(env_dcoord);
        printf("Returned E-Graph Vertex id: %d\n",egv2->id);
        assert(false);
    }
    if(egv2->id != egv->id){
        ROS_ERROR("[AEG] ErrorCheckEGraph: We didn't get back the egraph vertex we started with (egraph ids don't match)!\n");
        printf("Original E-Graph Vertex id: %d\n",egv->id);
        printf("Original disc: ");
        printVector(egv->coord);
        printf("Original cont: ");
        printVector(eg_coord);
        printf("getStateID=%d\n",env_id);
        printf("getCoord cont: ");
        printVector(env_coord);
        printf("getCoord disc: ");
        printVector(env_dcoord);
        printf("Returned E-Graph Vertex id: %d\n",egv2->id);
        assert(false);
    }
}

#endif
