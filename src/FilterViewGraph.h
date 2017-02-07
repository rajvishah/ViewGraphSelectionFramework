#ifndef FILTERVIEWGRAPH_H
#define FILTERVIEWGRAPH_H 

#include "PairwiseInfoWithPoints.h"
#include "ViewInfoWithPoints.h"

class ViewGraphFilter {
  vector< pair< pair<int,int>, double > > matA_sparse;
  //unordered_map< pair<int,int>, double > matA_sparse;
  vector<double> vecB;
  vector<double> lb;
  vector<double> ub;

  vector<double> cost;

  double sourceWeight;
  double sinkWeight;

  int totalFlow;

  int numVariables;
  int numConstraints;

  public:

  vector<double> networkFlowSol;
  vector< pair<string,int> > nwSol2vgIdx;

  vector< PairwiseInfoWithPoints >* pairsInfoPtr;
  vector< ViewInfoWithPoints >* viewsPtr;
  std::unordered_map<theia::ViewIdPair, int>* edge2indexMap;

  bool constructNetworkFlowProblem(int tFlow);
  bool solveNetworkFlowProblem();

  void writeSolution(string filename);
};




#endif /* FILTERVIEWGRAPH_H */
