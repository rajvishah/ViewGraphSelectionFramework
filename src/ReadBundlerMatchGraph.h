#ifndef READBUNDLERMATCHGRAPH_H
#define READBUNDLERMATCHGRAPH_H 
#include "defs.h"

class ViewInfoWithPoints;
class PairwiseInfoWithPoints;

bool ReadBundlerMatchGraph( string& matchesFile, vector<ViewInfoWithPoints>& viewInfo, 
    vector<PairwiseInfoWithPoints>& twoviewInfo, 
    std::unordered_map<theia::ViewIdPair, int>& edge2indexMap);
#endif /* READBUNDLERMATCHGRAPH_H */
