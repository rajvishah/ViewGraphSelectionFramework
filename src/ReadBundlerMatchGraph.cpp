#include "ReadBundlerMatchGraph.h"
#include "PairwiseInfoWithPoints.h"
#include "ViewInfoWithPoints.h"

// Read match graph file (bundler format) and populate fileds of ViewInfo and Pairwise classes
bool ReadBundlerMatchGraph( string& matchesFile, vector<ViewInfoWithPoints>& viewInfo, 
    vector<PairwiseInfoWithPoints>& pairwiseInfo, 
    std::unordered_map<theia::ViewIdPair, int>& edge2indexMap) {

  pairwiseInfo.reserve(500);

  // Read matches file and veriy two view matches
  FILE* fp = fopen( matchesFile.c_str() , "r");
  if( fp == NULL ) {
    printf("\nCould not read matches file");
    fflush(stdout);
    return 0;
  }

  int numTotalPairs = 0;
  int img1, img2;
  int pairCounter = 0;
  while(fscanf(fp,"%d %d",&img1,&img2) != EOF) {

    //viewInfo[img1].degree++;
    //viewInfo[img2].degree++;

    printf("\nAdding image pair %d %d", img1, img2);
    fflush(stdout);

    numTotalPairs++;

    int numMatches;
    fscanf(fp,"%d",&numMatches);

    PairwiseInfoWithPoints pairwise;
    pairwise.image1 = img1;
    pairwise.image2 = img2;

    // Correspondence indices to estimate pairwise geometry

    for(int i=0; i < numMatches; i++) {
      int matchIdx1, matchIdx2;
      fscanf(fp,"%d %d",&matchIdx1, &matchIdx2);

      // Enter the feature coordinates to list if not already done
      pair< set<FeatureId>::iterator, bool > inSet1;
      pair< set<FeatureId>::iterator, bool > inSet2;

      inSet1 = viewInfo[img1].matchedFeatureIndices.insert(matchIdx1);
      inSet2 = viewInfo[img2].matchedFeatureIndices.insert(matchIdx2);

      // Add match indices and coordinates to twoView class
      pairwise.matchedFeatureIndices1.insert(matchIdx1);
      pairwise.matchedFeatureIndices2.insert(matchIdx2);

      theia::IndexedFeatureMatch m1( matchIdx1, matchIdx2, 1.0);
      pairwise.putativeMatches.push_back(m1);
    }

    theia::ViewIdPair pair(img1, img2);
    edge2indexMap.insert(make_pair(pair, pairwiseInfo.size()));

    pairwise.matchGraphEdgeIntialized = true;
    pairwise.viewGraphEdgeIntialized = false;
    pairwiseInfo.push_back(pairwise);

    printf("\nRead match pair %d", numTotalPairs);
    fflush(stdout);
    
  }

  return true;
}

