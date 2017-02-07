#ifndef PAIRWISEINFOWITHPOINTS_H
#define PAIRWISEINFOWITHPOINTS_H

#include "defs.h"
#include "Util.h"

#include <cereal/types/utility.hpp>
#include <cereal/types/set.hpp>
#include <theia/theia.h>

struct PairwiseScores {
  double ratio1;
  double ratio2;

  double angularDiff;
  double positionDiff;

  double tripletError;
  double tripletScore;
  double homographyScore;

  double contextScoreHamming;
  double contextScoreChiSqr;
  double contextScoreCosine;

  double medianTriangAngle;

  friend class cereal::access;
  template<class Archive>
  void serialize(Archive & archive) {
    archive( ratio1, ratio2, angularDiff, positionDiff, 
        tripletError, tripletScore, homographyScore, contextScoreCosine, 
        contextScoreHamming, contextScoreChiSqr, medianTriangAngle);
  }
};


class PairwiseInfoWithPoints {

  public:

  double cost;

  //Populated at the time of reading match graph
  ViewId image1;
  ViewId image2;

  bool matchGraphEdgeIntialized;
  set<FeatureId> matchedFeatureIndices1; 
  set<FeatureId> matchedFeatureIndices2;
  vector<theia::IndexedFeatureMatch> putativeMatches;

  //Populated at estimating two view geometry
  bool viewGraphEdgeIntialized;
  theia::TwoViewInfo primaryMotion;
  theia::TwoViewInfo secondaryMotion;
  theia::RelativePose relative_pose1;
  theia::RelativePose relative_pose2;
  
  bool hasTwoGeometries;
  bool mergeMotions;
  
  vector<theia::IndexedFeatureMatch> primaryVerifiedMatches;
  double medianPrimaryTriAngle;
  vector<theia::IndexedFeatureMatch> secondaryVerifiedMatches;

  vector<float> triangulationAnglesSec;

  //Populated at estimating homography
  bool homographyEstimated;
  bool homographyFound;
  Eigen::Matrix3d homography;
  int numHInliers;

  //Populated while computing triplet geometry
  bool tripletEstimated;
  vector<float> tripletErrorVec;
  vector< pair<int, int> > connectedTripletEdges; 


  //Populated while computing scores 
  FeatureConvexHull hull1;
  FeatureConvexHull hull2;

  PairwiseScores scores;
  PairwiseInfoWithPoints();

private:
  friend class cereal::access;
  template<class Archive>
  void serialize(Archive & archive) {
    archive( image1, image2, 
        matchGraphEdgeIntialized,
        matchedFeatureIndices1,
        matchedFeatureIndices2,
        viewGraphEdgeIntialized,
        primaryMotion,
        secondaryMotion,
        putativeMatches,
        primaryVerifiedMatches,
        secondaryVerifiedMatches,
        tripletErrorVec,
        connectedTripletEdges,
        homographyEstimated,
        homographyFound,
        homography,
        numHInliers,
        scores); // serialize things by passing them to the archive
  }
};

bool WritePairwiseInfoToDisk( string fileName, PairwiseInfoWithPoints& pair);
bool ReadPairwiseInfoFromDisk( string fileName, PairwiseInfoWithPoints* pair);
  


#endif /* PAIRWISEINFOWITHPOINTS_H */
