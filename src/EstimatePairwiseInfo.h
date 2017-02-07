#ifndef ESTIMATEPAIRWISEINFO_H
#define ESTIMATEPAIRWISEINFO_H 

#include "PairwiseInfoWithPoints.h"
#include "ViewInfoWithPoints.h"
#include "defs.h"

bool EstimatePairwiseInfo( vector<ViewInfoWithPoints>& viewInfo, vector<PairwiseInfoWithPoints>& twoviewInfo );
bool ComputeTwoGeometryScores( vector<ViewInfoWithPoints>& viewInfo,  
                            vector<PairwiseInfoWithPoints>& twoviewInfo );


bool ComputeOverlapRatioScores( vector<ViewInfoWithPoints>& viewInfo,  
                            vector<PairwiseInfoWithPoints>& twoviewInfo );
bool ComputeHomographyScores( vector<ViewInfoWithPoints>& viewInfo,  
                          vector<PairwiseInfoWithPoints>& twoviewInfo );

bool ComputeContextScore( vector< ViewInfoWithPoints >& views, 
    vector< PairwiseInfoWithPoints>& twoViewInfoVec);

bool EvaluateViewFitness( vector< ViewInfoWithPoints >& views, 
    vector< PairwiseInfoWithPoints>& twoViewInfoVec );
void CreatePointSetsFromIndexedMatches(
    vector<theia::IndexedFeatureMatch>& matches,
    vector< theia::Keypoint >& keypoints1,
    vector< theia::Keypoint >& keypoints2,
    vector< cv::Point2f >& pointSet1,
    vector< cv::Point2f >& pointSet2);
void CreateViewingGraph(vector<ViewInfoWithPoints>& viewInfo,
    vector<PairwiseInfoWithPoints>& twoviewInfo,
    theia::ViewGraph* viewingGraph);
double FindMedianTriangulationAngles( const theia::CameraIntrinsicsPrior& intrinsics1, 
    const theia::CameraIntrinsicsPrior& intrinsics2, 
    const theia::TwoViewInfo& info,
    const vector<theia::IndexedFeatureMatch>& matches,
    const theia::KeypointsAndDescriptors& features1,
    const theia::KeypointsAndDescriptors& features2 );
#endif /* ESTIMATEPAIRWISEINFO_H */
