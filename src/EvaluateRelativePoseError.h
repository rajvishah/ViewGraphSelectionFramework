#ifndef EVALUATERELATIVEPOSEERROR_H
#define EVALUATERELATIVEPOSEERROR_H

#include "defs.h"
#include "ViewInfoWithPoints.h"
#include "PairwiseInfoWithPoints.h"
void NormalizeFeatures(
    const theia::CameraIntrinsicsPrior& prior1,
    const theia::CameraIntrinsicsPrior& prior2,
    const std::vector<theia::FeatureCorrespondence>& correspondences,
    std::vector<theia::FeatureCorrespondence>* normalized_correspondences);

void CreateCorrespondencesFromIndexedMatches(
    ViewInfoWithPoints& view1,
    ViewInfoWithPoints& view2,
    vector<theia::IndexedFeatureMatch>& matches,
    std::vector<theia::FeatureCorrespondence>* correspondences);

double RelativePoseError(const theia::FeatureCorrespondence& 
    correspondence, const theia::RelativePose& relative_pose);

const double ComputeResolutionScaledThreshold(double std_threshold, int width, int height);

double GetRelativePoseErrorThreshold(ViewInfoWithPoints& view1, ViewInfoWithPoints& view2);
int GetRelativePoseInliers(vector<theia::IndexedFeatureMatch>& indexedMatches, 
    ViewInfoWithPoints& view1,
    ViewInfoWithPoints& view2, 
    vector< pair< FeatureId, FeatureId> >& inlierIndices,
    theia::RelativePose relative_pose, double scale, bool flag);


int ComputeHomography( PairwiseInfoWithPoints& pairwise, ViewInfoWithPoints& view1, ViewInfoWithPoints& view2, Eigen::Matrix3d& homography);
#endif /* EVALUATERELATIVEPOSEERROR_H */
