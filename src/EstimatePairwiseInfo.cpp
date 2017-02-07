#include "EstimatePairwiseInfo.h"
#include "EvaluateRelativePoseError.h"

#include "Drawings.h"
#include <iterator>

void CreateViewingGraph(vector<ViewInfoWithPoints>& viewInfo,
    vector<PairwiseInfoWithPoints>& twoviewInfo,
    theia::ViewGraph* viewingGraph) {

  for(int i=0; i < twoviewInfo.size(); i++) {

    PairwiseInfoWithPoints& pairwise = twoviewInfo[i];

    int img1 = pairwise.image1;
    int img2 = pairwise.image2;

    if(pairwise.viewGraphEdgeIntialized) {
      (*viewingGraph).AddEdge(pairwise.image1, pairwise.image2, pairwise.primaryMotion);
    }

  }
}
bool SubtractIndexedMatches( vector< theia::IndexedFeatureMatch >& initialPoints, 
    vector< theia::IndexedFeatureMatch >&  verifiedPoints, 
    vector< theia::IndexedFeatureMatch >& remainingPoints,
    int numFeatures1, int numFeatures2) {

  set< pair<FeatureId, FeatureId> > initialSet;
  set< pair<FeatureId, FeatureId> > verifiedSet;

  for(int i=0; i < initialPoints.size(); i++) {
    initialSet.insert( make_pair( initialPoints[i].feature1_ind,
          initialPoints[i].feature2_ind) );
  }

  for(int i=0; i < verifiedPoints.size(); i++) {
    verifiedSet.insert( make_pair( verifiedPoints[i].feature1_ind,
          verifiedPoints[i].feature2_ind) );
  }

  set< pair<FeatureId, FeatureId> > remMatches;

  set_difference(initialSet.begin(), initialSet.end(), 
      verifiedSet.begin(), verifiedSet.end(), 
      inserter(remMatches, remMatches.end()));

  if(remMatches.size() < 30) {
    return false;
  }

  set< pair<FeatureId, FeatureId> >::iterator itr;

  for(itr = remMatches.begin(); itr != remMatches.end(); itr++) {
    remainingPoints.push_back( theia::IndexedFeatureMatch( (*itr).first, 
          (*itr).second, 1.0) );

    if((*itr).first >= numFeatures1 || (*itr).second >=numFeatures2) {
      printf("\nFeature Index Higher than total features.. something wrong");
      printf("\nIdx1 %d, Idx2 %d, NF1 %d, NF2 %d", (*itr).first, (*itr).second, numFeatures1, numFeatures2);
      fflush(stdout);
      exit(-1);
    }
  }
  return true;
}

void CreateIndexedMatchesFromPairedIndices(vector< pair<FeatureId, FeatureId> >& pairedIds, 
    vector< theia::IndexedFeatureMatch >& indexedMatches) {
  indexedMatches.clear();
  for(int i=0; i < pairedIds.size(); i++) {
    indexedMatches.push_back( theia::IndexedFeatureMatch( pairedIds[i].first, pairedIds[i].second, 1.0) ); 
  }
}




bool EstimatePairwiseInfo( vector<ViewInfoWithPoints>& viewInfo, 
    vector<PairwiseInfoWithPoints>& twoviewInfo ) {

  theia::TwoViewMatchGeometricVerification::Options options;
  options.min_num_inlier_matches = 30;
  options.bundle_adjustment = true;

  for(int i=0; i < twoviewInfo.size(); i++) {
    PairwiseInfoWithPoints& pairwise = twoviewInfo[i];

    int img1 = pairwise.image1;
    int img2 = pairwise.image2;

    vector<theia::FeatureCorrespondence> verified_matches;
    theia::TwoViewMatchGeometricVerification twoViewVerifier1(options,
        viewInfo[img1].viewObj.CameraIntrinsicsPrior(), 
        viewInfo[img2].viewObj.CameraIntrinsicsPrior(),
        viewInfo[img1].keysWithDesc, viewInfo[img2].keysWithDesc,
        pairwise.putativeMatches);

    bool status = twoViewVerifier1.VerifyMatches(&verified_matches, 
        &pairwise.primaryMotion, &pairwise.relative_pose1);

    if(!status) {
      pairwise.hasTwoGeometries = false;
      printf("\nNo valid geometry for pair %d", i);
      fflush(stdout);
      continue;
    }

    pairwise.viewGraphEdgeIntialized = true;
    viewInfo[img1].connected = true;
    viewInfo[img2].connected = true;
    pairwise.primaryVerifiedMatches = twoViewVerifier1.matches_;

    //Estimate Triangulation Angles
    {

      const theia::CameraIntrinsicsPrior& intrinsics1 = 
        viewInfo[img1].viewObj.CameraIntrinsicsPrior();
      const theia::CameraIntrinsicsPrior& intrinsics2 = 
        viewInfo[img2].viewObj.CameraIntrinsicsPrior();

      const theia::TwoViewInfo& info = pairwise.primaryMotion;

      const vector<theia::IndexedFeatureMatch>& matches = 
        pairwise.primaryVerifiedMatches;

      pairwise.medianPrimaryTriAngle = FindMedianTriangulationAngles( 
          intrinsics1, intrinsics2, info, matches,
          viewInfo[img1].keysWithDesc, viewInfo[img2].keysWithDesc);

      pairwise.scores.medianTriangAngle = pairwise.medianPrimaryTriAngle;

      printf("\nPairwise median angle %lf", pairwise.medianPrimaryTriAngle); 

    }


     
    /**********************/
    // Compute homography
    pairwise.homographyEstimated = true;
    Eigen::Matrix3d H;
    int numHomographyInliers = ComputeHomography( pairwise,  
        viewInfo[img1], viewInfo[img2], H);
    if(numHomographyInliers > 16) {
      pairwise.homographyFound = true;
      pairwise.homography = H;
    } else {
      pairwise.homographyFound = false;
    }

    vector<theia::IndexedFeatureMatch> remainingIndexedMatches;
    vector< pair<FeatureId, FeatureId> > remainingMatches;
    printf("\nImage1 %d - %s, Image2 %d - %s", img1, viewInfo[img1].viewObj.Name().c_str(), img2, viewInfo[img2].viewObj.Name().c_str());
    if(!SubtractIndexedMatches(pairwise.putativeMatches, 
          pairwise.primaryVerifiedMatches, remainingIndexedMatches, 
          viewInfo[img1].keysWithDesc.keypoints.size(),
          viewInfo[img2].keysWithDesc.keypoints.size())) {
      printf("\nFound valid geometry for pair %d, no secondary geometry", i);
      pairwise.secondaryVerifiedMatches.clear();
      fflush(stdout);
      continue;
    }

    vector<theia::FeatureCorrespondence> verified_matches2; 
    theia::TwoViewMatchGeometricVerification twoViewVerifier2(options,
        viewInfo[img1].viewObj.CameraIntrinsicsPrior(), 
        viewInfo[img2].viewObj.CameraIntrinsicsPrior(),
        viewInfo[img1].keysWithDesc, viewInfo[img2].keysWithDesc,
        remainingIndexedMatches);

    bool status2 = twoViewVerifier2.VerifyMatches(&verified_matches2, 
        &pairwise.secondaryMotion, &pairwise.relative_pose2);

    if(!status2) {
      printf("\nFound valid geometry for pair %d, no secondary geometry", i);
      pairwise.secondaryVerifiedMatches.clear();
      fflush(stdout);
      continue;
    }
    
    printf("\nFound valid geometry for pair %d, found secondary geometry", i);
    pairwise.secondaryVerifiedMatches = twoViewVerifier2.matches_;
    pairwise.hasTwoGeometries = true;
  }

  return true;
}

double FindMedianTriangulationAngles( const theia::CameraIntrinsicsPrior& intrinsics1, 
    const theia::CameraIntrinsicsPrior& intrinsics2, 
    const theia::TwoViewInfo& info,
    const vector<theia::IndexedFeatureMatch>& matches,
    const theia::KeypointsAndDescriptors& features1,
    const theia::KeypointsAndDescriptors& features2 ) {

  vector<double> angles;

  theia::Camera camera1, camera2;
  camera1.SetFocalLength(info.focal_length_1);
  theia::SetCameraIntrinsicsFromPriors(intrinsics1, false, &camera1);

  camera2.SetOrientationFromAngleAxis(info.rotation_2);
  camera2.SetPosition(info.position_2);
  camera2.SetFocalLength(info.focal_length_2);
  theia::SetCameraIntrinsicsFromPriors(intrinsics2, false, &camera2);

  for (int i = 0; i < matches.size(); i++) {
    const theia::Keypoint& keypoint1 = features1.keypoints[matches[i].feature1_ind];
    const theia::Keypoint& keypoint2 = features2.keypoints[matches[i].feature2_ind];
    const theia::Feature feature1(keypoint1.x(), keypoint1.y());
    const theia::Feature feature2(keypoint2.x(), keypoint2.y());

    // Make sure that there is enough baseline between the point so that the
    //// triangulation is well-constrained.
    std::vector<Eigen::Vector3d> ray_directions(2); 
    ray_directions[0] = camera1.PixelToUnitDepthRay(feature1).normalized();
    ray_directions[1] = camera2.PixelToUnitDepthRay(feature2).normalized();

    double triAngleCos = ray_directions[0].dot(ray_directions[1]);
    double angleDeg = theia::RadToDeg(acos(triAngleCos));
    angles.push_back( angleDeg );
  }
  vector<double>::iterator itr = angles.begin() + angles.size()/2;
  std::nth_element(angles.begin(), itr , angles.end());

  double medianAngle = (*itr); 
  
  return medianAngle;
}



bool ComputeTwoGeometryScores( vector<ViewInfoWithPoints>& viewInfo,  
                            vector<PairwiseInfoWithPoints>& twoviewInfo ) {

  for(int i=0; i < twoviewInfo.size(); i++) {

    PairwiseInfoWithPoints& pairwise = twoviewInfo[i];
    if(!pairwise.viewGraphEdgeIntialized) {
      continue;
    }
   
    pairwise.hasTwoGeometries = (!pairwise.secondaryVerifiedMatches.empty()) ? true : false;
    if(!pairwise.hasTwoGeometries) {
      if(i==608) cout << "No valid 2nd Motion";
      pairwise.scores.angularDiff = 0.0f;
      pairwise.scores.positionDiff = 0.0f;
      fflush(stdout);
      continue;
    }

    double positionDiff = pairwise.primaryMotion.position_2.dot(
        pairwise.secondaryMotion.position_2);

    Eigen::Matrix3d rotation0_1, rotation0_2;
    ceres::AngleAxisToRotationMatrix(
        pairwise.primaryMotion.rotation_2.data(),
        ceres::ColumnMajorAdapter3x3(rotation0_1.data()));
    ceres::AngleAxisToRotationMatrix(
        pairwise.secondaryMotion.rotation_2.data(),
        ceres::ColumnMajorAdapter3x3(rotation0_2.data()));

    const Eigen::Matrix3d rotation_diff =
      rotation0_1.transpose() * rotation0_2;
    Eigen::Vector3d rotation_diff_angle_axis;
    ceres::RotationMatrixToAngleAxis(
        ceres::ColumnMajorAdapter3x3(rotation_diff.data()),
        rotation_diff_angle_axis.data());

    double angularDiff = theia::RadToDeg(rotation_diff_angle_axis.norm());
    double positionDiffDeg = theia::RadToDeg(acos(positionDiff));

    printf("\nRotation Diff %lf, Translation diff %lf", angularDiff, positionDiffDeg);
    fflush(stdout);

    vector< cv::Point2f > matchPointSet1, matchPointSet2;
    CreatePointSetsFromIndexedMatches( pairwise.primaryVerifiedMatches, 
        viewInfo[pairwise.image1].keysWithDesc.keypoints,
        viewInfo[pairwise.image2].keysWithDesc.keypoints,
        matchPointSet1, matchPointSet2);

    vector< cv::Point2f > matchPointSet21, matchPointSet22;
    CreatePointSetsFromIndexedMatches( pairwise.secondaryVerifiedMatches, 
        viewInfo[pairwise.image1].keysWithDesc.keypoints,
        viewInfo[pairwise.image2].keysWithDesc.keypoints,
        matchPointSet21, matchPointSet22);

    DrawOutliers(viewInfo[pairwise.image1].fullImageName, 
        matchPointSet1, matchPointSet21,"Image1");
    DrawOutliers(viewInfo[pairwise.image2].fullImageName, 
        matchPointSet2, matchPointSet22,"Image2");

    cv::waitKey(0);

    if(angularDiff < 5.0 && positionDiffDeg < 5.0) {
      
      if(i==608) cout << "Merged Motion";

      pairwise.mergeMotions = true;
      pairwise.scores.angularDiff = 0.0f;
      pairwise.scores.positionDiff = 0.0f;


      pairwise.primaryVerifiedMatches.insert( 
          pairwise.primaryVerifiedMatches.end(),
          pairwise.secondaryVerifiedMatches.begin(), 
          pairwise.secondaryVerifiedMatches.end());

      pairwise.secondaryVerifiedMatches.clear(); 
    } else {
      if(i==608) cout << "Has valid 2nd Motion";
      pairwise.mergeMotions = false;
      pairwise.scores.angularDiff = angularDiff;
      pairwise.scores.positionDiff = positionDiffDeg; 
    }
  }
}

void CreatePointSetsFromIndexedMatches(
    vector<theia::IndexedFeatureMatch>& matches,
    vector< theia::Keypoint >& keypoints1,
    vector< theia::Keypoint >& keypoints2,
    vector< cv::Point2f >& pointSet1,
    vector< cv::Point2f >& pointSet2) {

  if(matches.empty()) {
    return;
  }

  for (int i = 0; i < matches.size(); i++) {

    int matchIdx1 = matches[i].feature1_ind;
    int matchIdx2 = matches[i].feature2_ind;

    if(matchIdx1 >= keypoints1.size() || matchIdx2 >= keypoints2.size()) {
      printf("\nIndex:%d", i);
      printf("\nMIDX1 %d, MIDX2 %d", matchIdx1, matchIdx2);
      printf("\nSIZE1 %d, SIZE2 %d", keypoints1.size(), keypoints2.size());
      fflush(stdout);
    } 
    const theia::Keypoint& keypoint1 = keypoints1[matchIdx1];
    const theia::Keypoint& keypoint2 = keypoints2[matchIdx2];

    pointSet1.push_back( cv::Point2f(keypoint1.x(), keypoint1.y()) );
    pointSet2.push_back( cv::Point2f(keypoint2.x(), keypoint2.y()) );
  }
}


void CreatePointSetFromMatchIndices(
    vector< FeatureId >& indices,
    vector< theia::Keypoint >& keypoints1,
    vector< cv::Point2f >& pointSet1) {
  for (int i = 0; i < indices.size(); i++) {
    int matchIdx1 = indices[i];
    const theia::Keypoint& keypoint1 = keypoints1[matchIdx1];
    pointSet1.push_back( cv::Point2f(keypoint1.x(), keypoint1.y()) );
  }
}

bool ComputeOverlapRatioScores( vector<ViewInfoWithPoints>& viewInfo,  
                            vector<PairwiseInfoWithPoints>& twoviewInfo ) {

  for(int i=0; i < viewInfo.size(); i++) { 
    set<FeatureId> set1 = viewInfo[i].matchedFeatureIndices; 
    vector< FeatureId > matchedFeatureIndices;
    matchedFeatureIndices.assign(set1.begin(), set1.end() );

    vector< cv::Point2f > globalPointSet;
    CreatePointSetFromMatchIndices( matchedFeatureIndices, 
        viewInfo[i].keysWithDesc.keypoints, globalPointSet );

    viewInfo[i].hull.findConvexHullContour( globalPointSet );
  }


  for(int i=0; i < twoviewInfo.size(); i++) {

    PairwiseInfoWithPoints& pairwise = twoviewInfo[i];
    if(!pairwise.viewGraphEdgeIntialized) {
      continue;
    }

    vector< cv::Point2f > matchPointSet1, matchPointSet2, globalPointSet1, globalPointSet2;
    CreatePointSetsFromIndexedMatches( pairwise.primaryVerifiedMatches, 
        viewInfo[pairwise.image1].keysWithDesc.keypoints,
        viewInfo[pairwise.image2].keysWithDesc.keypoints,
        matchPointSet1, matchPointSet2);

    pairwise.hull1.findConvexHullContour( matchPointSet1 );
    pairwise.hull2.findConvexHullContour( matchPointSet2 );
    
    pairwise.scores.ratio1 = pairwise.hull1.area / viewInfo[pairwise.image1].hull.area;
    pairwise.scores.ratio2 = pairwise.hull2.area / viewInfo[pairwise.image2].hull.area;
  }
}

bool ComputeHomographyScores( vector<ViewInfoWithPoints>& viewInfo,  
                          vector<PairwiseInfoWithPoints>& twoviewInfo ) {
  for(int i=0; i < twoviewInfo.size(); i++) {

    PairwiseInfoWithPoints& pairwise = twoviewInfo[i];
    if(!pairwise.viewGraphEdgeIntialized) {
      continue;
    }

    if(!pairwise.homographyEstimated) {
      printf("\nRe-estimating homography");
      pairwise.homographyEstimated = true;
      int img1 = pairwise.image1;
      int img2 = pairwise.image2;
      Eigen::Matrix3d H;
      int numHomographyInliers = ComputeHomography( pairwise,  
          viewInfo[img1], viewInfo[img2], H);

      if(numHomographyInliers > 16) {
        pairwise.homographyFound = true;
        pairwise.homography = H;
      } else {
        pairwise.homographyFound = false;
      }
      pairwise.numHInliers = numHomographyInliers;
    }

    if(pairwise.homographyFound) { 
      Eigen::Matrix3d H = pairwise.homography;
      Eigen::Matrix3d Ht = H.transpose();
      Eigen::Matrix3d HtH = Ht*H;
      Eigen::Matrix3d residue = HtH - Eigen::Matrix3d::Identity();
      pairwise.scores.homographyScore = residue.norm() > 2.0f ? 2.0f : residue.norm();
    } else {
      pairwise.scores.homographyScore = 1.0f;
    }
//    pairwise.scores.homographyScore = pairwise.numHInliers/pairwise.primaryVerifiedMatches.size();
  }
}

void ComputeFeatureDistribution(set<int>& featIndices, vector<float> weights, vector< set<FeatureId> >& feat2ImgMap, vector<float>& featHist) {
  set<int>::iterator itr = featIndices.begin();
  for(int count = 0; itr != featIndices.end(); itr++, count++) {
    float weight = weights[count];
    int featIdx = *itr;
    set<FeatureId>& imgMap = feat2ImgMap[featIdx];

    set<FeatureId>::iterator itr1 = imgMap.begin();
    for(; itr1 != imgMap.end(); itr1++) {
      featHist[(*itr1)] += (1.0f*weight);
    }
  }

  for(int i=0; i < featHist.size(); i++) {
    featHist[i] /= featIndices.size();
  }
}  


bool EvaluateViewFitness( vector< ViewInfoWithPoints >& views, 
    vector< PairwiseInfoWithPoints>& twoViewInfoVec ) {

  vector< vector < bool > > connectionMat( views.size() ); 
  for(int i=0; i < views.size(); i++) {
    connectionMat[i].resize( views.size() , false);
    views[i].feature2ImageMap.resize( views[i].keysWithDesc.keypoints.size() );
  }

  for(int i=0; i < twoViewInfoVec.size(); i++) {
    PairwiseInfoWithPoints& pairwise = twoViewInfoVec[i];
    int img1 = pairwise.image1;
    int img2 = pairwise.image2;

    if(!pairwise.viewGraphEdgeIntialized) {
      continue;
    }

    for(int i=0; i < pairwise.primaryVerifiedMatches.size(); i++) {
      int matchIdx1 = pairwise.primaryVerifiedMatches[i].feature1_ind;
      int matchIdx2 = pairwise.primaryVerifiedMatches[i].feature2_ind;

      views[img1].feature2ImageMap[matchIdx1].insert(pairwise.image2);
      views[img2].feature2ImageMap[matchIdx2].insert(pairwise.image1); 
    }

    connectionMat[img1][img2] = true;
    connectionMat[img2][img1] = true;
    
    views[img1].connectedImages.insert(img2);
    views[img2].connectedImages.insert(img1);
  }


  vector< double > groupConnectivity( views.size() , 0.0);
  int maxDegree = 0;
  for(int i=0; i < views.size(); i++) {
    set<theia::ViewId>::iterator itr1, itr2;
    for(itr1 = views[i].connectedImages.begin(); 
        itr1 != views[i].connectedImages.end(); itr1++) {

      int img1 = (*itr1);
      for(itr2 = views[i].connectedImages.begin(); 
          itr2 != itr1; itr2++) {

        int img2 = (*itr2);
        if(connectionMat[img1][img2]) {
          groupConnectivity[i]++;
        }
      } 
    }
    int degree = views[i].connectedImages.size();
    maxDegree = maxDegree < degree ? degree : maxDegree;
  }

  for(int i=0; i < views.size(); i++) {
    double degree = (double)(views[i].connectedImages.size());
    double numFeatures = (double)(views[i].keysWithDesc.keypoints.size());
    double numMatchedFeatures = (double)(views[i].matchedFeatureIndices.size());
    views[i].disjointFitness = (double)(groupConnectivity[i]) /(degree*(degree-1) / 2);
    views[i].connectivityFitness = (double)(degree) / (maxDegree * 1.0f);
    views[i].featureFitness = (double)(numMatchedFeatures)/ (numFeatures);
  }

  return true;
}


bool ComputeContextScore( vector< ViewInfoWithPoints >& views, 
    vector< PairwiseInfoWithPoints>& twoViewInfoVec ) {


  for(int i=0; i < twoViewInfoVec.size(); i++) {
    PairwiseInfoWithPoints& pairwise = twoViewInfoVec[i];
    int img1 = pairwise.image1;
    int img2 = pairwise.image2;

    if(!pairwise.viewGraphEdgeIntialized) {
      continue;
    }

    int degree1 = views[img1].connectedImages.size();
    int degree2 = views[img2].connectedImages.size();

    set<FeatureId>& s1 = views[img1].matchedFeatureIndices;
    set<FeatureId>& s2 = views[img2].matchedFeatureIndices;

    set<FeatureId>& d1 = pairwise.matchedFeatureIndices1;
    set<FeatureId>& d2 = pairwise.matchedFeatureIndices2;

    set<int> u1, u2;
    set_difference(s1.begin(), s1.end(), d1.begin(), d1.end(), inserter(u1, u1.end())); 
    set_difference(s2.begin(), s2.end(), d2.begin(), d2.end(), inserter(u2, u2.end())); 

    vector< float > identityWeights1(u1.size(), 1.0f);
    vector< float > identityWeights2(u2.size(), 1.0f);

    vector< float > contextFeatureDist1;
    vector< float > contextFeatureDist2;

    int numImages = views.size();

    contextFeatureDist1.resize(numImages, 0.0f);
    contextFeatureDist2.resize(numImages, 0.0f);

    ComputeFeatureDistribution(u1, identityWeights1, 
        views[img1].feature2ImageMap, contextFeatureDist1); 
    ComputeFeatureDistribution(u2, identityWeights2, 
        views[img2].feature2ImageMap, contextFeatureDist2); 

    float* ptr1 = contextFeatureDist1.data();
    float* ptr2 = contextFeatureDist2.data();

    Eigen::Map<Eigen::VectorXf> vec1( ptr1 , contextFeatureDist1.size());
    Eigen::Map<Eigen::VectorXf> vec2( ptr2 , contextFeatureDist2.size());

    double threshold1 = 10.0/u1.size();
    double threshold2 = 10.0/u2.size();

    double hammingDistance = 0.0f;
    for(int j=0; j < contextFeatureDist1.size(); j++) {
      bool val1 = false;
      bool val2 = false;
      if(contextFeatureDist1[j] > threshold1) {
        val1 = true;
      } 

      if(contextFeatureDist2[j] > threshold2) {
        val2 = true;
      }

      if(val1 != val2) {
        hammingDistance = hammingDistance + 1;
      }
    }

    if(hammingDistance > degree1 + degree2) {
      printf("\nProblem with context score");
    }

    pairwise.scores.contextScoreChiSqr = cv::compareHist(cv::Mat(contextFeatureDist1), cv::Mat(contextFeatureDist2), CV_COMP_CHISQR);
    pairwise.scores.contextScoreCosine = vec1.dot(vec2);
    pairwise.scores.contextScoreHamming = hammingDistance / (float)(degree1 + degree2);

  }
}
