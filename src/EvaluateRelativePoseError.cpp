#include "EvaluateRelativePoseError.h"
void NormalizeFeatures(
    const theia::CameraIntrinsicsPrior& prior1,
    const theia::CameraIntrinsicsPrior& prior2,
    const std::vector<theia::FeatureCorrespondence>& correspondences,
    std::vector<theia::FeatureCorrespondence>* normalized_correspondences) {
  normalized_correspondences->clear();
  static const bool kSetFocalLengthFromMedianFOV = false;
  theia::Camera camera1, camera2;
  theia::SetCameraIntrinsicsFromPriors(prior1, 
      kSetFocalLengthFromMedianFOV, &camera1);
  theia::SetCameraIntrinsicsFromPriors(prior2, 
      kSetFocalLengthFromMedianFOV, &camera2);
  normalized_correspondences->reserve(correspondences.size());
  for (const theia::FeatureCorrespondence& correspondence : correspondences) {
    theia::FeatureCorrespondence normalized_correspondence;
    const Eigen::Vector3d normalized_feature1 =
      camera1.PixelToNormalizedCoordinates(correspondence.feature1);
    normalized_correspondence.feature1 = normalized_feature1.hnormalized();

    const Eigen::Vector3d normalized_feature2 =
      camera2.PixelToNormalizedCoordinates(correspondence.feature2);
    normalized_correspondence.feature2 = normalized_feature2.hnormalized();

    normalized_correspondences->emplace_back(normalized_correspondence);
  }
}

void CreateCorrespondencesFromIndexedMatches(
    ViewInfoWithPoints& view1,
    ViewInfoWithPoints& view2,
    vector<theia::IndexedFeatureMatch>& matches,
    std::vector<theia::FeatureCorrespondence>* correspondences) {
  CHECK_NOTNULL(correspondences)->clear();
  correspondences->reserve(matches.size());
  for (int i = 0; i < matches.size(); i++) {
    int matchIdx1 = matches[i].feature1_ind;
    int matchIdx2 = matches[i].feature2_ind;
    const theia::Keypoint& keypoint1 = view1.keysWithDesc.keypoints[matchIdx1];
    const theia::Keypoint& keypoint2 = view2.keysWithDesc.keypoints[matchIdx2];
    correspondences->emplace_back(theia::Feature(keypoint1.x(), keypoint1.y()), theia::Feature(keypoint2.x(), keypoint2.y()));

  }
}

double RelativePoseError(const theia::FeatureCorrespondence& 
    correspondence,
    const theia::RelativePose& relative_pose) {
  if (theia::IsTriangulatedPointInFrontOfCameras(correspondence,
        relative_pose.rotation,
        relative_pose.position)) {
    return theia::SquaredSampsonDistance(relative_pose.essential_matrix,
        correspondence.feature1,
        correspondence.feature2);
  }
  return std::numeric_limits<double>::max();
}

const double ComputeResolutionScaledThreshold(double std_threshold, int width, int height) {
  static const double kDefaultImageDimension = 1024.0;

  if (width == 0 && height == 0) {
    return std_threshold;
  }

  const int max_image_dimension = std::max(width, height);
  return std_threshold * static_cast<double>(max_image_dimension) /
    kDefaultImageDimension;
}

double GetRelativePoseErrorThreshold(ViewInfoWithPoints& view1, ViewInfoWithPoints& view2) {

  double max_sampson_error_pixels = 6.0;

  theia::CameraIntrinsicsPrior* prior1 = view1.viewObj.MutableCameraIntrinsicsPrior();
  theia::CameraIntrinsicsPrior* prior2 = view2.viewObj.MutableCameraIntrinsicsPrior();

  const double max_sampson_error_pixels1 = ComputeResolutionScaledThreshold(
      max_sampson_error_pixels,
      prior1->image_width, prior1->image_height);

  const double max_sampson_error_pixels2 = ComputeResolutionScaledThreshold(
      max_sampson_error_pixels,
      prior2->image_width,
      prior2->image_height);

  double error_thresh = max_sampson_error_pixels1 * max_sampson_error_pixels2 /
    (prior1->focal_length.value * prior2->focal_length.value);

  return error_thresh;
}

int GetRelativePoseInliers(vector<theia::IndexedFeatureMatch>& indexedMatches, 
    ViewInfoWithPoints& view1,
    ViewInfoWithPoints& view2, 
    vector< pair< FeatureId, FeatureId> >& inlierIndices,
    theia::RelativePose relative_pose,
    double scale, 
    bool printErrors = false) {

  theia::CameraIntrinsicsPrior* prior1 = view1.viewObj.MutableCameraIntrinsicsPrior();
  theia::CameraIntrinsicsPrior* prior2 = view2.viewObj.MutableCameraIntrinsicsPrior();
  vector<theia::FeatureCorrespondence> matches, normalized_matches;
  CreateCorrespondencesFromIndexedMatches(view1, view2, indexedMatches, &matches);
  NormalizeFeatures(view1.viewObj.CameraIntrinsicsPrior(), view2.viewObj.CameraIntrinsicsPrior(), 
      matches, &normalized_matches);
  const double error_thresh = scale * scale * GetRelativePoseErrorThreshold(view1, view2);

  /*
  printf("\nError threshold in pixels %lf", error_thresh* 
    (prior1->focal_length.value * prior2->focal_length.value));

  printf("\nError threshold %lf", error_thresh);
  */

  int inlCount = 0;
  vector< double > errors( normalized_matches.size() );
  for(int c = 0; c < normalized_matches.size(); c++) {
    errors[c] = RelativePoseError(normalized_matches[c], relative_pose);
    if(errors[c] < error_thresh) {

      /*
      theia::Feature feat1 = matches[c].feature1;
      theia::Feature feat2 = matches[c].feature2;

      cv::Point2f pt1(feat1.x(), feat1.y());
      cv::Point2f pt2(feat2.x(), feat2.y());

      pairwise.verifiedFeatures1.push_back(pt1);
      pairwise.verifiedFeatures2.push_back(pt2);
      */

      theia::IndexedFeatureMatch match = indexedMatches[c];
      inlierIndices.push_back( make_pair(match.feature1_ind, match.feature2_ind) );
    }
  }

  fflush(stdout);

  return (int)(inlierIndices.size());
}

int ComputeHomography( PairwiseInfoWithPoints& pairwise, ViewInfoWithPoints& view1, ViewInfoWithPoints& view2, Eigen::Matrix3d& homography) {

  vector<theia::FeatureCorrespondence> correspondences, norm_correspondences;
  CreateCorrespondencesFromIndexedMatches(view1, view2, pairwise.putativeMatches, &correspondences);

  NormalizeFeatures(view1.viewObj.CameraIntrinsicsPrior(), view2.viewObj.CameraIntrinsicsPrior(), 
      correspondences, &norm_correspondences);
  

  double threshold = GetRelativePoseErrorThreshold(view1, view2);
  
  theia::RansacParameters homography_params;
  homography_params.error_thresh = threshold;

  homography_params.max_iterations = 1000;
  homography_params.min_iterations = 10;
  homography_params.use_mle = true;
  homography_params.failure_probability =
    1.0 - 0.9999;

  theia::RansacSummary homography_summary;

  EstimateHomography(homography_params, theia::RansacType::RANSAC,
      norm_correspondences, &homography, &homography_summary);

  return homography_summary.inliers.size();
}

