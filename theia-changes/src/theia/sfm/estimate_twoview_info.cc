// Copyright (C) 2014 The Regents of the University of California (Regents).
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above
//       copyright notice, this list of conditions and the following
//       disclaimer in the documentation and/or other materials provided
//       with the distribution.
//
//     * Neither the name of The Regents or University of California nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Please contact the author of this library if you have any questions.
// Author: Chris Sweeney (cmsweeney@cs.ucsb.edu)

#include "theia/sfm/estimate_twoview_info.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <glog/logging.h>

#include <vector>

#include "theia/matching/feature_correspondence.h"
#include "theia/sfm/camera/camera.h"
#include "theia/sfm/camera_intrinsics_prior.h"
#include "theia/sfm/estimators/estimate_relative_pose.h"
#include "theia/sfm/estimators/estimate_uncalibrated_relative_pose.h"
#include "theia/sfm/pose/util.h"
#include "theia/sfm/set_camera_intrinsics_from_priors.h"
#include "theia/sfm/triangulation/triangulation.h"
#include "theia/sfm/twoview_info.h"
#include "theia/sfm/types.h"
#include "theia/solvers/sample_consensus_estimator.h"

namespace theia {

using Eigen::AngleAxisd;
using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;

namespace {

// Normalizes the image features by the camera intrinsics.
void NormalizeFeatures(
    const CameraIntrinsicsPrior& prior1,
    const CameraIntrinsicsPrior& prior2,
    const std::vector<FeatureCorrespondence>& correspondences,
    std::vector<FeatureCorrespondence>* normalized_correspondences) {
  CHECK_NOTNULL(normalized_correspondences)->clear();
  static const bool kSetFocalLengthFromMedianFOV = false;

  Camera camera1, camera2;
  SetCameraIntrinsicsFromPriors(prior1, kSetFocalLengthFromMedianFOV, &camera1);
  SetCameraIntrinsicsFromPriors(prior2, kSetFocalLengthFromMedianFOV, &camera2);
  normalized_correspondences->reserve(correspondences.size());
  for (const FeatureCorrespondence& correspondence : correspondences) {
    FeatureCorrespondence normalized_correspondence;
    const Eigen::Vector3d normalized_feature1 =
        camera1.PixelToNormalizedCoordinates(correspondence.feature1);
    normalized_correspondence.feature1 = normalized_feature1.hnormalized();

    const Eigen::Vector3d normalized_feature2 =
        camera2.PixelToNormalizedCoordinates(correspondence.feature2);
    normalized_correspondence.feature2 = normalized_feature2.hnormalized();

    normalized_correspondences->emplace_back(normalized_correspondence);
  }
}

bool EstimateTwoViewInfoCalibrated(
    const EstimateTwoViewInfoOptions& options,
    const CameraIntrinsicsPrior& intrinsics1,
    const CameraIntrinsicsPrior& intrinsics2,
    const std::vector<FeatureCorrespondence>& correspondences,
    RelativePose* relative_pose, /*RAJVI -added this*/
    TwoViewInfo* twoview_info,
    std::vector<int>* inlier_indices) {
  // Normalize features w.r.t focal length.
  std::vector<FeatureCorrespondence> normalized_correspondences;
  NormalizeFeatures(intrinsics1,
                    intrinsics2,
                    correspondences,
                    &normalized_correspondences);

  // Set the ransac parameters.
  RansacParameters ransac_options;
  ransac_options.failure_probability = 1.0 - options.expected_ransac_confidence;
  ransac_options.min_iterations = options.min_ransac_iterations;
  ransac_options.max_iterations = options.max_ransac_iterations;
  ransac_options.error_thresh =
      options.max_sampson_error_pixels * options.max_sampson_error_pixels /
      (intrinsics1.focal_length.value * intrinsics2.focal_length.value);
  ransac_options.use_mle = options.use_mle;

  //RelativePose relative_pose; /*RAJVI -commented this, due to arg passed */
  RansacSummary summary;
  if (!EstimateRelativePose(ransac_options,
                            options.ransac_type,
                            normalized_correspondences,
                            relative_pose,
                            &summary)) {
    return false;
  }

  AngleAxisd rotation(relative_pose->rotation);

  // Set the twoview info.
  twoview_info->rotation_2 = rotation.angle() * rotation.axis();
  twoview_info->position_2 = relative_pose->position;
  twoview_info->focal_length_1 = intrinsics1.focal_length.value;
  twoview_info->focal_length_2 = intrinsics2.focal_length.value;
  twoview_info->num_verified_matches = summary.inliers.size();

  *inlier_indices = summary.inliers;

  return true;
}

bool EstimateTwoViewInfoUncalibrated(
    const EstimateTwoViewInfoOptions& options,
    const CameraIntrinsicsPrior& intrinsics1,
    const CameraIntrinsicsPrior& intrinsics2,
    const std::vector<FeatureCorrespondence>& correspondences,
    TwoViewInfo* twoview_info,
    std::vector<int>* inlier_indices) {
  // Normalize features w.r.t principal point.
  std::vector<FeatureCorrespondence> centered_correspondences;
  NormalizeFeatures(intrinsics1,
                    intrinsics2,
                    correspondences,
                    &centered_correspondences);

  // Set the ransac parameters.
  RansacParameters ransac_options;
  ransac_options.failure_probability = 1.0 - options.expected_ransac_confidence;
  ransac_options.min_iterations = options.min_ransac_iterations;
  ransac_options.max_iterations = options.max_ransac_iterations;
  ransac_options.error_thresh =
      options.max_sampson_error_pixels * options.max_sampson_error_pixels;

  UncalibratedRelativePose relative_pose;
  RansacSummary summary;
  if (!EstimateUncalibratedRelativePose(ransac_options,
                                        options.ransac_type,
                                        centered_correspondences,
                                        &relative_pose,
                                        &summary)) {
    return false;
  }

  AngleAxisd rotation(relative_pose.rotation);

  // Set the twoview info.
  twoview_info->rotation_2 = rotation.angle() * rotation.axis();
  twoview_info->position_2 = relative_pose.position;
  twoview_info->focal_length_1 = relative_pose.focal_length1;
  twoview_info->focal_length_2 = relative_pose.focal_length2;

  // Get the number of verified features.
  twoview_info->num_verified_matches = summary.inliers.size();
  *inlier_indices = summary.inliers;

  return true;
}

}  // namespace

bool EstimateTwoViewInfo(
    const EstimateTwoViewInfoOptions& options,
    const CameraIntrinsicsPrior& intrinsics1,
    const CameraIntrinsicsPrior& intrinsics2,
    const std::vector<FeatureCorrespondence>& correspondences,
    RelativePose* relative_pose, /*RAJVI -added this*/
    TwoViewInfo* twoview_info,
    std::vector<int>* inlier_indices) {
  CHECK_NOTNULL(twoview_info);
  CHECK_NOTNULL(inlier_indices)->clear();

  // Case where both views are calibrated.
  if (intrinsics1.focal_length.is_set && intrinsics2.focal_length.is_set) {
    return EstimateTwoViewInfoCalibrated(options,
                                         intrinsics1,
                                         intrinsics2,
                                         correspondences,
                                         relative_pose, /*RAJVI added this*/
                                         twoview_info,
                                         inlier_indices);
  }

  // Only one of the focal lengths is set.
  if (intrinsics1.focal_length.is_set || intrinsics2.focal_length.is_set) {
    LOG(WARNING) << "Solving for two view infos when exactly one view is "
                    "calibrated has not been implemented yet. Treating both "
                    "views as uncalibrated instead.";
    return EstimateTwoViewInfoUncalibrated(options,
                                           intrinsics1,
                                           intrinsics2,
                                           correspondences,
                                           twoview_info,
                                           inlier_indices);
  }

  // Assume both views are uncalibrated.
  return EstimateTwoViewInfoUncalibrated(options,
                                         intrinsics1,
                                         intrinsics2,
                                         correspondences,
                                         twoview_info,
                                         inlier_indices);
}

}  // namespace theia
