// Copyright (C) 2015 The Regents of the University of California (Regents).
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

#include <ceres/rotation.h>
#include <Eigen/Core>
#include <Eigen/LU>
#include <glog/logging.h>
#include <algorithm>

#include "gtest/gtest.h"
#include "theia/math/util.h"
#include "theia/util/random.h"
#include "theia/sfm/bundle_adjustment/optimize_relative_position_with_known_rotation.h"
#include "theia/sfm/camera/camera.h"
#include "theia/matching/feature_correspondence.h"
#include "theia/sfm/pose/test_util.h"

namespace theia {

namespace {

Camera RandomCamera() {
  Camera camera;
  camera.SetPosition(Eigen::Vector3d::Random());
  camera.SetOrientationFromAngleAxis(0.2 * Eigen::Vector3d::Random());
  camera.SetImageSize(1000, 1000);
  camera.SetFocalLength(800);
  camera.SetAspectRatio(1.0);
  camera.SetSkew(0.0);
  camera.SetPrincipalPoint(500.0, 500.0);
  return camera;
}

void GetInverseCalibrationMatrix(const Camera& camera,
                                 Eigen::Matrix3d* inv_calibration) {
  Eigen::Matrix3d calibration;
  camera.GetCalibrationMatrix(&calibration);
  bool invertible;
  double determinant;
  calibration.computeInverseAndDetWithCheck(*inv_calibration,
                                            determinant,
                                            invertible);
  if (!invertible) {
    LOG(FATAL) << "Calibration matrices are ill formed. Cannot optimize "
                  "epipolar constraints.";
    return;
  }
}

void GetRelativeTranslationFromCameras(const Camera& camera1,
                                       const Camera& camera2,
                                       Eigen::Vector3d* relative_position) {
  const Eigen::Vector3d rotated_relative_position =
      camera2.GetPosition() - camera1.GetPosition();
  ceres::AngleAxisRotatePoint(camera1.GetOrientationAsAngleAxis().data(),
                              rotated_relative_position.data(),
                              relative_position->data());
  relative_position->normalize();
}

void TestOptimization(const Camera& camera1,
                      const Camera& camera2,
                      const std::vector<Eigen::Vector3d>& world_points,
                      const double kPixelNoise,
                      const double kTranslationNoise,
                      const double kTolerance) {
  Eigen::Matrix3d inv_calibration1, inv_calibration2;
  GetInverseCalibrationMatrix(camera1, &inv_calibration1);
  GetInverseCalibrationMatrix(camera2, &inv_calibration2);

  // Project points and create feature correspondences.
  std::vector<FeatureCorrespondence> matches;
  for (int i = 0; i < world_points.size(); i++) {
    const Eigen::Vector4d point = world_points[i].homogeneous();
    FeatureCorrespondence match;
    camera1.ProjectPoint(point, &match.feature1);
    camera2.ProjectPoint(point, &match.feature2);
    AddNoiseToProjection(kPixelNoise, &match.feature1);
    AddNoiseToProjection(kPixelNoise, &match.feature2);

    // Undo the calibration.
    match.feature1 =
        (inv_calibration1 * match.feature1.homogeneous()).eval().hnormalized();
    match.feature2 =
        (inv_calibration2 * match.feature2.homogeneous()).eval().hnormalized();
    matches.emplace_back(match);
  }

  Eigen::Vector3d relative_position;
  GetRelativeTranslationFromCameras(camera1, camera2, &relative_position);

  const Eigen::Vector3d gt_relative_position = relative_position;

  // Add noise to relative translation.
  const Eigen::AngleAxisd translation_noise(DegToRad(
      RandGaussian(0.0, kTranslationNoise)), Eigen::Vector3d::Random());
  relative_position = translation_noise * relative_position;

  CHECK(OptimizeRelativePositionWithKnownRotation(
      matches,
      camera1.GetOrientationAsAngleAxis(),
      camera2.GetOrientationAsAngleAxis(),
      &relative_position));

  const double translation_error = RadToDeg(
      acos(Clamp(gt_relative_position.dot(relative_position), -1.0, 1.0)));
  EXPECT_LT(translation_error, kTolerance)
      << "GT Position = " << gt_relative_position.transpose()
      << "\nEstimated position = " << relative_position.transpose();
}

}  // namespace

TEST(OptimizeRelativePositionWithKnownRotationTest, NoNoise) {
  static const double kTolerance = 1e-6;
  static const double kPixelNoise = 0.0;
  static const double kTranslationNoise = 0.0;
  static const int kNumPoints = 100;
  std::vector<Eigen::Vector3d> points(kNumPoints);

  // Set up random points.
  InitRandomGenerator();
  for (int i = 0; i < kNumPoints; i++) {
    Eigen::Vector3d point(RandDouble(-2.0, 2.0),
                          RandDouble(-2.0, -2.0),
                          RandDouble(8.0, 10.0));
    points[i] = point;
  }

  // Set up random cameras.
  Camera camera1 = RandomCamera();
  Camera camera2 = RandomCamera();
  camera2.SetPosition(camera2.GetPosition().normalized());
  TestOptimization(camera1, camera2, points, kPixelNoise, kTranslationNoise,
                   kTolerance);
}

TEST(OptimizeRelativePositionWithKnownRotationTest, PixelNoise) {
  static const double kTolerance = 2.0;
  static const double kPixelNoise = 1.0;
  static const double kTranslationNoise = 0.0;
  static const int kNumPoints = 100;
  std::vector<Eigen::Vector3d> points(kNumPoints);

  // Set up random points.
  InitRandomGenerator();
  for (int i = 0; i < kNumPoints; i++) {
    Eigen::Vector3d point(RandDouble(-2.0, 2.0),
                          RandDouble(-2.0, -2.0),
                          RandDouble(8.0, 10.0));
    points[i] = point;
  }

  // Set up random cameras.
  Camera camera1 = RandomCamera();
  Camera camera2 = RandomCamera();
  camera2.SetPosition(camera2.GetPosition().normalized());
  TestOptimization(camera1, camera2, points, kPixelNoise, kTranslationNoise,
                   kTolerance);
}


TEST(OptimizeRelativePositionWithKnownRotationTest, TranslationNoise) {
  static const double kTolerance = 2.0;
  static const double kPixelNoise = 0.0;
  static const double kTranslationNoise = 5.0;
  static const int kNumPoints = 100;
  std::vector<Eigen::Vector3d> points(kNumPoints);

  // Set up random points.
  InitRandomGenerator();
  for (int i = 0; i < kNumPoints; i++) {
    Eigen::Vector3d point(RandDouble(-2.0, 2.0),
                          RandDouble(-2.0, -2.0),
                          RandDouble(8.0, 10.0));
    points[i] = point;
  }

  // Set up random cameras.
  Camera camera1 = RandomCamera();
  Camera camera2 = RandomCamera();
  camera2.SetPosition(camera2.GetPosition().normalized());
  TestOptimization(camera1, camera2, points, kPixelNoise, kTranslationNoise,
                   kTolerance);
}


TEST(OptimizeRelativePositionWithKnownRotationTest, PixelAndTranslationNoise) {
  static const double kTolerance = 5.0;
  static const double kPixelNoise = 1.0;
  static const double kTranslationNoise = 5.0;
  static const int kNumPoints = 100;
  std::vector<Eigen::Vector3d> points(kNumPoints);

  // Set up random points.
  InitRandomGenerator();
  for (int i = 0; i < kNumPoints; i++) {
    Eigen::Vector3d point(RandDouble(-2.0, 2.0),
                          RandDouble(-2.0, -2.0),
                          RandDouble(8.0, 10.0));
    points[i] = point;
  }

  // Set up random cameras.
  Camera camera1 = RandomCamera();
  Camera camera2 = RandomCamera();
  camera2.SetPosition(camera2.GetPosition().normalized());
  TestOptimization(camera1, camera2, points, kPixelNoise, kTranslationNoise,
                   kTolerance);
}

}  // namespace theia
