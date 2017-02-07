// Copyright (C) 2013 The Regents of the University of California (Regents).
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

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <glog/logging.h>
#include "gtest/gtest.h"

#include "theia/math/util.h"
#include "theia/util/random.h"
#include "theia/sfm/pose/four_point_homography.h"
#include "theia/sfm/pose/test_util.h"

namespace theia {
namespace {
using Eigen::AngleAxisd;
using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector2d;
using Eigen::Vector3d;

// Creates a test scenario from ground truth 3D points and ground truth rotation
// and translation. Projection (i.e., image) noise is optional (set to 0 for no
// noise). The fundamental matrix is computed to ensure that the reprojection
// errors are sufficiently small.
void GenerateImagePoints(const std::vector<Vector3d>& points_3d,
                         const double projection_noise_std_dev,
                         const Quaterniond& expected_rotation,
                         const Vector3d& expected_translation,
                         std::vector<Vector2d>* image_1_points,
                         std::vector<Vector2d>* image_2_points) {
  image_1_points->reserve(points_3d.size());
  image_2_points->reserve(points_3d.size());
  for (int i = 0; i < points_3d.size(); i++) {
    image_1_points->push_back(points_3d[i].hnormalized());
    image_2_points->push_back((expected_rotation * points_3d[i] +
                               expected_translation).hnormalized());
  }

  if (projection_noise_std_dev) {
    for (int i = 0; i < points_3d.size(); i++) {
      AddNoiseToProjection(projection_noise_std_dev, &((*image_1_points)[i]));
      AddNoiseToProjection(projection_noise_std_dev, &((*image_2_points)[i]));
    }
  }
}

// Check that the symmetric error is small. NOTE: this is a different error than
// the reprojection error.
void CheckSymmetricError(const std::vector<Vector2d>& image_1_points,
                         const std::vector<Vector2d>& image_2_points,
                         const Matrix3d& homography_matrix,
                         const double max_symmetric_error) {
  const Matrix3d inv_homography = homography_matrix.inverse();
  for (int i = 0; i < image_1_points.size(); i++) {
    const Vector3d image_1_hat =
        inv_homography * image_2_points[i].homogeneous();
    const Vector3d image_2_hat =
        homography_matrix * image_1_points[i].homogeneous();
    // Compute reprojection error.
    const double img_1_error =
        (image_1_points[i] - image_1_hat.hnormalized()).squaredNorm();
    const double img_2_error =
        (image_2_points[i] - image_2_hat.hnormalized()).squaredNorm();

    EXPECT_LT(img_1_error, max_symmetric_error);
    EXPECT_LT(img_2_error, max_symmetric_error);
  }
}

// Run a test for the homography with at least 4 points.
void FourPointHomographyWithNoiseTest(const std::vector<Vector3d>& points_3d,
                                      const double projection_noise_std_dev,
                                      const Quaterniond& expected_rotation,
                                      const Vector3d& expected_translation,
                                      const double kMaxSymmetricError) {
  InitRandomGenerator();
  std::vector<Vector2d> image_1_points;
  std::vector<Vector2d> image_2_points;
  GenerateImagePoints(points_3d, projection_noise_std_dev, expected_rotation,
                      expected_translation, &image_1_points, &image_2_points);
  // Compute homography matrix.
  Matrix3d homography_matrix;
  EXPECT_TRUE(FourPointHomography(image_1_points,
                                  image_2_points,
                                  &homography_matrix));

  CheckSymmetricError(image_1_points, image_2_points, homography_matrix,
                         kMaxSymmetricError);
}

void BasicTest() {
  const std::vector<Vector3d> points_3d = {
    Vector3d(-1.0, 3.0, 3.0),
    Vector3d(1.0, -1.0, 2.0),
    Vector3d(-1.0, 1.0, 2.0),
    Vector3d(2.0, 1.0, 3.0),
  };

  const Quaterniond soln_rotation(
      AngleAxisd(DegToRad(13.0), Vector3d(0.0, 0.0, 1.0)));
  const Vector3d soln_translation(0.0, 0.0, 0.0);
  const double kNoise = 0.0 / 512.0;
  const double kMaxSymmetricError = 1e-12;

  FourPointHomographyWithNoiseTest(
      points_3d, kNoise, soln_rotation, soln_translation,
      kMaxSymmetricError);
}

TEST(FourPointHomography, BasicTest) {
  BasicTest();
}

TEST(FourPointHomography, NoiseTest) {
  const std::vector<Vector3d> points_3d = {
    Vector3d(-1.0, 3.0, 3.0),
    Vector3d(1.0, -1.0, 2.0),
    Vector3d(-1.0, 1.0, 2.0),
    Vector3d(2.0, 1.0, 3.0),
  };

  const Quaterniond soln_rotation(
      AngleAxisd(DegToRad(13.0), Vector3d(0.0, 0.0, 1.0)));
  const Vector3d soln_translation(0.0, 0.0, 0.0);
  const double kNoise = 1.0 / 512.0;
  const double kMaxSymmetricError = 1e-4;

  FourPointHomographyWithNoiseTest(points_3d, kNoise, soln_rotation,
                                    soln_translation, kMaxSymmetricError);
}

TEST(FourPointHomography, PlanarPoints) {
  const std::vector<Vector3d> points_3d = {
    Vector3d(-1.0, 3.0, 5.0),
    Vector3d(1.0, -1.0, 5.0),
    Vector3d(-1.0, 1.0, 5.0),
    Vector3d(2.0, 1.0, 5.0),
  };

  const Quaterniond soln_rotation(
      AngleAxisd(DegToRad(13.0), Vector3d(0.0, 0.0, 1.0)));
  const Vector3d soln_translation(1.0, -0.5, -1.0);
  const double kNoise = 1.0 / 512.0;
  const double kMaxSymmetricError = 1e-4;

  FourPointHomographyWithNoiseTest(points_3d, kNoise, soln_rotation,
                                    soln_translation, kMaxSymmetricError);
}

void ManyPointsTest() {
  const Quaterniond soln_rotation(
      AngleAxisd(DegToRad(13.0), Vector3d(0.0, 0.0, 1.0)));
  const Vector3d soln_translation(0.0, 0.0, 0.0);
  const double kNoise = 1.0 / 512.0;
  const double kMaxSymmetricError = 1e-4;
  const int num_points = 100;

  std::vector<Vector3d> points_3d(num_points);
  for (int j = 0; j < num_points; j++) {
    points_3d[j] = Vector3d(RandDouble(-2.0, 2.0),
                            RandDouble(-2.0, 2.0),
                            RandDouble(1.0, 5.0));
  }

  FourPointHomographyWithNoiseTest(points_3d, kNoise, soln_rotation,
                                   soln_translation, kMaxSymmetricError);
}

TEST(FourPointHomography, ManyPoints) {
  ManyPointsTest();
}

}  // namespace
}  // namespace theia
