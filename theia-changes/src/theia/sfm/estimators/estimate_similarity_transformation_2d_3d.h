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
//     * Neither the name of The Regents or University of California,
//       nor the names of its contributors may be used to endorse or promote
//       products derived from this software without specific prior written
//       permission.
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

#ifndef THEIA_SFM_ESTIMATORS_ESTIMATE_SIMILARITY_TRANSFORMATION_2D_3D_H_
#define THEIA_SFM_ESTIMATORS_ESTIMATE_SIMILARITY_TRANSFORMATION_2D_3D_H_

#include <Eigen/Core>
#include <vector>

#include "theia/sfm/camera/camera.h"
#include "theia/sfm/create_and_initialize_ransac_variant.h"
#include "theia/sfm/estimators/feature_correspondence_2d_3d.h"
#include "theia/sfm/feature.h"
#include "theia/sfm/similarity_transformation.h"

namespace theia {

struct RansacParameters;
struct RansacSummary;

// A struct to manage the correspondences between 3d points and the respective
// camera and feature observation.
struct CameraAndFeatureCorrespondence2D3D {
  // A camera with known pose (i.e., extrinsics) and intrinsics.
  Camera camera;

  // A feature observation in pixels.
  Feature observation;

  // The homogeneous 3D point.
  Eigen::Vector4d point3d;
};

// Estimates the similarity transformation that aligns the projection of the 3D
// points to the 2D features observed by the corresponding camera. This assumes
// that there are multiple cameras observing the 3D points, but the cameras are
// in a different coordinate system than the 3D points. This is the case, for
// instance, if you have known camera poses from SLAM and want to align them to
// a known SfM reconstruction. The SLAM poses are in a different coordinate
// system than the SfM reconstruction and so a 7 d.o.f. similarity
// transformation must be applied to align the coordinate systems. Please cite
// the paper "gDLS: A Scalable Solution to the Generalized Pose And Scale
// Problem" by Sweeney et al (ECCV 2014) when using this algorithm.
//
// The error threshold in the ransac parameters should correspond to square
// reprojection error.
bool EstimateSimilarityTransformation2D3D(
    const RansacParameters& ransac_params,
    const RansacType& ransac_type,
    const std::vector<CameraAndFeatureCorrespondence2D3D>& correspondences,
    SimilarityTransformation* similarity_transformation,
    RansacSummary* ransac_summary);

}  // namespace theia

#endif  // THEIA_SFM_ESTIMATORS_ESTIMATE_SIMILARITY_TRANSFORMATION_2D_3D_H_
