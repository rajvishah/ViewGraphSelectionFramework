#include "EvaluateTripletsForRotationLoop.h"
#include "PairwiseInfoWithPoints.h"

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
#include "defs.h"
//#include <ceres/rotation.h>
//#include <Eigen/Core>
#include <unordered_map>
//#include <vector>

namespace {

double ComputeLoopRotationError(const theia::ViewTriplet& triplet) {
  // Get relative rotation matrices.
  Eigen::Matrix3d rotation1_2, rotation1_3, rotation2_3;
  ceres::AngleAxisToRotationMatrix(
      triplet.info_one_two.rotation_2.data(),
      ceres::ColumnMajorAdapter3x3(rotation1_2.data()));
  ceres::AngleAxisToRotationMatrix(
      triplet.info_one_three.rotation_2.data(),
      ceres::ColumnMajorAdapter3x3(rotation1_3.data()));
  ceres::AngleAxisToRotationMatrix(
      triplet.info_two_three.rotation_2.data(),
      ceres::ColumnMajorAdapter3x3(rotation2_3.data()));

  // Compute loop rotation.
  const Eigen::Matrix3d loop_rotation =
      rotation2_3 * rotation1_2 * rotation1_3.transpose();
  Eigen::Vector3d loop_rotation_angle_axis;
  ceres::RotationMatrixToAngleAxis(
      ceres::ColumnMajorAdapter3x3(loop_rotation.data()),
      loop_rotation_angle_axis.data());

  // Return the angle of the loop rotation which is the error of the
  // concatenated triplet rotation.
  return theia::RadToDeg(loop_rotation_angle_axis.norm());
}

/*
*/

void AccumulateTripletEdgesError(
    double rotation_error,
    const theia::ViewTriplet& triplet,
    std::unordered_map<theia::ViewIdPair, int >& edge2indexMap, 
    vector<PairwiseInfoWithPoints>& twoviewVec) {

  theia::ViewIdPair pair1(triplet.view_ids[0], triplet.view_ids[1]);
  theia::ViewIdPair pair2(triplet.view_ids[0], triplet.view_ids[2]);
  theia::ViewIdPair pair3(triplet.view_ids[1], triplet.view_ids[2]);

  int idx1 = edge2indexMap[pair1];
  int idx2 = edge2indexMap[pair2];
  int idx3 = edge2indexMap[pair3];

  twoviewVec[idx1].tripletErrorVec.push_back(rotation_error);
  twoviewVec[idx2].tripletErrorVec.push_back(rotation_error);
  twoviewVec[idx3].tripletErrorVec.push_back(rotation_error);

  if(rotation_error < 10) {
    twoviewVec[idx1].scores.tripletScore++;
    twoviewVec[idx2].scores.tripletScore++;
    twoviewVec[idx3].scores.tripletScore++; 
  }

//  twoviewVec[idx1].tripletError += rotation_error;
//  twoviewVec[idx1].tripletError += rotation_error;
//  twoviewVec[idx1].tripletError += rotation_error; 
}
}  // namespace




void EvaluateTripletsForRotationLoop(theia::ViewGraph* view_graph, 
                                     std::unordered_map<theia::ViewIdPair, int >& edge2indexMap, 
                                     vector<PairwiseInfoWithPoints>& twoviewVec) {
  const std::unordered_map<theia::ViewIdPair, theia::TwoViewInfo>& view_pairs =
      view_graph->GetAllEdges();

  // Find all triplets.
  theia::TripletExtractor triplet_extractor;
  std::vector<std::vector<theia::ViewTriplet> > connected_triplets;
  CHECK(triplet_extractor.ExtractTripletsFromViewPairs(view_pairs,
        &connected_triplets))
      << "Could not extract triplets from view pairs.";

  // Examine the cycles of size 3 to determine invalid view pairs from the
  // rotations.
  for (const auto& triplets : connected_triplets) {
    for (const theia::ViewTriplet& triplet : triplets) {
      // Compute loop rotation error.
      double loop_rotation_error_degrees =
          ComputeLoopRotationError(triplet);
      LOG(INFO) << "Loop rotation error = " << loop_rotation_error_degrees;
      AccumulateTripletEdgesError(loop_rotation_error_degrees, triplet, edge2indexMap, twoviewVec);
    }
  }

  for (const auto& triplets : connected_triplets) {
    for (const theia::ViewTriplet& triplet : triplets) {
      theia::ViewIdPair pair1(triplet.view_ids[0], triplet.view_ids[1]);
      theia::ViewIdPair pair2(triplet.view_ids[0], triplet.view_ids[2]);
      theia::ViewIdPair pair3(triplet.view_ids[1], triplet.view_ids[2]);

      int idx1 = edge2indexMap[pair1];
      int idx2 = edge2indexMap[pair2];
      int idx3 = edge2indexMap[pair3];

      twoviewVec[idx1].connectedTripletEdges.push_back( make_pair(idx2, idx3) );
      twoviewVec[idx2].connectedTripletEdges.push_back( make_pair(idx1, idx3) );
      twoviewVec[idx3].connectedTripletEdges.push_back( make_pair(idx1, idx2) );
    }
  }
}

