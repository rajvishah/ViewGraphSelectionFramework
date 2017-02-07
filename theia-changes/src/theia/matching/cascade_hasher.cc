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

#include "theia/matching/cascade_hasher.h"

#include <Eigen/Core>
#include <glog/logging.h>
#include <stdint.h>

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include "theia/matching/distance.h"
#include "theia/matching/feature_matcher.h"
#include "theia/matching/indexed_feature_match.h"
#include "theia/util/random.h"

namespace theia {

namespace {

void GetZeroMeanDescriptor(const std::vector<Eigen::VectorXf>& sift_desc,
                           Eigen::VectorXf* mean) {
  mean->setZero(sift_desc[0].size());
  for (int i = 0; i < sift_desc.size(); i++) {
    *mean += sift_desc[i];
  }
  *mean /= static_cast<double>(sift_desc.size());
}

// Uses the Box-Muller transforma to get a random number from a normal
// distribution.
double GetNormRand() {
  const double u1 = (rand() % 1000 + 1) / 1000.0;
  const double u2 = (rand() % 1000 + 1) / 1000.0;
  return sqrt(-2 * log(u1)) * cos(2 * acos(-1.0) * u2);
}

}  // namespace

bool CascadeHasher::Initialize(const int num_dimensions_of_descriptor) {
  num_dimensions_of_descriptor_ = num_dimensions_of_descriptor;
  primary_hash_projection_.resize(kHashCodeSize, num_dimensions_of_descriptor_);

  InitRandomGenerator();

  // Initialize primary hash projection.
  for (int i = 0; i < kHashCodeSize; i++) {
    for (int j = 0; j < num_dimensions_of_descriptor; j++) {
      primary_hash_projection_(i, j) = GetNormRand();
    }
  }

  // Initialize secondary hash projection.
  for (int i = 0; i < kNumBucketGroups; i++) {
    secondary_hash_projection_[i].resize(kNumBucketBits,
                                         num_dimensions_of_descriptor_);
    for (int j = 0; j < kNumBucketBits; j++) {
      for (int k = 0; k < num_dimensions_of_descriptor_; k++) {
        secondary_hash_projection_[i](j, k) = GetNormRand();
      }
    }
  }

  return true;
}

void CascadeHasher::CreateHashedDescriptors(
    const std::vector<Eigen::VectorXf>& sift_desc,
    HashedImage* hashed_image) const {
  for (int i = 0; i < sift_desc.size(); i++) {
    // Use the zero-mean shifted descriptor.
    const auto descriptor = sift_desc[i] - hashed_image->mean_descriptor;
    auto& hash_code = hashed_image->hashed_desc[i].hash_code;

    // Compute hash code.
    const Eigen::VectorXf primary_projection =
        primary_hash_projection_ * descriptor;
    for (int j = 0; j < kHashCodeSize; j++) {
      hash_code[j] = primary_projection(j) > 0;
    }

    // Determine the bucket index for each group.
    for (int j = 0; j < kNumBucketGroups; j++) {
      uint16_t bucket_id = 0;
      const Eigen::VectorXf secondary_projection =
          secondary_hash_projection_[j] * descriptor;

      for (int k = 0; k < kNumBucketBits; k++) {
        bucket_id = (bucket_id << 1) + (secondary_projection(k) > 0 ? 1 : 0);
      }
      hashed_image->hashed_desc[i].bucket_ids[j] = bucket_id;
    }
  }
}

void CascadeHasher::BuildBuckets(HashedImage* hashed_image) const {
  for (int i = 0; i < kNumBucketGroups; i++) {
    // Add the descriptor ID to the proper bucket group and id.
    for (int j = 0; j < hashed_image->hashed_desc.size(); j++) {
      const uint16_t bucket_id = hashed_image->hashed_desc[j].bucket_ids[i];
      hashed_image->buckets[i][bucket_id].push_back(j);
    }
  }
}

// Steps:
//   1) Get zero mean descriptor.
//   2) Compute hash code and hash buckets.
//   3) Construct buckets.
HashedImage CascadeHasher::CreateHashedSiftDescriptors(
    const std::vector<Eigen::VectorXf>& sift_desc) const {
  HashedImage hashed_image;

  // Allocate the buckets even if no descriptors exist to fill them.
  hashed_image.buckets.resize(kNumBucketGroups);
  for (int i = 0; i < kNumBucketGroups; i++) {
    hashed_image.buckets[i].resize(kNumBucketsPerGroup);
  }

  if (sift_desc.size() == 0) {
    return hashed_image;
  }

  GetZeroMeanDescriptor(sift_desc, &hashed_image.mean_descriptor);

  // Allocate space for hash codes and bucket ids.
  hashed_image.hashed_desc.resize(sift_desc.size());

  // Allocate space for each bucket id.
  for (int i = 0; i < sift_desc.size(); i++) {
    hashed_image.hashed_desc[i].bucket_ids.resize(kNumBucketGroups);
  }

  // Create hash codes for each feature.
  CreateHashedDescriptors(sift_desc, &hashed_image);

  // Build the buckets.
  BuildBuckets(&hashed_image);
  return hashed_image;
}

// Matches images with a fast matching scheme based on the hash codes
// previously generated.
void CascadeHasher::MatchImages(
    const HashedImage& hashed_image1,
    const std::vector<Eigen::VectorXf>& descriptors1,
    const HashedImage& hashed_image2,
    const std::vector<Eigen::VectorXf>& descriptors2,
    const double lowes_ratio,
    std::vector<IndexedFeatureMatch>* matches) const {
  if (descriptors1.size() == 0 || descriptors2.size() == 0) {
    return;
  }

  static const int kNumTopCandidates = 10;
  const double sq_lowes_ratio = lowes_ratio * lowes_ratio;
  L2 l2_distance;

  // Reserve space for the matches.
  matches->reserve(
      static_cast<int>(std::min(descriptors1.size(), descriptors2.size())));

  // Preallocate the candidate descriptors container.
  std::vector<int> candidate_descriptors;
  candidate_descriptors.reserve(descriptors2.size());

  // Preallocated hamming distances. Each column indicates the hamming distance
  // and the rows collect the descriptor ids with that
  // distance. num_descriptors_with_hamming_distance keeps track of how many
  // descriptors have that distance.
  Eigen::MatrixXi candidate_hamming_distances(descriptors2.size(),
                                              kHashCodeSize + 1);
  Eigen::VectorXi num_descriptors_with_hamming_distance(kHashCodeSize + 1);

  // Preallocate the container for keeping euclidean distances.
  std::vector<std::pair<float, int> > candidate_euclidean_distances;
  candidate_euclidean_distances.reserve(kNumTopCandidates);

  // A preallocated vector to determine if we have already used a particular
  // feature for matching (i.e., prevents duplicates).
  std::vector<bool> used_descriptor(descriptors2.size());
  for (int i = 0; i < hashed_image1.hashed_desc.size(); i++) {
    candidate_descriptors.clear();
    num_descriptors_with_hamming_distance.setZero();
    candidate_euclidean_distances.clear();

    const auto& hashed_desc = hashed_image1.hashed_desc[i];

    // Accumulate all descriptors in each bucket group that are in the same
    // bucket id as the query descriptor.
    for (int j = 0; j < kNumBucketGroups; j++) {
      const uint16_t bucket_id = hashed_desc.bucket_ids[j];
      for (const auto& feature_id : hashed_image2.buckets[j][bucket_id]) {
        candidate_descriptors.emplace_back(feature_id);
        used_descriptor[feature_id] = false;
      }
    }

    // Skip matching this descriptor if there are not at least 2 candidates.
    if (candidate_descriptors.size() <= kNumTopCandidates) {
      continue;
    }

    // Compute the hamming distance of all candidates based on the comp hash
    // code. Put the descriptors into buckets corresponding to their hamming
    // distance.
    for (const int candidate_id : candidate_descriptors) {
      if (used_descriptor[candidate_id]) {
        continue;
      }
      used_descriptor[candidate_id] = true;
      const uint8_t hamming_distance =
          (hashed_desc.hash_code ^
           hashed_image2.hashed_desc[candidate_id].hash_code).count();
      candidate_hamming_distances(
          num_descriptors_with_hamming_distance(hamming_distance)++,
          hamming_distance) = candidate_id;
    }

    // Compute the euclidean distance of the k descriptors with the best hamming
    // distance.
    candidate_euclidean_distances.reserve(kNumTopCandidates);
    for (int j = 0; j < candidate_hamming_distances.cols(); j++) {
      for (int k = 0; k < num_descriptors_with_hamming_distance(j); k++) {
        const int candidate_id = candidate_hamming_distances(k, j);
        const float distance =
            l2_distance(descriptors2[candidate_id], descriptors1[i]);
        candidate_euclidean_distances.emplace_back(distance, candidate_id);
        if (candidate_euclidean_distances.size() > kNumTopCandidates) {
          break;
        }
      }
      if (candidate_euclidean_distances.size() > kNumTopCandidates) {
        break;
      }
    }

    // Find the top 2 candidates based on euclidean distance.
    std::partial_sort(candidate_euclidean_distances.begin(),
                      candidate_euclidean_distances.begin() + 2,
                      candidate_euclidean_distances.end());

    // Only add to output matches if it passes the ratio test.
    if (candidate_euclidean_distances[0].first >
        candidate_euclidean_distances[1].first * sq_lowes_ratio) {
      continue;
    }

    matches->emplace_back(i,
                          candidate_euclidean_distances[0].second,
                          candidate_euclidean_distances[0].first);
  }
}

}  // namespace theia
