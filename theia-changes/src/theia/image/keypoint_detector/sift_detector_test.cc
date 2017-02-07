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

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <string>
#include "gtest/gtest.h"

#include "theia/image/image.h"
#include "theia/image/keypoint_detector/keypoint.h"
#include "theia/image/keypoint_detector/sift_detector.h"
#include "theia/util/random.h"

DEFINE_string(test_img, "image/keypoint_detector/img1.png",
              "Name of test image file.");

namespace theia {
std::string img_filename = THEIA_DATA_DIR + std::string("/") + FLAGS_test_img;

TEST(SiftDetector, Sanity) {
  FloatImage input_img(img_filename);

  // Get the keypoints our way.
  SiftDetector sift_detector;
  ASSERT_TRUE(sift_detector.Initialize());
  std::vector<Keypoint> sift_keypoints;
  ASSERT_TRUE(sift_detector.DetectKeypoints(input_img, &sift_keypoints));
}

TEST(SiftDetector, DifferentImageSizes) {
  FloatImage input_img(img_filename);

  // Use the same extractor for both images.
  SiftDetector sift_extractor;

  std::vector<Keypoint> keypoints;
  EXPECT_TRUE(sift_extractor.DetectKeypoints(input_img, &keypoints));

  // Get the second image, which is a different size!
  const std::string img2_filename =
      THEIA_DATA_DIR + std::string("/image/test1.jpg");
  FloatImage input_img2(img2_filename);
  keypoints.clear();
  EXPECT_TRUE(sift_extractor.DetectKeypoints(input_img2, &keypoints));
}

}  // namespace theia
