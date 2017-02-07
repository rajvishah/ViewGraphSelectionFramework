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

#ifndef THEIA_IMAGE_DESCRIPTOR_CREATE_DESCRIPTOR_EXTRACTOR_H_
#define THEIA_IMAGE_DESCRIPTOR_CREATE_DESCRIPTOR_EXTRACTOR_H_

#include <memory>

#include "theia/image/descriptor/descriptor_extractor.h"
#include "theia/image/keypoint_detector/sift_parameters.h"

namespace theia {

// The various types of feature descriptors you can choose. We use the default
// keypoint extractor for each feature type. Since this is a convenience class
// anyways, this functionality is acceptable. If more flexibility (custom
// features and custom descriptors) is needed then a new class may be developed.
enum class DescriptorExtractorType {
  SIFT = 0,
};

// Options for creating the feature extractor.
struct CreateDescriptorExtractorOptions {
  DescriptorExtractorType descriptor_extractor_type =
      DescriptorExtractorType::SIFT;

  SiftParameters sift_options;
};

// Factory method to create the keypoint detector and descriptor extractor.
std::unique_ptr<DescriptorExtractor> CreateDescriptorExtractor(
    const CreateDescriptorExtractorOptions& options);

}  // namespace theia

#endif  // THEIA_IMAGE_DESCRIPTOR_CREATE_DESCRIPTOR_EXTRACTOR_H_
