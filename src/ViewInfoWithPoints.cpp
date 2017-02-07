#include "ViewInfoWithPoints.h"
#include "Reader.h"
#include "keys2a.h"

bool ViewInfoWithPoints :: readKeys( const string& keyPath , bool binarized) {

  connected = false;

  string filename, directory;
  bool s1 = theia::GetFilenameFromFilepath(keyPath, false, &filename);
  bool s2 = theia::GetDirectoryFromFilepath(keyPath, &directory);
  const string binFileName = directory + "/" + filename + ".bin";

  string image_name = filename + ".jpg";
  
  theia::KeypointsAndDescriptors &kpd = keysWithDesc;
  kpd.image_name = image_name;

  if(!binarized) {
    unsigned char* keys;
    keypt_t* keysInfo;

    int numFeatures = ReadKeyFile(keyPath.c_str(), &keys, &keysInfo);
    if(numFeatures == 0) {
      printf("\nProbably Invalid Key %s", keyPath.c_str());
    }
    for(int n=0; n < numFeatures; n++) {
      double x1 = keysInfo[n].x;
      double y1 = keysInfo[n].y;

      theia::Keypoint kp(x1, y1, theia::Keypoint::SIFT);
      Eigen::VectorXf desc(128);
      for(int d=0; d < 128; d++) {
        desc[d] = *(keys + (128*n) +d); 
      }
      
      kpd.keypoints.push_back(kp);
      kpd.descriptors.push_back(desc);
    }
    bool status = theia::WriteKeypointsAndDescriptors(binFileName, kpd.keypoints, kpd.descriptors);
  } else { 
    bool status = theia::ReadKeypointsAndDescriptors(keyPath, &kpd.keypoints, &kpd.descriptors); 
    printf("\nProblem with reading serialized points and descriptors file %s", image_name.c_str());
    fflush(stdout);

  }

  /*
  for(int i=0; i < kpd.keypoints.size(); i++) { 
    float x1 = kpd.keypoints[i].x();
    float y1 = kpd.keypoints[i].y();
    detectedFeatures.push_back(cv::Point2f(x1,y1)); 
  }

  feature2ImageMap.resize( kpd.keypoints.size() );
  */

  return true;
}


double ViewInfoWithPoints :: computeImageSharpness(cv::Mat& image) {
  cv::Mat grad_x, grad_y, grad_x2, grad_y2;
  cv::Sobel( image, grad_x, CV_32F, 1, 0, 3);
  cv::Sobel( image, grad_y, CV_32F, 0, 1, 3);

  grad_x2 = grad_x.mul(grad_x);
  grad_y2 = grad_y.mul(grad_y);

  float x_sum = (float)(cv::sum(grad_x2)[0]);
  float y_sum = (float)(cv::sum(grad_y2)[0]);

  cv::Size sz = image.size();
  double sharpness = sqrt((x_sum + y_sum))/(sz.width*sz.height);

  return sharpness;
}


bool ViewInfoWithPoints :: readImageInfo( const string& imagePath, 
    theia::ExifReader& exifReader) {

  fullImageName = imagePath;

  theia::CameraIntrinsicsPrior* prior = viewObj.MutableCameraIntrinsicsPrior();
  exifReader.ExtractEXIFMetadata(imagePath, prior);

  if (!(*prior).focal_length.is_set) {
    (*prior).focal_length.is_set = true;
    (*prior).focal_length.value =
      1.2 * static_cast<double>(
          std::max((*prior).image_width, (*prior).image_height));
    calibrated = false;
  }

  calibrated = true;

//  camera_intrinsics_prior_ = prior;
}

bool WriteViewInfoToDisk( string filename, ViewInfoWithPoints& view) {
  std::ofstream view_writer(filename, std::ios::out | std::ios::binary);
  if (!view_writer.is_open()) {
    LOG(ERROR) << "Could not open the matches file: " << filename
      << " for writing.";
    return false;
  }
  // Make sure that Cereal is able to finish executing before returning.
  {
    cereal::PortableBinaryOutputArchive output_archive(view_writer);
    output_archive(view);
  }

  return true;
}

bool ReadViewInfoFromDisk( string filename, ViewInfoWithPoints* view) {

  std::ifstream view_reader(filename, std::ios::in | std::ios::binary);
  if (!view_reader.is_open()) {
    LOG(ERROR) << "Could not open the matches file: " << filename
      << " for reading.";
    return false;
  }
  // Make sure that Cereal is able to finish executing before returning.
  {
    cereal::PortableBinaryInputArchive input_archive(view_reader);
    input_archive(*view);
  }

  return true;
}

