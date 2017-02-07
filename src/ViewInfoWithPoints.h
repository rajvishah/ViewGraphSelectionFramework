#ifndef VIEWINFOWITHPOINTS_H
#define VIEWINFOWITHPOINTS_H 
#include "defs.h"
#include "Util.h"
#include <cereal/types/utility.hpp>
#include <cereal/types/set.hpp>
class ViewInfoWithPoints {

  public:
    theia::View viewObj;

    bool connected;

    bool calibrated;
    double cost;
    
    double sharpness;
    double calibrationFitness;
    double featureFitness;
    double connectivityFitness;
    double disjointFitness;

    string fullImageName;

    set<theia::ViewId> connectedImages;
    set<FeatureId> matchedFeatureIndices;

    theia::KeypointsAndDescriptors  keysWithDesc;
    //vector< cv::Point2f > matchedFeatures;
    //vector< cv::Point2f > detectedFeatures;

    vector< set<theia::ViewId> > feature2ImageMap;

    FeatureConvexHull hull;

    bool readKeys( const string& imagePath , bool binarized);
    bool readImageInfo( const string& imagePath, theia::ExifReader& exifReader);

    double computeImageSharpness(cv::Mat& image);
    double setCalibrationFitness();

  private:
    friend class cereal::access;
    template<class Archive>
      void serialize(Archive & archive) {
        archive( fullImageName, calibrated, viewObj, featureFitness, 
            connectivityFitness, disjointFitness, 
            matchedFeatureIndices, connectedImages, keysWithDesc, 
            feature2ImageMap
            //viewObj, matchedFeatureIndices, connectedImages, 
            //featureFitness, connectivityFitness, disjointFitness
            );  
        // serialize things by passing them to the archive
      }

};
bool ReadViewInfoFromDisk( string filename, ViewInfoWithPoints* view);
bool WriteViewInfoToDisk( string filename, ViewInfoWithPoints& view);
#endif /* VIEWINFOWITHPOINTS_H */
