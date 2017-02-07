#include "defs.h"
//#include "keys2a.h"
#include "Reader.h"
#include "Drawings.h"
#include "PairwiseInfoWithPoints.h"
#include "ViewInfoWithPoints.h"
#include "ReadBundlerMatchGraph.h"
#include "EstimatePairwiseInfo.h"
#include "EvaluateTripletsForRotationLoop.h"
#include "ComputeOptimizationCosts.h"
#include "EvaluateRelativePoseError.h"
#include "Util.h"
#include "FilterViewGraph.h"
/*
#include "EstimateSecondaryGeometry.h"
*/
#include <theia/theia.h>


int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);

  string baseDirectory = string(argv[1]);
  string paramPrefix = string(argv[2]);

  string solFileName = baseDirectory + "/output/" + paramPrefix + "_flow_" + argv[3] + "_solution.txt";
  string viewgraphFile = baseDirectory + "/output/" + paramPrefix + "_flow_" + argv[3] + "_view_graph.bin";

  int numViews = atoi(argv[4]);
  int numPairs = atoi(argv[5]);

  vector< ViewInfoWithPoints > views(numViews);
  vector< PairwiseInfoWithPoints > twoViewInfoVec(numPairs);

  for(int i=0; i < views.size(); i++) {
    ViewInfoWithPoints& currView =  views[i];
    stringstream ss;
    ss << baseDirectory << "/binary_tmp/view-" << i <<".bin";
    ReadViewInfoFromDisk( ss.str(), &currView );
  }

  std::unordered_map<theia::ViewIdPair, int> edge2indexMap;
  for(int i=0; i < twoViewInfoVec.size(); i++) { 
    PairwiseInfoWithPoints& pair = twoViewInfoVec[i];
    stringstream ss;
    ss << baseDirectory << "/binary_tmp/pair-" << i <<".bin";
    ReadPairwiseInfoFromDisk( ss.str(), &pair );

    theia::ViewIdPair view_pair(pair.image1, pair.image2);
    edge2indexMap.insert(make_pair(view_pair, i));
  }

  ifstream solFile( solFileName , ifstream::in);
  if(!solFile.is_open()) {
    printf("\nSolution file does not exist");
    fflush(stdout);
  }

  unordered_map<string,int> imageName2ViewId;
  for(int i=0; i < views.size(); i++) {
    imageName2ViewId.insert(make_pair(views[i].fullImageName,i));
  }

  vector<bool> selectedImages( views.size() , false);
  vector<bool> selectedPairs( twoViewInfoVec.size() , false);

  int numSelImg=0;
  int numSelPair=0;

  string line;
  while(getline(solFile, line)) {
    stringstream ss(line);
    string string1;
    string string2;
    double flow;
    double cost;

    ss >> string1 >> string2 >> flow >> cost;
    
    if(string1 == "source" || string1 == "sink") {
      continue;
    }

    if(string1 == "image") {
      if(flow > 0.0f) {
        int imIdx = imageName2ViewId[string2];
        selectedImages[imIdx] = true;
        numSelImg++;
      }
      continue;
    } 

    int imIdx1 = imageName2ViewId[string1];
    int imIdx2 = imageName2ViewId[string2];

    int pIdx = edge2indexMap[make_pair(imIdx1, imIdx2)];
    if(flow > 0.0) {
      selectedPairs[pIdx] = true;
      numSelPair++;
      if(!selectedImages[imIdx1] | !selectedImages[imIdx2]) {
        printf("\nSome problem with the network - nodes unselected pair selected");
        fflush(stdout);
      }
    }
  }

  printf("\nSelected %d images %d pairs", numSelImg, numSelPair);

  vector<string> view_names;
  vector<theia::CameraIntrinsicsPrior> camPriors;
  for(int i=0; i < views.size(); i++) {
    if(selectedImages[i]) {
      view_names.push_back( views[i].viewObj.Name() );
      camPriors.push_back( views[i].viewObj.CameraIntrinsicsPrior() );
    }
  }
  printf("\nAdded all images");
  fflush(stdout);

   vector<theia::ImagePairMatch> matches;
  // Create a subset view-graph
  for (int i=0; i < twoViewInfoVec.size(); i++) {
    if(selectedPairs[i]) {
      PairwiseInfoWithPoints pairwise = twoViewInfoVec[i];

      theia::ImagePairMatch imgPair;
      imgPair.image1 = views[pairwise.image1].viewObj.Name();
      imgPair.image2 = views[pairwise.image2].viewObj.Name();

      imgPair.twoview_info = pairwise.primaryMotion;
      CreateCorrespondencesFromIndexedMatches( views[pairwise.image1], 
          views[pairwise.image2], pairwise.primaryVerifiedMatches, &imgPair.correspondences);


      matches.push_back(imgPair);
    }
  }

  bool vgStatus = WriteMatchesAndGeometry(viewgraphFile, view_names, camPriors, matches);
  if(vgStatus) {
    printf("\nSuccessfully wrote view-graph");
    fflush(stdout);
  } else {
    printf("\nCould not write view-graph");
    fflush(stdout);
  }

  return 0;
}
