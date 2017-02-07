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
  string vsfmPairwiseFile = baseDirectory + "/output/" + paramPrefix + "_flow_" + argv[3] + "_vsfm_pairs.txt";
  string vsfmPairwiseListFile = baseDirectory + "/output/" + paramPrefix + "_flow_" + argv[3] + "_vsfm_list_pairs.txt";

  int numViews = atoi(argv[4]);
  int numPairs = atoi(argv[5]);

  string pairwiseImgPath= baseDirectory + "/img_pairs/";

  bool writePairImages = (bool)(atoi(argv[6]));

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

  ofstream pairFile( vsfmPairwiseFile, ofstream::out );
  if( !pairFile.is_open() ) {
    printf("\nCould not open pairfile");
    fflush(stdout);
  }

  ofstream pairListFile( vsfmPairwiseListFile, ofstream::out );
  if( !pairListFile.is_open() ) {
    printf("\nCould not open pairfile");
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

  // Create a subset view-graph
  for (int i=0; i < twoViewInfoVec.size(); i++) {
    if(writePairImages == false) {
      if(selectedPairs[i]) {
        PairwiseInfoWithPoints pairwise = twoViewInfoVec[i];
        int img1 = pairwise.image1;
        int img2 = pairwise.image2;

        pairListFile << views[img1].fullImageName << " " << views[img2].fullImageName << endl; 
        pairFile << views[img1].fullImageName << " " << 
          views[img2].fullImageName << " " << pairwise.putativeMatches.size() << endl;

        vector<int> matchIds1, matchIds2;
        matchIds1.resize( pairwise.putativeMatches.size() );
        matchIds2.resize( pairwise.putativeMatches.size() );

        for(int j=0; j < pairwise.putativeMatches.size(); j++) {
          matchIds1[j] = pairwise.putativeMatches[j].feature1_ind;
          matchIds2[j] = pairwise.putativeMatches[j].feature2_ind; 
        }

        for(int j=0; j < matchIds1.size(); j++) {
          pairFile << matchIds1[j] << " " ;
        }
        pairFile << endl;

        for(int j=0; j < matchIds2.size(); j++) {
          pairFile << matchIds2[j] << " "; 
        }
        pairFile << endl;
      }
    } else {
      printf("\nWriting image pair %d", i);
      PairwiseInfoWithPoints pairwise = twoViewInfoVec[i];
      int img1 = pairwise.image1;
      int img2 = pairwise.image2;

      vector< cv::Point2f > matchPointSet1, matchPointSet2;
      CreatePointSetsFromIndexedMatches( pairwise.primaryVerifiedMatches, 
          views[pairwise.image1].keysWithDesc.keypoints,
          views[pairwise.image2].keysWithDesc.keypoints,
          matchPointSet1, matchPointSet2);

      string pairImageName = pairwiseImgPath + 
        views[img1].viewObj.Name() + "_" + 
        views[img2].viewObj.Name() + ".jpg" ;

      DrawPairsWithPoints(views[img1].fullImageName, 
          views[img2].fullImageName,
          matchPointSet1,
          matchPointSet2,
          -pairwise.cost,
          pairImageName);
    }
  }
  pairFile.close();
  pairListFile.close();
  return 0;
}
