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
#include "Util.h"
#include "FilterViewGraph.h"
/*
#include "EstimateSecondaryGeometry.h"
*/
#include <theia/theia.h>

bool FixViewsAndPairs( vector<ViewInfoWithPoints>& views ,
    vector<PairwiseInfoWithPoints>& twoviewInfo ) {

  vector<bool> isViewInEdge( views.size() , false);
  unordered_map<int, int> viewIndices;

  for(int i=0; i < twoviewInfo.size(); i++) {
    PairwiseInfoWithPoints& pairwise = twoviewInfo[i];
    if(pairwise.viewGraphEdgeIntialized) {
      isViewInEdge[pairwise.image1] = true;
      isViewInEdge[pairwise.image2] = true;
    }
  }

  vector<ViewInfoWithPoints> prunedViews;
  vector<PairwiseInfoWithPoints> prunedPairs;

  int numAddedViews = 0;
  for(int i=0; i < views.size(); i++) {
    if(isViewInEdge[i]) {
      numAddedViews++;
      ViewInfoWithPoints currView = views[i];
      prunedViews.push_back(currView);
      viewIndices.insert(make_pair(i, prunedViews.size() - 1));
    } 
  }

  printf("\nNum of views found in edges %d", numAddedViews);
  fflush(stdout);

  for(int i=0; i < twoviewInfo.size(); i++) {
    PairwiseInfoWithPoints pairwise = twoviewInfo[i];
    if(pairwise.viewGraphEdgeIntialized) {
      int img1 = pairwise.image1;
      int img2 = pairwise.image2;

      int nimg1 = viewIndices[img1];
      int nimg2 = viewIndices[img2];

      pairwise.image1 = nimg1;
      pairwise.image2 = nimg2;

      prunedPairs.push_back(pairwise);
    }
  }

  views.clear();
  twoviewInfo.clear();

  views = prunedViews;
  twoviewInfo = prunedPairs;
}

/*
void EstimateConnectivityScore( vector<PairwiseInfoWithPoints>& twoviewInfo, 
    vector<ViewInfoWithPoints>& views) {

  int numImages = views.size();
  vector< vector < bool > > connectionMat( numImages ); 
  int maxDegree = 0;
  for(int i=0; i < numImages; i++) {
    connectionMat[i].resize( numImages );
    for(int j=0; j < connectionMat[i].size(); j++) {
      connectionMat[i][j] = false;
    }
    if(views[i].degree > maxDegree) {
      maxDegree = views[i].degree;
    }
  }

  vector< vector< int > > connectionLists(numImages);

  for(int i=0; i < twoviewInfo.size(); i++) {
    int img1 = twoviewInfo[i].image1;
    int img2 = twoviewInfo[i].image2;

    connectionMat[img1][img2] = true;
    connectionMat[img2][img1] = true;

    connectionLists[img1].push_back(img2);
    connectionLists[img2].push_back(img1);
  }

  for(int i=0; i < views.size(); i++) {
    int groupConnectivity = 0;
    for(int j=0; j < connectionLists[i].size(); j++) {
      int img1 = connectionLists[i][j];
      for(int k=0; k < j; k++) {
        int img2 = connectionLists[i][k];

        if(connectionMat[img1][img2]) {
          groupConnectivity++;
        }
      }
    }

    double N = (double)(connectionLists[i].size());
    views[i].disjointFitness = (double) (groupConnectivity) / (N * (N - 1) / 2); 
    views[i].connectivityFitness = (double)(views[i].degree) / (maxDegree * 1.0f);
    views[i].featureFitness = (double)(views[i].numMatchedFeatures)/ (views[i].numFeatures);
  }

}

void CreateViewingGraph(vector<ViewInfoWithPoints>& viewInfo,
    vector<PairwiseInfoWithPoints>& twoviewInfo,
    theia::ViewGraph* viewingGraph) {

  for(int i=0; i < twoviewInfo.size(); i++) {

    PairwiseInfoWithPoints& pairwise = twoviewInfo[i];

    int img1 = pairwise.image1;
    int img2 = pairwise.image2;

    if(pairwise.viewGraphEdgeIntialized) {
      (*viewingGraph).AddEdge(pairwise.image1, pairwise.image2, pairwise.primaryMotion);
    }

  }
}


void VisualizeTripletsWithScores(vector<ViewInfoWithPoints>& viewInfo,
    vector<PairwiseInfoWithPoints>& twoviewInfo) {

  printf("\nHere.....");
  fflush(stdout);
  for(int i=0; i < twoviewInfo.size(); i++) {
    printf("\nPair %d", i);
    PairwiseInfoWithPoints& pairwise = twoviewInfo[i];
    if(!pairwise.viewGraphEdgeIntialized) {
      continue;
    }

    vector< cv::Point2f > matchPointSet11, matchPointSet12;
    CreatePointSetsFromIndexedMatches( pairwise.primaryVerifiedMatches, 
        viewInfo[pairwise.image1].keysWithDesc.keypoints,
        viewInfo[pairwise.image2].keysWithDesc.keypoints,
        matchPointSet11, matchPointSet12);

    printf("\nTotal %d triplets", pairwise.tripletErrorVec.size());
    fflush(stdout);

    for(int j=0; j < pairwise.tripletErrorVec.size(); j++) {
      printf("\nScore %f", pairwise.tripletErrorVec[j]);

      fflush(stdout);
      int pairIndex2 = pairwise.connectedTripletEdges[j].first;
      int pairIndex3 = pairwise.connectedTripletEdges[j].second;

      PairwiseInfoWithPoints& pairwise2 = twoviewInfo[pairIndex2];
      PairwiseInfoWithPoints& pairwise3 = twoviewInfo[pairIndex3];

      string firstImage = viewInfo[pairwise.image1].fullImageName;
      string secondImage = viewInfo[pairwise.image2].fullImageName;
      string thirdImage;

      vector< cv::Point2f > matchPointSet21, matchPointSet22;
      vector< cv::Point2f > matchPointSet31, matchPointSet32;
      CreatePointSetsFromIndexedMatches( pairwise2.primaryVerifiedMatches, 
          viewInfo[pairwise2.image1].keysWithDesc.keypoints,
          viewInfo[pairwise2.image2].keysWithDesc.keypoints,
          matchPointSet21, matchPointSet22);

      CreatePointSetsFromIndexedMatches( pairwise3.primaryVerifiedMatches, 
          viewInfo[pairwise3.image1].keysWithDesc.keypoints,
          viewInfo[pairwise3.image2].keysWithDesc.keypoints,
          matchPointSet31, matchPointSet32);

      DrawOutliers(viewInfo[pairwise.image1].fullImageName, 
          matchPointSet11, matchPointSet11,"Pair1 Image1");
      DrawOutliers(viewInfo[pairwise.image2].fullImageName, 
          matchPointSet12, matchPointSet12,"Pair1 Image2");

      DrawOutliers(viewInfo[pairwise2.image1].fullImageName, 
          matchPointSet21, matchPointSet21,"Pair2 Image1");
      DrawOutliers(viewInfo[pairwise2.image2].fullImageName, 
          matchPointSet22, matchPointSet22,"Pair2 Image2");

      DrawOutliers(viewInfo[pairwise3.image1].fullImageName, 
          matchPointSet31, matchPointSet31,"Pair3 Image1");
      DrawOutliers(viewInfo[pairwise3.image2].fullImageName, 
          matchPointSet32, matchPointSet32,"Pair3 Image2");

      cv::waitKey();
    }
  }
}

void VisualizePairsWithScores(vector<ViewInfoWithPoints>& viewInfo,
    vector<PairwiseInfoWithPoints>& twoviewInfo) {

  for(int i=0; i < twoviewInfo.size(); i++) {
    printf("\n%Pair %d", i);
    PairwiseInfoWithPoints& pairwise = twoviewInfo[i];
    if(!pairwise.viewGraphEdgeIntialized) {
      continue;
    }

    printf("\nPrimary matches");
    vector< cv::Point2f > matchPointSet1, matchPointSet2;
    CreatePointSetsFromIndexedMatches( pairwise.primaryVerifiedMatches, 
        viewInfo[pairwise.image1].keysWithDesc.keypoints,
        viewInfo[pairwise.image2].keysWithDesc.keypoints,
        matchPointSet1, matchPointSet2);

    printf("\nSecondary matches");

    vector< cv::Point2f > matchPointSet21, matchPointSet22;
    CreatePointSetsFromIndexedMatches( pairwise.secondaryVerifiedMatches, 
        viewInfo[pairwise.image1].keysWithDesc.keypoints,
        viewInfo[pairwise.image2].keysWithDesc.keypoints,
        matchPointSet21, matchPointSet22);

    DrawOutliers(viewInfo[pairwise.image1].fullImageName, 
        matchPointSet1, matchPointSet21,"Image1");
    DrawOutliers(viewInfo[pairwise.image2].fullImageName, 
        matchPointSet2, matchPointSet22,"Image2");

    vector<string> scoreNames(8);
    scoreNames[0] = "Ratio1";
    scoreNames[1] = "Ratio2";
    scoreNames[2] = "Has2Gm";
    scoreNames[3] = "MerMot";
    scoreNames[4] = "Angulr";
    scoreNames[5] = "Positn"; 
    scoreNames[6] = "hasHom";
    scoreNames[7] = "Homogr";

    vector<double> scoreValue(8);
    scoreValue[0] = pairwise.scores.ratio1;
    scoreValue[1] = pairwise.scores.ratio2;
    scoreValue[2] = (double)(pairwise.hasTwoGeometries);
    scoreValue[3] = (double)(pairwise.mergeMotions);
    scoreValue[4] = pairwise.scores.angularDiff;
    scoreValue[5] = pairwise.scores.positionDiff;
    scoreValue[6] = (double)(pairwise.homographyFound);
    scoreValue[7] = pairwise.scores.homographyScore;

    DrawScoreBoard( scoreNames, scoreValue );
    cv::waitKey(0);


  }
}

*/
int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);

  string baseDirectory = string(argv[1]);
  bool writePairImages = (bool)(atoi(argv[2]));

  string matchesFile = baseDirectory + "/matches.txt";
  string imgListFile = baseDirectory + "/list_images.txt";
  string keyListFile = baseDirectory + "/list_keys.txt";
  string normOptFile = baseDirectory + "/norm_options.txt";

  // Read image file names and dimensions
  reader::ImageListReader imgList(baseDirectory);
  bool s1 = imgList.read();

  // Read keyfile names
  reader::KeyListReader keyList(baseDirectory);
  bool s2 = keyList.read();

  // Check if list files are read correctly
  if(!s1 || !s2) {
    printf("\nError reading list file or key file");
    return 0;
  }

  unsigned int numImages = imgList.getNumImages();
  unsigned int numKeys = keyList.getNumKeys();

  printf("\nNumber of images %d", numImages);
  printf("\nNumber of keys %d", numKeys);

  theia::ExifReader exReader;
  vector< ViewInfoWithPoints > views( numImages );

  for(int i=0; i < numImages; i++) {
    // Get image name (abc.jpg) if full path is given
    string path = imgList.getImageName(i);
    size_t sep = path.find_last_of("\\/");
    if (sep != std::string::npos)
      path = path.substr(sep + 1, path.size() - sep - 1);
    views[i].viewObj = theia::View( path );

    views[i].readImageInfo( imgList.getImageName(i) , exReader);
    views[i].readKeys( keyList.getKeyName(i) , false);

    printf("\nRead image %d of %d", i, numImages);

  }

  vector< PairwiseInfoWithPoints > twoViewInfoVec;
  std::unordered_map<theia::ViewIdPair, int> edge2indexMap;
  bool s3 = ReadBundlerMatchGraph( matchesFile, views, twoViewInfoVec, edge2indexMap );
  printf("\nDone reading bundler files");
  fflush(stdout);

  bool s33 = EstimatePairwiseInfo( views, twoViewInfoVec );
  printf("\nDone estimating view-graph geometry");
  fflush(stdout);

  int oldNumPairs = twoViewInfoVec.size();
  int oldNumViews = views.size();

  printf("\nOld Two View Info Size %d", twoViewInfoVec.size());
  printf("\nOld View Info Size %d", views.size());

  //*****************************
  //Fix this later by better engineering
  // If too many pairs are there we write back to disk
  
  vector<int> viewIndexMap( views.size() , -1);
  int numValidImages = 0;
  int viewIndex = 0;
  for(int i=0; i < views.size(); i++) {
    if(views[i].connected) {
      viewIndexMap[i] = viewIndex;
      viewIndex++;    
      ViewInfoWithPoints& view = views[i];
      stringstream ss;
      ss << baseDirectory << "binary_tmp/view-" << numValidImages << ".bin";
      WriteViewInfoToDisk( ss.str(), view );
      numValidImages++;
    } 
  }

  int numValidPairs = 0;
  for(int i=0; i < twoViewInfoVec.size(); i++) {
    PairwiseInfoWithPoints& pairwise = twoViewInfoVec[i];
    if(pairwise.viewGraphEdgeIntialized) {
      pairwise.image1 = viewIndexMap[pairwise.image1];
      pairwise.image2 = viewIndexMap[pairwise.image2];
      stringstream ss;
      ss << baseDirectory << "/binary_tmp/pair-" << numValidPairs <<".bin";
      WritePairwiseInfoToDisk( ss.str(), pairwise );
      numValidPairs++;
    }
  }

  /// Now try to compute scores and write new files
//  bool changed = FixViewsAndPairs( views, twoViewInfoVec );

  printf("\nNew Two View Info Size %d", numValidImages);
  printf("\nNew View Info Size %d", numValidPairs);  

  fflush(stdout);

  views.clear();
  twoViewInfoVec.clear();

  views.resize( numValidImages );
  twoViewInfoVec.resize( numValidPairs );

  for(int i=0; i < numValidImages; i++) {
    ViewInfoWithPoints& currView = views[i];
    stringstream ss;
    ss << baseDirectory << "/binary_tmp/view-" << i <<".bin";
    ReadViewInfoFromDisk( ss.str(), &currView );
  }

  printf("\nRead back all images");

  for(int i=0; i < numValidPairs; i++) {
    PairwiseInfoWithPoints& pairwise = twoViewInfoVec[i];
    stringstream ss;
    ss << baseDirectory << "/binary_tmp/pair-" << i <<".bin";
    ReadPairwiseInfoFromDisk( ss.str(), &pairwise ); 
  }

  printf("\nRead back all pairs");


  bool viewEvalStatus = EvaluateViewFitness( views, twoViewInfoVec);

  /* Pairwise Priors Added Here */

  bool s4 = ComputeTwoGeometryScores( views, twoViewInfoVec ); 
  printf("\nRead two geometry scores");
  fflush(stdout);

  bool s5 = ComputeOverlapRatioScores( views, twoViewInfoVec ); 
  printf("\nRead ratio scores");
  fflush(stdout);

  bool s6 = ComputeHomographyScores( views, twoViewInfoVec ); 
  printf("\nComputed homography scores");
  fflush(stdout);

  //VisualizePairsWithScores( views, twoViewInfoVec );
  bool s7 = ComputeContextScore( views, twoViewInfoVec );
  printf("\nComputed context scores");
  fflush(stdout);

  theia::ViewGraph viewingGraph;
  CreateViewingGraph( views, twoViewInfoVec, &viewingGraph ); 
  EvaluateTripletsForRotationLoop(&viewingGraph, edge2indexMap, twoViewInfoVec );
  printf("\nComputed triplet scores");
  fflush(stdout);

  vector< PairwiseInfoWithPoints >& twoViewInfoVecRead = twoViewInfoVec;
  string pairwiseImgPath= baseDirectory + "/img_pairs/";
  for(int i=0; i < twoViewInfoVec.size(); i++) {
    PairwiseInfoWithPoints& pair = twoViewInfoVec[i];
    stringstream ss;
    ss << baseDirectory << "/binary_tmp/pair-" << i <<".bin";
    WritePairwiseInfoToDisk( ss.str(), pair );
    if(writePairImages == true) {
      int img1 = pair.image1;
      int img2 = pair.image2;

      vector< cv::Point2f > matchPointSet1, matchPointSet2;
      CreatePointSetsFromIndexedMatches( pair.primaryVerifiedMatches, 
          views[img1].keysWithDesc.keypoints,
          views[img2].keysWithDesc.keypoints,
          matchPointSet1, matchPointSet2);

      string imgName1 = views[img1].viewObj.Name();
      string imgName2 = views[img2].viewObj.Name();

      size_t sep1 = imgName1.find_last_of(".");
      if (sep1 != std::string::npos)
        imgName1 = imgName1.substr(0, sep1);

      size_t sep2 = imgName2.find_last_of(".");
      if (sep2 != std::string::npos)
        imgName2 = imgName2.substr(0, sep2);

      string pairImageName = pairwiseImgPath + 
        imgName1 + "_" + imgName2 + ".jpg" ;

      cout << imgName1 << " "  << imgName2 << endl;

      DrawPairsWithPoints(views[img1].fullImageName, 
          views[img2].fullImageName,
          matchPointSet1,
          matchPointSet2,
          0.0f,
          pairImageName);
    }
  }

  for(int i=0; i < views.size(); i++) {
    ViewInfoWithPoints& view = views[i];
    stringstream ss;
    ss << baseDirectory << "binary_tmp/view-" << i << ".bin";
    WriteViewInfoToDisk( ss.str(), view );
  }
  /*
  for(int i=0; i < twoViewInfoVecRead.size(); i++) {
    PairwiseInfoWithPoints& pair = twoViewInfoVecRead[i];
    PairwiseInfoWithPoints& pair1 = twoViewInfoVec[i];
    stringstream ss;
    ss << baseDirectory << "/binary_tmp/pair-" << i <<".bin";
    ReadPairwiseInfoFromDisk( ss.str(), &pair );
  }

  theia::ViewGraph viewingGraph;
  bool s4 = ComputeTwoGeometryScores( views, twoViewInfoVecRead ); 
  bool s5 = ComputeOverlapRatioScores( views, twoViewInfoVecRead ); 
  bool s6 = ComputeHomographyScores( views, twoViewInfoVecRead ); 
  bool s7 = ComputeContextScore( views, twoViewInfoVecRead );
  

  CreateViewingGraph( views, twoViewInfoVecRead, &viewingGraph ); 
  EvaluateTripletsForRotationLoop(&viewingGraph, edge2indexMap, twoViewInfoVecRead );
  //VisualizePairsWithScores( views, twoViewInfoVecRead );
  //VisualizeTripletsWithScores( views, twoViewInfoVecRead );

  printf("\nDone estimating triplet loops");
  fflush(stdout);

  ScoresStruct scoreStr;
  bool s0 = ReadNormOptFile( normOptFile , scoreStr ); 
  PopulateScoresStructure( twoViewInfoVec, views , scoreStr, baseDirectory); //Written as raw scores 
  ComputeCostsFromScores( scoreStr, views, twoViewInfoVec, baseDirectory); //Write normalized scores

  ViewGraphFilter filter;
  filter.pairsInfoPtr = &(twoViewInfoVec);
  filter.viewsPtr = &(views);
  filter.edge2indexMap = &(edge2indexMap);

  filter.constructNetworkFlowProblem(totalFlow);
  filter.solveNetworkFlowProblem();
  exit(-1);

*/
/*



  const auto& all_edges = viewingGraph.GetAllEdges();
  theia::ReconstructionBuilderOptions options;
  options.reconstruction_estimator_options.reconstruction_estimator_type = 
    theia::ReconstructionEstimatorType::INCREMENTAL;

  theia::ReconstructionBuilder reconstruction_builder(options);

  for(int i=0; i < base_views.size(); i++) {
    reconstruction_builder.AddImageWithCameraIntrinsicsPrior(
        base_views[i].Name(), views[i].CameraIntrinsicsPrior(), 
        theia::kInvalidCameraIntrinsicsGroupId);
  }

  printf("\nAdded all images");
  fflush(stdout);

  // Create an MST reconstruction.
  vetcor<int> selectedPairsIndices;
  for(int i=0; i < filter.networkFlowSol.size(); i++) {
    if(filter.nwSol2vgIdx[i].first == "pair") {
    if(filter.networkFlowSol[i] > 0.0f) {
    filter.selectedPairIndices.push_back( nwSol2vgIdx[i].second );
    }
    } 
  }

  for (int i=0; i < filter.selectedPairIndices.size(); i++) {
    int pairIndex = filter.selectedPairIndices[i];
    PairwiseInfoWithPoints pairwise = twoViewInfoVec[pairIndex];
    //DrawPairsWithPoints(views[pairwise.image1].fullImageName, views[pairwise.image2].fullImageName, pairwise.verifiedFeatures1, pairwise.verifiedFeatures2, jointScore[pairIndex], outputFile.str());

    theia::ImagePairMatch imgPair;
    imgPair.image1 = base_views[pairwise.image1].Name();
    imgPair.image2 = base_views[pairwise.image2].Name();

    imgPair.twoview_info = pairwise;
    imgPair.correspondences.resize( pairwise.verifiedFeatures1.size() );

    for(int i=0; i < imgPair.correspondences.size(); i++) {
      imgPair.correspondences[i].feature1 = theia::Feature(pairwise.verifiedFeatures1[i].x, pairwise.verifiedFeatures1[i].y);
      imgPair.correspondences[i].feature2 = theia::Feature(pairwise.verifiedFeatures2[i].x, pairwise.verifiedFeatures2[i].y);
    }

    reconstruction_builder.AddTwoViewMatch( base_views[pairwise.image1].Name(), base_views[pairwise.image2].Name(), imgPair );

  }
  printf("\nAdded all pairs");
  fflush(stdout);


  std::vector<theia::Reconstruction*> reconstructions;
  bool status = reconstruction_builder.BuildReconstruction(&reconstructions);

  if(!status) {
    cout << "\nFailed to create reconstruction";
    fflush(stdout);
    exit(-1);

  }

  string output_prefix = "result_reconstruction";
  for (int i = 0; i < reconstructions.size(); i++) {
    const std::string output_file =
      theia::StringPrintf("%s-%d", output_prefix.c_str(), i);
    cout << "Writing reconstruction " << i << " to " << output_file;
    if(!theia::WriteReconstruction(*reconstructions[i], output_file))
      cout << "Could not write reconstruction to file.";
  }

  fflush(stdout);
  */

  return 0;
}
