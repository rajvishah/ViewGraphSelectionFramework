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
#include <unistd.h>
#include <sys/time.h>

int timeval_subtract(struct timeval *result, 
        struct timeval *t2, 
        struct timeval *t1) {
    long int diff = (t2->tv_usec + 1000000 * t2->tv_sec) - (t1->tv_usec + 1000000 * t1->tv_sec);
    result->tv_sec = diff / 1000000;
    result->tv_usec = diff % 1000000;

    return (diff<0);
}
/*
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
*/

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

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);

  string baseDirectory = string(argv[1]);
  string paramPrefix = string(argv[2]);
  int nodeConstant = atoi(argv[6]);

  string solFileName = baseDirectory + "/output/" + paramPrefix + "_nodeconst_" + argv[6] + 
    "_flow_" + argv[3] + "_solution.txt";
  string normOptFile = baseDirectory + "/normalization_options/" + paramPrefix + "_options.txt";
  string rawScoresFile = baseDirectory + "/output/" + paramPrefix + "_unnormalized_scores.txt";
  string normScoresFile = baseDirectory + "/output/" + paramPrefix + "_normalized_scores.txt";
  string cumScoresFile = baseDirectory + "/output/" + paramPrefix + "_cummulative_scores.txt";
  string viewgraphFile = baseDirectory + "/output/" + paramPrefix + "_nodeconst_" + argv[6] + "_flow_" + argv[3] + "_view_graph.bin";
  string vsfmPairwiseListFile = baseDirectory + "/output/" + paramPrefix + "_nodeconst_" + argv[6] +"_flow_" + argv[3] + "_vsfm_list_pairs.txt";

  ofstream pairListFile( vsfmPairwiseListFile, ofstream::out );
  if( !pairListFile.is_open() ) {
    printf("\nCould not open pairfile");
    fflush(stdout);
  }
  
  int totalFlow = atoi(argv[3]);
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

  /*

  theia::ViewGraph viewingGraph;
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
  
  CreateViewingGraph( views, twoViewInfoVec, &viewingGraph ); 
  EvaluateTripletsForRotationLoop(&viewingGraph, edge2indexMap, twoViewInfoVec );
  printf("\nComputed triplet scores");
  fflush(stdout);
  //VisualizeTripletsWithScores( views, twoViewInfoVec );


  printf("\nDone estimating triplet loops");
  fflush(stdout);

  */
  double nodeCostMul = 1.0f;
  if(nodeConstant == 0) {
    nodeCostMul = 0.0f;
  } else if(nodeConstant == 1) {
    nodeCostMul = 1.0f; 
  } else if(nodeConstant == 2) {
    nodeCostMul = 0.5f; 
  }

  ScoresStruct scoreStr;
  bool s0 = ReadNormOptFile( normOptFile , scoreStr ); 
  PopulateScoresStructure( twoViewInfoVec, views , scoreStr, rawScoresFile, normScoresFile); 
  //Written as raw scores 
  ComputeCostsFromScores( scoreStr, views, twoViewInfoVec, cumScoresFile, nodeCostMul ); //Write normalized scores

  ViewGraphFilter filter;
  filter.pairsInfoPtr = &(twoViewInfoVec);
  filter.viewsPtr = &(views);
  filter.edge2indexMap = &(edge2indexMap);

  filter.constructNetworkFlowProblem(totalFlow);
  struct timeval t1, t2, t3;
  gettimeofday(&t1, NULL);
  filter.solveNetworkFlowProblem();
  gettimeofday(&t2, NULL);
  timeval_subtract(&t3,&t2,&t1);
  
  filter.writeSolution(solFileName);

  int numImgVars = views.size();
  int numSrcVars = views.size();
  int numSinkVars = views.size();
  int numPairVars = twoViewInfoVec.size();

  vector<bool> selectedImages( numImgVars , false);
  vector<bool> selectedEdges( numPairVars , false);

  int imgCounter = 0;
  int pairCounter = 0;

  int numSelectedImgs = 0;
  int numSelectedPairs = 0;

  for( int i=0; i < filter.networkFlowSol.size(); i++ ) {
    if(i < numSrcVars + numSinkVars) {
      continue;
    }

    if(i < numSrcVars + numSinkVars + numImgVars) {
      if(filter.networkFlowSol[i] > 0.0f) {
        selectedImages[imgCounter] = true;
        numSelectedImgs++;
      }
      imgCounter++;
      continue;
    }

    if(filter.networkFlowSol[i] > 0.0f) {
      selectedEdges[pairCounter] = true;
      numSelectedPairs++;
    }
    pairCounter++;
  }

  printf("\nFlow Statistics:");
  printf("\nNodes:%d\nEdges:%d\nFlow:%d", (int) views.size(), (int) twoViewInfoVec.size(), totalFlow);
  printf("\nSelected nodes:%d\nSelected edges:%d", numSelectedImgs, numSelectedPairs);
  printf("\nTime taken (in seconds) : %ld.%06d", t3.tv_sec, t3.tv_usec);
  fflush(stdout);

  for (int i=0; i < twoViewInfoVec.size(); i++) {
    if(selectedEdges[i]) {
      PairwiseInfoWithPoints pairwise = twoViewInfoVec[i];
      int img1 = pairwise.image1;
      int img2 = pairwise.image2;

      pairListFile << views[img1].fullImageName << " " << views[img2].fullImageName << endl; 
    }
  } 

  pairListFile.close();


  vector<string> view_names( views.size() );
  vector<theia::CameraIntrinsicsPrior> camPriors( views.size() );
  for(int i=0; i < views.size(); i++) {
    view_names[i] = views[i].viewObj.Name();
    camPriors[i] = views[i].viewObj.CameraIntrinsicsPrior();
  }

  vector<theia::ImagePairMatch> matches;
  for (int i=0; i < twoViewInfoVec.size(); i++) {
    if(selectedEdges[i]) {
      PairwiseInfoWithPoints pairwise = twoViewInfoVec[i];
      theia::ImagePairMatch imgPair;
      imgPair.image1 = views[pairwise.image1].viewObj.Name();
      imgPair.image2 = views[pairwise.image2].viewObj.Name();
        imgPair.twoview_info = pairwise.primaryMotion;
      CreateCorrespondencesFromIndexedMatches( views[pairwise.image1], views[pairwise.image2], pairwise.primaryVerifiedMatches, &imgPair.correspondences);
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
/*

  theia::ReconstructionBuilderOptions options;
  options.reconstruction_estimator_options.reconstruction_estimator_type = 
    theia::ReconstructionEstimatorType::INCREMENTAL;

  theia::ReconstructionBuilder reconstruction_builder(options);

  for(int i=0; i < views.size(); i++) {
    reconstruction_builder.AddImageWithCameraIntrinsicsPrior(
        views[i].viewObj.Name(), views[i].viewObj.CameraIntrinsicsPrior(), 
        theia::kInvalidCameraIntrinsicsGroupId);
  }

  printf("\nAdded all images");
  fflush(stdout);

  // Create a subset view-graph
  vector<int> selectedPairIndices;
  for(int i=0; i < filter.networkFlowSol.size(); i++) {
    if(filter.nwSol2vgIdx[i].first == "pair") {
      if(filter.networkFlowSol[i] > 0.0f) {
        selectedPairIndices.push_back( filter.nwSol2vgIdx[i].second );
      }
    } 
  }

   vector<theia::ImagePairMatch> matches;
  for (int i=0; i < selectedPairIndices.size(); i++) {
    int pairIndex = selectedPairIndices[i];
    PairwiseInfoWithPoints pairwise = twoViewInfoVec[pairIndex];

    theia::ImagePairMatch imgPair;
    imgPair.image1 = views[pairwise.image1].viewObj.Name();
    imgPair.image2 = views[pairwise.image2].viewObj.Name();

    imgPair.twoview_info = pairwise.primaryMotion;
    CreateCorrespondencesFromIndexedMatches( views[pairwise.image1], 
        views[pairwise.image2], pairwise.primaryVerifiedMatches, &imgPair.correspondences);

    reconstruction_builder.AddTwoViewMatch( imgPair.image1, imgPair.image2, imgPair );
    
    matches.push_back(imgPair);
  }





  std::vector<theia::Reconstruction*> reconstructions;
  bool status = reconstruction_builder.BuildReconstruction(&reconstructions);

  if(!status) {
    cout << "\nFailed to create reconstruction";
    fflush(stdout);
  }

  string output_prefix = "result_reconstruction";
  for (int i = 0; i < reconstructions.size(); i++) {
    const std::string output_file =
      theia::StringPrintf("%s/%s-%d", baseDirectory.c_str(),output_prefix.c_str(), i);
    cout << "Writing reconstruction " << i << " to " << output_file;
    if(!theia::WriteReconstruction(*reconstructions[i], output_file))
      cout << "Could not write reconstruction to file.";
  }

  fflush(stdout);
  return 0;
  */
}
