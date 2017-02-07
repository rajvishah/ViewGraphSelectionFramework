#include "ComputeOptimizationCosts.h"

ScoresStruct::ScoresStruct() {

  inScore.resize(NUM_SCORES);
  outScore.resize(NUM_SCORES);

}


void ScoresStruct::normalizeAllScores() {
  for(int i=0; i < NUM_SCORES; i++) {
    if(params[i].initialized && inScore[i].size()) {
      normalize(inScore[i], outScore[i], params[i]);  
    }
  } 
}


void ScoresStruct::normalize( vector<double>& input, 
    vector<double>& output, NormParams& param) {
    
//    double k1, double k2, 
//    double k3, double k4, double k5, double k6, 
//    double low, double high) {

  double minInp = *std::min_element(input.begin(), input.end());
  double maxInp = *std::max_element(input.begin(), input.end());

  vector<double> tmp(input.size());
  if(param.minMax01) {
    for(int i=0; i < tmp.size(); i++) {
      tmp[i] = (input[i] - minInp)/(maxInp - minInp);
    }
  } else{
    tmp = input;
  }

  output.resize(input.size());
  for(int i=0; i < tmp.size(); i++) {
    if(tmp[i] >= param.k1 && tmp[i] < param.k2) {
       output[i] = param.low;
    }

    if(tmp[i] >= param.k2 && tmp[i] < param.k3) {
      double m = (param.high - param.low)/ (param.k3 - param.k2);
      double c = (param.low*param.k3 - param.high*param.k2)/(param.k3 - param.k2);
      output[i] = m * tmp[i] + c; 
    }

    if(tmp[i] >=param.k3 && tmp[i] < param.k4) {
      output[i] = param.high;
    }

    if(tmp[i] >=param.k4 && tmp[i] < param.k5) {
      double m = (param.low - param.high)/(param.k5 - param.k4); 
      double c = (param.k5*param.high - param.k4*param.low)/(param.k5 - param.k4);
      output[i] = m* tmp[i] + c; 
    }

    if(tmp[i] >= param.k5 && tmp[i] <= param.k6) {
      output[i] = param.low; 
    }
  }
}

void PopulateScoresStructure( vector<PairwiseInfoWithPoints>& twoviewInfo, 
    vector<ViewInfoWithPoints>& views, ScoresStruct& scoreStr, 
    string rawScoresFile, string normScoresFile) {

  FILE* f1 = fopen(rawScoresFile.c_str(), "w");
  if(f1 == NULL) {
    printf("\nProblem opening image score file");
  }
  
  FILE* f2 = fopen(normScoresFile.c_str(), "w");
  if(f2 == NULL) {
    printf("\nProblem opening pair score file");
  }

  vector<bool> isViewInEdge( views.size() , false);

  set<int> allIndices;
  int edgeIdx = 0;
  for(int i=0; i < twoviewInfo.size(); i++) {
    PairwiseInfoWithPoints& pairwise = twoviewInfo[i];
    if(pairwise.viewGraphEdgeIntialized) {
      scoreStr.scoreIdx2Edge.insert( make_pair(edgeIdx, i) );
      edgeIdx++;
      allIndices.insert(pairwise.image1);
      allIndices.insert(pairwise.image2);

      isViewInEdge[pairwise.image1] = true;
      isViewInEdge[pairwise.image2] = true;
    }
  }

  int numViewAdded = 0;
  for(int i=0; i < isViewInEdge.size(); i++) {
    if(isViewInEdge[i]) {
      numViewAdded++;
    }
  }

  if(allIndices.size() != views.size()) {
    printf("\nSome images not part of edges ... ");
    printf("\nNum views added = %d", numViewAdded);
    printf("\nNum images in edges %d, num images %d", allIndices.size(), views.size());
    fflush(stdout);
    exit(-1);
  }

  for(int i=3; i < NUM_SCORES; i++) {
    scoreStr.inScore[i].resize(scoreStr.scoreIdx2Edge.size(), 0.0f); 
    scoreStr.outScore[i].resize(scoreStr.scoreIdx2Edge.size(), 0.0f); 
  }

//  fprintf(f1,"NumPairs %d\n", twoviewInfo.size());

  for(int i=0; i < scoreStr.scoreIdx2Edge.size(); i++) {

    int pIdx = scoreStr.scoreIdx2Edge[i];
    PairwiseInfoWithPoints& pairwise = twoviewInfo[pIdx];

    scoreStr.inScore[OVSUM][i] = ( 0.5*(pairwise.scores.ratio1 + pairwise.scores.ratio2) ); 
    scoreStr.inScore[OVMUL][i] = (pairwise.scores.ratio1*pairwise.scores.ratio2); 
    scoreStr.inScore[HG][i] = (pairwise.scores.homographyScore);
    scoreStr.inScore[CTXCS][i] = (pairwise.scores.contextScoreCosine);
    scoreStr.inScore[CTXHM][i] = (pairwise.scores.contextScoreHamming);
    scoreStr.inScore[CTXCHI2][i] = (pairwise.scores.contextScoreChiSqr);

    double rotDiff = (pairwise.scores.angularDiff);
    double transDiff = pairwise.scores.positionDiff;
    double max_pose_diff = rotDiff > transDiff ? rotDiff : transDiff;
    scoreStr.inScore[MMOTIONR][i] = (pairwise.scores.angularDiff);
    scoreStr.inScore[MMOTIONT][i] = pairwise.scores.positionDiff;
    scoreStr.inScore[LOOP][i] = pairwise.scores.tripletScore;
    scoreStr.inScore[INLIERS][i] = (double) pairwise.primaryVerifiedMatches.size();
    scoreStr.inScore[TRIANGLE][i] = pairwise.medianPrimaryTriAngle;
    //scoreStr.inScore[LOOP][i] = ( );
   

    fprintf(f1,"%d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n", 
        pIdx, 
        pairwise.scores.ratio1, pairwise.scores.ratio2, pairwise.scores.homographyScore, 
        pairwise.scores.contextScoreCosine, pairwise.scores.contextScoreHamming, 
        pairwise.scores.contextScoreChiSqr, pairwise.scores.angularDiff, 
        pairwise.scores.positionDiff, 
        pairwise.scores.tripletScore,
        (double) pairwise.primaryVerifiedMatches.size(),
        pairwise.scores.medianTriangAngle);
  } 
  
//  fprintf(f1,"NumImages %d\n", views.size());
  for(int i=0; i < views.size(); i++) {
    scoreStr.scoreIdx2Node.insert(make_pair(i,i));
    scoreStr.inScore[NCONN].push_back(views[i].connectivityFitness);
    scoreStr.inScore[FCONN].push_back(views[i].featureFitness);
    scoreStr.inScore[DSJT].push_back(views[i].disjointFitness);

//    fprintf(f1,"%s %lf %lf %lf\n", views[i].viewObj.Name().c_str(), views[i].connectivityFitness, views[i].featureFitness, views[i].disjointFitness);
  }

  scoreStr.outScore[NCONN].resize( scoreStr.inScore[NCONN].size(), 0.0f);
  scoreStr.outScore[FCONN].resize( scoreStr.inScore[FCONN].size(), 0.0f);
  scoreStr.outScore[DSJT].resize( scoreStr.inScore[DSJT].size(), 0.0f);

  fclose(f1);
  scoreStr.normalizeAllScores();

  for(int i=0; i < 3; i++) { 
    if(scoreStr.params[i].initialized && scoreStr.inScore[i].size()) {
//      fprintf(f2,"\nscore %d\n", i);
      for(int j=0; j < scoreStr.outScore[i].size(); j++) {
//        fprintf(f2,"%lf ", scoreStr.outScore[i][j]);
      }
    }
  }

  for(int i=3; i < NUM_SCORES; i++) {
    if(scoreStr.params[i].initialized && scoreStr.inScore[i].size()) {
//      fprintf(f2,"\nscore %d\n", i);
      fprintf(f2,"\n");
      for(int j=0; j < scoreStr.outScore[i].size(); j++) {
        fprintf(f2,"%lf ", scoreStr.outScore[i][j]);
      }
    }
  }
  fclose(f2);
}

bool ReadNormOptFile(string fileName, ScoresStruct& scoreStr) {

  ifstream inFile( fileName, ifstream::in );
  if( !inFile.is_open() ) {
    printf("\nNormalization File is not open");
    fflush(stdout);
    exit(-1);
  }

  string line;
  vector< vector<string> > strings;
  while ( std::getline(inFile, line)) {
    std::istringstream str ( line );
    vector<string> currLine;
    currLine.insert(currLine.end(), 
        std::istream_iterator<std::string>(str),  
        std::istream_iterator<std::string>());
    strings.push_back(currLine);
  }

  int numParamInitialized = 0;
  for(int i=0; i < strings.size(); i++) {
    vector<string> currLine = strings[i];
    string token = currLine[0];
    if(token[0] == '#') {
      continue;
    }

    numParamInitialized++;

    NormParams* params;
    if(token == "NODECONN") {
      params = &(scoreStr.params[NCONN]);
    } else if(token == "FEATCONN" ) {
      params = &(scoreStr.params[FCONN]);
    } else if(token == "DISJOINT") { 
      params = &(scoreStr.params[DSJT]);
    } else if(token == "OVERLAPSUM") { 
      params = &(scoreStr.params[OVSUM]);
    } else if(token == "OVERLAPMULT") {
      params = &(scoreStr.params[OVMUL]);
    } else if(token == "HOMOGRAPHY") { 
      params = &(scoreStr.params[HG]);
    } else if(token == "CTXCHISQR") { 
      params = &(scoreStr.params[CTXCHI2]); 
    } else if(token == "CTXHAMMING") {
      params = &(scoreStr.params[CTXHM]);
    } else if(token == "CTXCOSINE") { 
      params = &(scoreStr.params[CTXCS]);
    } else if(token == "TRIPLET") { 
      params = &(scoreStr.params[TRIPLET]);
    } else if( token == "MULTIMOTROT") {
      params = &(scoreStr.params[MMOTIONR]);
    } else if( token == "MULTIMOTPOS") {
      params = &(scoreStr.params[MMOTIONT]);
    } else if( token == "LOOPCONST" ) {
      params = &(scoreStr.params[LOOP]);
    } else if( token == "TRIANGLE" ) {
      params = &(scoreStr.params[TRIANGLE]);
    }
    params->initialized = true;
    printf("\nUsing parameter %s", token.c_str());

    istringstream(currLine[1]) >> params->minMax01;
    istringstream(currLine[2]) >> params->k1;
    istringstream(currLine[3]) >> params->k2;
    istringstream(currLine[4]) >> params->k3;
    istringstream(currLine[5]) >> params->k4;
    istringstream(currLine[6]) >> params->k5;
    istringstream(currLine[7]) >> params->k6;
    istringstream(currLine[8]) >> params->high;
    istringstream(currLine[9]) >> params->low;
  }

  printf("\nNumber of parameters initialized %d", numParamInitialized);
  fflush(stdout);

  if(numParamInitialized == 0) {
    printf("\nNo parameters specified, exiting");
    fflush(stdout);
    exit(-1);
  }
}

void ComputeCostsFromScores( ScoresStruct& scoreStr, vector<ViewInfoWithPoints>& views, vector<PairwiseInfoWithPoints>& twoviewInfo, 
    string cummulativeScoreFile, double nodeConstant) {
  vector<double> alpha(NUM_SCORES,1.0f);
  for(int i=0; i < NUM_SCORES; i++) { 
    if(!scoreStr.params[i].initialized) {
      alpha[i] = 0.0f;
    }
  }

  FILE* fp1 = fopen(cummulativeScoreFile.c_str(), "w");
  if(fp1 == NULL) {
    printf("\nProblem opening cumm score file");
  }

  // specific alpha values 
  // alpha[NCONN] = 0.33;

  for(int i=0; i < scoreStr.scoreIdx2Node.size(); i++) {
    double imageEdgeCost = 0.0f;
    double alphaSum = 0.0f;
    for(int j=0; j < 3; j++) {
      alphaSum += alpha[j];
      imageEdgeCost += alpha[j]*scoreStr.outScore[j][i];
    }

    if(alphaSum == 0.0f) {
      imageEdgeCost = 0.0f;
    } else {
      imageEdgeCost /= alphaSum;
    }

    imageEdgeCost = imageEdgeCost* nodeConstant;

    int nodeIdx = scoreStr.scoreIdx2Node[i];
    views[nodeIdx].cost = imageEdgeCost;

    fprintf(fp1, "Image %s %f\n", views[nodeIdx].viewObj.Name().c_str(), imageEdgeCost);
  }

  for(int i=0; i < scoreStr.scoreIdx2Edge.size(); i++) {
    double pairEdgeCost = 0.0f;
    double alphaSum = 0.0f;
    int edgeIdx = scoreStr.scoreIdx2Edge[i];
    for(int j=3; j < NUM_SCORES; j++) {
      double alphaVal = alpha[j];
      if( twoviewInfo[edgeIdx].secondaryVerifiedMatches.empty() && 
          (j == MMOTIONR || j == MMOTIONT) ) {
        alphaVal = 0.0f;
      }
      pairEdgeCost += alphaVal*scoreStr.outScore[j][i];
      alphaSum += alphaVal;
    }

    if(alphaSum == 0.0f) {
      pairEdgeCost = 0.0f;
    } else {
      int img1 = twoviewInfo[edgeIdx].image1;
      int img2 = twoviewInfo[edgeIdx].image2;
      pairEdgeCost = pairEdgeCost /= alphaSum;
      fprintf(fp1, "%s %s %f\n", views[img1].viewObj.Name().c_str(), 
          views[img2].viewObj.Name().c_str(), pairEdgeCost);
    }

    twoviewInfo[edgeIdx].cost = pairEdgeCost;
  }

}

