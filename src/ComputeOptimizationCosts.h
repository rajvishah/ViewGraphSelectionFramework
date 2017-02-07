#ifndef COMPUTEOPTIMIZATIONCOSTS_H
#define COMPUTEOPTIMIZATIONCOSTS_H


#include "PairwiseInfoWithPoints.h"
#include "ViewInfoWithPoints.h"
#include "defs.h"

struct NormParams {
  NormParams(): initialized(false) {}

  bool initialized;
  bool minMax01;

  double k1;
  double k2;
  double k3;
  double k4;
  double k5;
  double k6;

  double high;
  double low;
};

enum ScoreType{
  NCONN = 0,
  FCONN,
  DSJT,
  OVSUM,
  OVMUL,
  HG,
  CTXCS,
  CTXHM,
  CTXCHI2,
  TRIPLET,
  MMOTIONR,
  MMOTIONT,//ALWAYS ADD AFTER THIS, OTHERWISE MAJOR CODE BREAK
  LOOP,
  INLIERS,
  TRIANGLE,
  NUM_SCORES //Always keep this last, as we use this enum for indexing
};

class ScoresStruct {

  public:
  map<int, int> scoreIdx2Node;
  map<int, int> scoreIdx2Edge;


  NormParams params[NUM_SCORES];
  vector< vector< double > > inScore;
  vector< vector< double > > outScore;

  /*
  vector< double > nodeConnFitness;
  vector< double > featConnFitness;
  vector< double > disjointFitness;

  vector< double > overlapSum;
  vector< double > overlapMult;
  vector< double > homographyFitness;
  vector< double > contextScoreCosine;
  vector< double > contextScoreHamming;
  vector< double > contextScoreChiSqr;

  vector< double > tripletConnFitness;
  vector< double > multiMotionFitness;
  vector< double > loopFitness; 
  */

  ScoresStruct();

  void normalizeAllScores();
  void normalize( vector<double>& input, vector<double>& output, NormParams& param);  

//  double k1, double k2, double k3, double k4, double k5, double k6, 
//      double low, double high);

};

bool ReadNormOptFile(string fileName, ScoresStruct& scoreStr);
void PopulateScoresStructure( vector<PairwiseInfoWithPoints>& twoviewInfo, 
    vector<ViewInfoWithPoints>& views, ScoresStruct& scoreStr, string file1, string file2);
void ComputeCostsFromScores( ScoresStruct& scoreStr, vector<ViewInfoWithPoints>& views, vector<PairwiseInfoWithPoints>& twoviewInfo, string basePath, double multiplier);
#endif /* COMPUTEOPTIMIZATIONCOSTS_H */
