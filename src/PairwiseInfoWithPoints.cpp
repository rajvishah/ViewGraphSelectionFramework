#include "PairwiseInfoWithPoints.h"

PairwiseInfoWithPoints::PairwiseInfoWithPoints() {
  matchGraphEdgeIntialized = false;
  viewGraphEdgeIntialized = false;
  mergeMotions = false;
  hasTwoGeometries = false;
  homographyEstimated = false;
  homographyFound = false;

  scores.ratio1 = 0;
  scores.ratio2 = 0;
  scores.angularDiff = 0;
  scores.positionDiff = 0;
  scores.tripletError = 0;
  scores.tripletScore = 0;
  scores.homographyScore = 0;
  scores.contextScoreCosine = 0;
  scores.contextScoreChiSqr = 0;
  scores.contextScoreChiSqr = 0;

}

bool WritePairwiseInfoToDisk( string fileName , PairwiseInfoWithPoints& pair) {

  std::ofstream pairwise_writer(fileName, std::ios::out | std::ios::binary);
  if (!pairwise_writer.is_open()) {
    LOG(ERROR) << "Could not open the matches file: " << fileName
      << " for writing.";
    return false;
  }

  // Make sure that Cereal is able to finish executing before returning.
  {
    cereal::PortableBinaryOutputArchive output_archive(pairwise_writer);
    output_archive(pair);
  }

  return true;
}

bool ReadPairwiseInfoFromDisk( string fileName , PairwiseInfoWithPoints* pair) {
  if(pair == NULL) {
    printf("\nError with NULL pair pointer");
    return false;
  }

  std::ifstream pairwise_reader(fileName, std::ios::in | std::ios::binary);
  if (!pairwise_reader.is_open()) {
    LOG(ERROR) << "Could not open the matches file: " << fileName
      << " for reading.";
    return false;
  }

  // Make sure that Cereal is able to finish executing before returning.
  {
    cereal::PortableBinaryInputArchive input_archive(pairwise_reader);
    input_archive(*pair);
  }

  return true;
}
