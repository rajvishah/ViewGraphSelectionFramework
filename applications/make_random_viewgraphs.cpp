#include <stdio.h>
#include "defs.h"
#include <iostream>
#include <random>
#include <vector>
#include <math.h>
#include <ctime>
#include <set>
#include "FilterViewGraph.h"
#include <theia/theia.h>
#include <sys/time.h>

using namespace std;
int timeval_subtract(struct timeval *result, 
        struct timeval *t2, 
        struct timeval *t1) {
    long int diff = (t2->tv_usec + 1000000 * t2->tv_sec) - (t1->tv_usec + 1000000 * t1->tv_sec);
    result->tv_sec = diff / 1000000;
    result->tv_usec = diff % 1000000;

    return (diff<0);
}

int main(int argc, char* argv[]) {

  unsigned int numVertices = atoi(argv[1]);
  unsigned int meanDeg = atoi(argv[2]);

  unsigned int  stdDeg = ceil(meanDeg/4);
  vector< int > degrees( numVertices, 0 );
  vector< set <int> > adjacencyList( numVertices );

  std::default_random_engine generator;
  std::normal_distribution<double> degree_dist( meanDeg, stdDeg);

  for (int i=0; i< numVertices; i++) {
    int number = (int)(degree_dist(generator));
    if( number < 0) number += numVertices - 1;
    if( number >= numVertices) number -= (numVertices - 1);
    degrees[i] = number;
  }

  for(int i=0; i < numVertices; i++) {
    std::cout << "Generating adjacency list for vertex : " << i ;
    std::normal_distribution<double> adj_dist(i, meanDeg);

    set<int>& adjSet = adjacencyList[i];
    while(adjSet.size() != degrees[i]) {
      int number = (int)(adj_dist(generator));
      
      if(number == i) {
        int randomNumber = rand() % 2;
        if(randomNumber == 0) {
          number = i-1;
        } else {
          number = i+1;
        }
      }
      
      if(number < 0) number += numVertices - 1;
      if(number >= numVertices) number -= (numVertices - 1);
      adjSet.insert(number);
    }
    printf(" connected to %d verts\n", adjSet.size());
  }

  cout << "Making graph " << endl;
  vector< pair<int, int> > graph;
  for(int i=0; i < numVertices; i++) {
    set<int>::iterator adjItr = adjacencyList[i].begin();
    while(adjItr != adjacencyList[i].end()) {
      
      if(i < *adjItr) { 
        graph.push_back( make_pair(i, *adjItr) );
      } else {
        graph.push_back( make_pair(*adjItr, i) );
      }
      adjItr++;
    }
    printf("\nVert %d", i);
    fflush(stdout);
  }

  std::cout << "Printing data to file " << std::endl;
  FILE* fp = fopen("synthetic_graph.txt", "w");
  FILE* fp1 = fopen("synthetic_graph_sol_statistics.txt", "w");
  fprintf(fp1,"#Nodes Edges AvgDeg. Flow Time SelNode SelEdge");
  for(int i=0; i < graph.size(); i++) {
    fprintf(fp, "\n%d %d", graph[i].first, graph[i].second);
  }
  fclose(fp);

  std::uniform_real_distribution<double> uni_dist(0.0,1.0);
  vector< double > vert_cost( numVertices );
  for(int i=0; i < numVertices; i++) {
    vert_cost[i] = uni_dist(generator);
  }

  vector< double > pair_cost( graph.size() );
  for(int i=0; i < graph.size(); i++) {
    pair_cost[i] = uni_dist(generator);
  }

  for(int tFlow=1; tFlow <= graph.size(); tFlow*=2) {
    int numSelectedImgs = 0;
    int numSelectedPairs = 0;

    ViewGraphFilter filter;
    filter.constructSyntheticNetworkFlowProblem(numVertices, 
        graph, vert_cost, pair_cost, tFlow);

    struct timeval t1, t2, t3;
    gettimeofday(&t1, NULL);
    filter.solveNetworkFlowProblem();
    gettimeofday(&t2, NULL);
    timeval_subtract(&t3,&t2,&t1);
    
    for( int i=0; i < filter.networkFlowSol.size(); i++ ) {
      if(i < 2*numVertices) {
        continue;
      }

      if(i < 3*numVertices) {
        if(filter.networkFlowSol[i] > 0.0f) {
          numSelectedImgs++;
        }
        continue;
      }

      if(filter.networkFlowSol[i] > 0.0f) {
        numSelectedPairs++;
      }
    }

    //fprintf(fp1,"#Nodes Edges AvgDeg. Flow Time SelNode SelEdge");
    fprintf(fp1,"\n%d %d %d %d %ld.%06d %d %d",(int)numVertices, (int) 
        graph.size(), meanDeg, tFlow, t3.tv_sec, t3.tv_usec, 
        numSelectedImgs, numSelectedPairs);

    fflush(stdout);
  }

  fclose(fp1);
  return 0;
}
