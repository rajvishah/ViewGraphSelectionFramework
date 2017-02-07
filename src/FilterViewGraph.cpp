#include "FilterViewGraph.h"
#include "mosek.h"


static void MSKAPI printstr(void *handle, 
    MSKCONST char str[]) 
{ 
  printf("%s",str); 
} /* printstr */ 

void ViewGraphFilter::writeSolution(string filename) {

  int numImages = viewsPtr->size();
  int numSrcEdges = numImages;
  int numSinkEdges = numImages;
  int numImageEdges = numImages;
  
  FILE* fp = fopen(filename.c_str(), "w");
  if(fp == NULL) {
    printf("\nCan't open solution file");
    fflush(stdout);
    exit(-1);
  }

  for(int i=0; i < numSrcEdges; i++) {
    fprintf(fp,"source %s %f %f\n", viewsPtr->at(i).fullImageName.c_str(), networkFlowSol[i], cost[i]);  
  }

  for(int i=numSrcEdges; i < numSrcEdges+numSinkEdges; i++) {
    int id = i - numSrcEdges;
    fprintf(fp,"sink %s %f %f\n", viewsPtr->at(id).fullImageName.c_str(), networkFlowSol[i], cost[i]);  
  }

  for(int i=numSrcEdges+numSinkEdges; i < numSrcEdges+numSinkEdges+numImageEdges; i++) {
    int id = i - (numSrcEdges + numSinkEdges);
    fprintf(fp,"image %s %f %f\n", viewsPtr->at(id).fullImageName.c_str(), networkFlowSol[i], cost[i]);  
  }

  for(int i=0; i < pairsInfoPtr->size(); i++) {
    PairwiseInfoWithPoints pair = pairsInfoPtr->at(i);
    int img1 = pair.image1;
    int img2 = pair.image2;
    int id = numSrcEdges + numSinkEdges + numImageEdges + i;
      fprintf(fp,"%s %s %f %f\n", viewsPtr->at(img1).fullImageName.c_str(), 
          viewsPtr->at(img2).fullImageName.c_str(),
          networkFlowSol[id], cost[id]);
  }

  fclose(fp);
}


bool ViewGraphFilter :: constructNetworkFlowProblem(int tFlow) {
  totalFlow = tFlow;
  sourceWeight = 0.0f;
  sinkWeight = 0.0f;

  /*
  FILE* fp1 = fopen("AMatrix.txt","w");
  FILE* fp2 = fopen("BMatrix.txt","w");
  FILE* fp3 = fopen("bounds.txt","w");
  FILE* fp5 = fopen("costij.txt","w");
  FILE* fp4 = fopen("edgeindices.txt","w");
  */

  int numImages = viewsPtr->size();
  int numSrcEdges = numImages;
  int numSinkEdges = numImages;
  int numImageEdges = numImages;
  

  int numPairEdges = pairsInfoPtr->size();
  printf("\nNum Images %d, Num Pairs %d", numImages, numPairEdges);

  numVariables = numSrcEdges + numSinkEdges + numImageEdges + numPairEdges;
  numConstraints = 2 + 2*numImages;

  cost.resize( numVariables, 0.0f);
  networkFlowSol.resize(numVariables, 0);

  // Prepare left vector, right vector for every node (not counting connection edges)
  vector< vector<int> > incomingEdges( numImages*2 ); //Since each image is rep by two aux nodes
  vector< vector<int> > outgoingEdges( numImages*2 );
  
  int srcEdgeCounter = 0;
  int sinkEdgeCounter = 0;
  int imgEdgeCounter = 0;

  //(1 - numImage) conn from src to leftAuxNodes, 
  //(1 - numImage) conn from src to rightAuxNodes,
  //(1 - numImage) conn from leftAux to rightAux nodes

  for(int i=0; i < (2*numImages); i++) {
    // Define incoming and outgoing edges for all leftAux node
    if(i%2 == 0) {
      //var for src to left edge
      incomingEdges[i].push_back(srcEdgeCounter); 

      //var for leftAux to rightAux edge
      outgoingEdges[i].push_back((numSrcEdges + numSinkEdges) + srcEdgeCounter); 

      //increase left node counter for next image
      srcEdgeCounter++;

    }

    // Define incoming and outgoing edges for all rightAux node
    if(i%2 == 1) {
      //var for rightAux to sink  edge
      outgoingEdges[i].push_back((numSrcEdges) + sinkEdgeCounter);

      //var for rightAux to 
      incomingEdges[i].push_back((numSrcEdges + numSinkEdges) + sinkEdgeCounter);

      //increase right node counter for next image
      sinkEdgeCounter++;

    }
  }

  map<int, int> edgeCount2pairIdx;
  int edgeCount = 0;
  //(1 - numPairs) conn from rightAux to leftAux for (i j) pairs
  for(int i=0; i < pairsInfoPtr->size(); i++) {
    PairwiseInfoWithPoints pair = pairsInfoPtr->at(i);
    if(pair.viewGraphEdgeIntialized) {
      cost[3*numImages + edgeCount] = -pair.cost;
      int img1 = pair.image1; //HAS TO BE LOWER IDX
      int img2 = pair.image2; //HAS TO BE HIGHER IDX

      //fprintf(fp4, "%d %d\n", img1, img2);

      if(img1 > img2) { 
        printf("\nFirst Index Higher than Second index");
        fflush(stdout);
        exit(-1);
      }

      int leftNode = 2*img1 + 1;
      int rightNode = 2*img2;

      incomingEdges[rightNode].push_back( (numSrcEdges + numSinkEdges + numImageEdges) 
          + edgeCount);
      outgoingEdges[leftNode].push_back( (numSrcEdges + numSinkEdges + numImageEdges) 
          + edgeCount); 

      edgeCount2pairIdx.insert(make_pair(edgeCount, i));

      edgeCount++;

    }
  }

  vecB.resize(numConstraints, 0.0f );
  lb.resize( numVariables , 0.0f );
  ub.resize( numVariables, 1.0f );

  vector< vector < double > > matrixA( numConstraints );
  for(int i=0; i < numConstraints; i++) {
    matrixA[i].resize( numVariables );
  }


  for(int j=0; j < numSrcEdges; j++) {
    cost[j] = sourceWeight;
    pair<int, int> rc(0,j);
    matA_sparse.push_back( make_pair( rc, 1) );
    matrixA[0][j] = 1;
    ub[j] = totalFlow;
  }
  vecB[0] = totalFlow;

  for(int j=numSrcEdges; j < numSrcEdges + numSinkEdges; j++) {
    cost[j] = sinkWeight;
    matrixA[1][j] = 1;
    pair<int, int> rc(1,j);
    matA_sparse.push_back( make_pair( rc, 1) );
    ub[j] = totalFlow;
  }
  vecB[1] = totalFlow;

  for(int i=0; i < numImages; i++) {
    int leftNode = 2*i;
    int rightNode = 2*i + 1;
    int incomingFlow = incomingEdges[leftNode].size();
    int outgoingFlow = outgoingEdges[rightNode].size();

    ub[2*numImages + i] = incomingFlow + outgoingFlow - 2; /* subtract src and sink edge contrib */  
    cost[2*numImages + i] = -viewsPtr->at(i).cost ;
  }

  for(int i=0; i < 2*numImages; i++) {
    int n = i+2;
    for(int j=0; j < incomingEdges[i].size(); j++) {
      pair<int, int> rc(n,incomingEdges[i][j]);
      matA_sparse.push_back( make_pair( rc, 1) );
      matrixA[n][incomingEdges[i][j]] = 1; 
      //matA_sparse.insert( make_pair(rc, 1));
    }

    for(int j=0; j < outgoingEdges[i].size(); j++) {
      pair<int, int> rc(n,outgoingEdges[i][j]);
      matA_sparse.push_back( make_pair( rc, -1) );
      matrixA[n][outgoingEdges[i][j]] = -1;
      //matA_sparse.insert( make_pair(rc, 1));
    }

    vecB[n] = 0;
    //ub[n] = incomingEdges[i].size() + outgoingEdges[i].size(); 
  }

  /*
  for(int i=0; i < numConstraints; i++) {
    for(int j=0; j < numVariables; j++) {
      fprintf(fp1,"%lf ", matrixA[i][j]);  
    }
    fprintf(fp1,"\n");
  }

  for(int i=0; i < numVariables; i++) {
    fprintf(fp3,"%lf %lf\n", lb[i], ub[i]);  
    fprintf(fp5, "%lf\n", cost[i]);
  }

  for(int i=0; i < numConstraints; i++) {
    fprintf(fp2,"%lf ", vecB[i]);   
  }
  */

  nwSol2vgIdx.resize( numVariables );
  for(int i=0; i < numImages; i++) {
    nwSol2vgIdx[i] = make_pair("src",-1);
  }

  for(int i=0; i < numImages; i++) {
    nwSol2vgIdx[numImages + i] = make_pair("sink", -2);
  }

  for(int i=0; i < numImages; i++) {
    nwSol2vgIdx[2*numImages + i] = make_pair("image", i);
  }

  for(int i=0; i < edgeCount2pairIdx.size(); i++) {
    int pairIndex = edgeCount2pairIdx[i];
    nwSol2vgIdx[3*numImages + i] = make_pair("pair", pairIndex);
  }
  
  /*
  fclose(fp1);
  fclose(fp2);
  fclose(fp3);
  fclose(fp4);
  */
}

bool ViewGraphFilter :: solveNetworkFlowProblem() {
  
  const MSKint32t numvar = numVariables,
        numcon = numConstraints;

  MSKenv_t env  = NULL;
  MSKrescodee  r;


  /* Create the mosek environment. */
  r = MSK_makeenv(&env,NULL);

  if ( r != MSK_RES_OK ) {
    printf("\nCANNOT OPEN MOSEK ENV");
    fflush(stdout);
    return false;
  }

  /* Create the optimization task. */
  MSKtask_t    task = NULL;
  r = MSK_maketask(env,numcon,numvar,&task);
  r = MSK_linkfunctotaskstream(task,MSK_STREAM_LOG,NULL,printstr);

  /* Append 'numcon' empty constraints.
     The constraints will initially have no bounds. */
  if ( r == MSK_RES_OK )
    r = MSK_appendcons(task,numcon);

  /* Append 'numvar' variables.
     The variables will initially be fixed at zero (x=0). */
  if ( r == MSK_RES_OK )
    r = MSK_appendvars(task,numvar);

  vector<MSKboundkeye> boundType( numVariables, MSK_BK_RA ); 
  for(int j=0; j<numvar && r == MSK_RES_OK; ++j) {
    /* Set the linear term c_j in the objective.*/  
    if(r == MSK_RES_OK) {
      r = MSK_putcj(task,j,cost[j]);
    } else {
      printf("\nProblem setting cost");
    }

    /* Set the bounds on variable j.
       blx[j] <= x_j <= bux[j] */
    if(r == MSK_RES_OK) {
      r = MSK_putvarbound(task,
          j,           /* Index of variable.*/
          boundType[j],      /* Bound key.*/
          lb[j],      /* Numerical value of lower bound.*/
          ub[j]);     /* Numerical value of upper bound.*/
    } else {
      printf("\nProblem setting variable bounds"); 
    }

    /* Input column j of A */   
    //if(r == MSK_RES_OK)
    //  r = MSK_putacol(task,
    //      j,                 /* Variable (column) index.*/
    //      aptre[j]-aptrb[j], /* Number of non-zeros in column j.*/
    //      asub+aptrb[j],     /* Pointer to row indexes of column j.*/
    //      aval+aptrb[j]);    /* Pointer to Values of column j.*/
  }
  
  /* Set the bounds on constraints.
     for i=1, ...,numcon : blc[i] <= constraint i <= buc[i] */
  for(int i=0; i<numcon && r==MSK_RES_OK; ++i) {
    r = MSK_putconbound(task,
        i,           /* Index of constraint.*/
        MSK_BK_FX,      /* Bound key.*/
        vecB[i],      /* Numerical value of lower bound.*/
        vecB[i]);     /* Numerical value of upper bound.*/
  }

  for(int s=0; s < matA_sparse.size(); s++) {
    int row = matA_sparse[s].first.first;
    int col = matA_sparse[s].first.second;

    double val = matA_sparse[s].second;
    if(r == MSK_RES_OK) {
      r = MSK_putaij(task, row, col, val);
    } else {
      printf("\nProblem putting A matrix values");
    }
  }

  /* Use this for debugging 
  MSKrescodee MSK_printdata ( 
      MSKtask_t       task, 
      MSKstreamtypee  whichstream, 
      MSKint32t       firsti, 
      MSKint32t       lasti, 
      MSKint32t       firstj, 
      MSKint32t       lastj, 
      MSKint32t       firstk, 
      MSKint32t       lastk, 
      MSKint32t       c, 
      MSKint32t       qo, 
      MSKint32t       a, 
      MSKint32t       qc, 
      K_STREAM_LOG
      MSKint32t       bx, 
      MSKint32t       vartype, 
      MSKint32t       cones); 

  r = MSK_printdata(task, MSK_STREAM_LOG, 
      0, numConstraints, 
      0, numVariables,
      0, 1,
      0, 
      0,
      0, 
      0,
      0,
      0,
      0,
      0);

  */


  /* Minimize objective function. */
  if(r == MSK_RES_OK)
    r = MSK_putobjsense(task, MSK_OBJECTIVE_SENSE_MINIMIZE);

  if(r == MSK_RES_OK) 
    r = MSK_putintparam(task, MSK_IPAR_OPTIMIZER,  MSK_OPTIMIZER_NETWORK_PRIMAL_SIMPLEX);

  /*
  FILE* fp = fopen("solution.txt","w");
  */



  if ( r==MSK_RES_OK )
  {
    printf("\nNow solving optimization");
    fflush(stdout);
    MSKrescodee trmcode;

    /* Run optimizer */
    r = MSK_optimizetrm(task,&trmcode);

    /* Print a summary containing information
       about the solution for debugging purposes. */
    MSK_solutionsummary (task,MSK_STREAM_LOG);

    if ( r==MSK_RES_OK )
    {
      MSKsolstae solsta;

      if ( r==MSK_RES_OK )
        r = MSK_getsolsta (task,
            MSK_SOL_BAS,
            &solsta);
      switch(solsta)
      {
        case MSK_SOL_STA_OPTIMAL:   
        case MSK_SOL_STA_NEAR_OPTIMAL:
          {
            double *xx = (double*) calloc(numvar,sizeof(double));
            if ( xx )
            {
              MSK_getxx(task,
                  MSK_SOL_BAS,    /* Request the basic solution. */
                  xx);

              printf("Optimal primal solution\n");
              for(int j=0; j<numvar; ++j) {
                //printf("x[%d]: %e\n",j,xx[j]);
                networkFlowSol[j] = xx[j];

                //fprintf(fp,"%e\n",xx[j]);
              }

              free(xx);
            }
            else 
              r = MSK_RES_ERR_SPACE;

            break;
          }
        case MSK_SOL_STA_DUAL_INFEAS_CER:
        case MSK_SOL_STA_PRIM_INFEAS_CER:
        case MSK_SOL_STA_NEAR_DUAL_INFEAS_CER:
        case MSK_SOL_STA_NEAR_PRIM_INFEAS_CER:  
          printf("Primal or dual infeasibility certificate found.\n");
          break;
        case MSK_SOL_STA_UNKNOWN:
          {
            char symname[MSK_MAX_STR_LEN];
            char desc[MSK_MAX_STR_LEN];

            /* If the solutions status is unknown, print the termination code
               indicating why the optimizer terminated prematurely. */

            MSK_getcodedesc(trmcode,
                symname,
                desc);

            printf("The solution status is unknown.\n");
            printf("The optimizer terminitated with code: %s\n",symname);
            break;
          }
        default:
          printf("Other solution status.\n");
          break;
      }
    }
  }

  //fclose(fp);

  if (r != MSK_RES_OK)
  {
    /* In case of an error print error code and description. */      
    char symname[MSK_MAX_STR_LEN];
    char desc[MSK_MAX_STR_LEN];

    printf("An error occurred while optimizing.\n");     
    MSK_getcodedesc (r,
        symname,
        desc);
    printf("Error %s - '%s'\n",symname,desc);
  }

  /* Delete the task and the associated data. */
  MSK_deletetask(&task);
}
