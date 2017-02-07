The code is run in the following steps.

1) Estimate pairwise geometry, compute costs, and save to disk.
2) Construct network using saved connections and costs and specified flow and weights. 
3) Write selected viewgraph, named as per the choice of parameters to disk.
OR
3) Write NVM File corresponding to selected viewgraph, named as per the choice of parameters to disk. 

Externally,

Scripts run step1,
Step2 and 3 are run multiple times with different choice of parameters. 

Final Step,
Run GlobalSFM with output view-graph or VSFM with output NVMs to reconstruct. 

