#Param 01 k1 k2 k3 k4 k5 k6 high low
#
#sym  ramp highstep ramp 
#ABC 1 0 0 0.25 0.75 1 1 1 0
#
#sym lowstep ramp highstep ramp lowstep
#ABC 1 0 0.165 0.333 0.666 0.835 1 1 0
#
#sym lowstep highstep lowstep
#ABC 1 0 0.25 0.25 0.75 0.75 1 1 0
#
#sym ramp ramp
#ABC 1 0 0 0.5 0.5 1 1 1 0
#
#sym lowstep ramp ramp lowstep
#ABC 1 0 0.165 0.5 0.6 0.835 1 1 0
#
#oneside ramp (low --> high)
#ABC 1 0 0 1 1 1 1 1 0
#
#oneside ramp (high --> low)
#ABC 1 0 0 0 0 1 1 1 1 0
#
#oneside lowstep ramp (low --> high)
#ABC 1 0 0.165 1 1 1 1
#
#oneside highstep ramp (high --> low)
#
#oneside lowstep highstep
#prefer an image that is connected to many images
NODECONN 1 0 0 0 0 1 1 0 -1
#prefer an image that has many of its features connected
FEATCONN 1 0 0 0 0 1 1 0 -1
#prefer an image that is strongly connected (disjoint set selection)
DISJOINT 1 0 0 0 0 1 1 0 -1
#prefer an image pair with moderate (>20 & < 70) % overlap
OVERLAPSUM 0 0 0.10 0.50 0.50 0.80 1 0 -1 
#OVERLAPMULT 0
#prefer an image pair with no rotational homograpghy (good baseline) (> 0)
HOMOGRAPHY 1 0 0.10 0.80 1 1 1 0 -1 
#
CTXCHISQR 1 0 0 0 0 1 1 0 -1
CTXHAMMING 1 0 0 0 0 1 1 0 -1
CTXCOSINE 1 0 0 0 0 1 1 0 -1
#TRIPLET 0
#small angle differences are good large angle diff are bad
MULTIMOTROT 0 0 5 75 180 180 180 0 -1
#dot product so 1 is good 0 is bad
MULTIMOTPOS 0 0 0 0 0.50 0.90 1 0 -1
#LOOPCONST
