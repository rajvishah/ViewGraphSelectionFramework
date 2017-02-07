CC=g++ 
#CFLAGS=-O2 -w -c -std=c++11 -fopenmp -Wall
CFLAGS=-O0 -w -c -g -std=c++11 -fopenmp -Wall
PKGCONFIGFLAG=`pkg-config --cflags --libs opencv`

THEIAPATH=/home/rajvi/PhD/Projects/OptProject/DuplicateStructures/git_repo/duplicatestructures/theia-changes/

LIBPATH=-L$(THEIAPATH)/theia-build/lib/ -L/usr/local/lib/ -L/usr/lib/ -L/home/rajvi/Bundler/lib/ann_1.1_char/lib/ -L/home/rajvi/PhD/Projects/ThirdPartyCodes/GlobalSFM/ceres-solver/build/lib/ -L/home/rajvi/mosek/7/tools/platform/linux64x86/bin/

IFLAGS=-I/usr/local/include/ -I/usr/include/ -I/home/rajvi/PhD/Projects/openMVG-master/src/third_party/eigen/ -I/home/rajvi/Bundler/lib/ann_1.1_char/include/ -I$(THEIAPATH)/libraries/ -I$(THEIAPATH)/TheiaSfM/libraries/ -I/usr/local/include/theia/libraries/vlfeat/ -I/usr/local/include/theia/libraries/statx -I/usr/local/include/theia/libraries/optimo/ -I/home/rajvi/mosek/7/tools/platform/linux64x86/h/ 

LIBS=-lANN_char -lz -lglog -lceres -leasyexif -ltheia -lstlplus3 -leasyexif -lceres -lvlfeat -lmosek64

all: compute_geometry run_nw_flow write_viewgraph write_nvms
# File names
EXEC1 = compute_geometry 
EXEC2 = run_nw_flow
EXEC3 = write_viewgraph
EXEC4 = write_nvms
SOURCES = $(wildcard *.cpp)
OBJECTS = $(SOURCES:.cpp=.o)

exclude1=solve_flow.o make_viewgraphs.o write_nvm_matches.o
exclude2=main.o make_viewgraphs.o write_nvm_matches.o
exclude3=main.o solve_flow.o write_nvm_matches.o
exclude4=main.o solve_flow.o make_viewgraphs.o

# make compute_geometry
OBJECTS1=$(filter-out $(exclude1),$(OBJECTS))
$(EXEC1):$(OBJECTS1) 
	$(CC) $(IFLAGS) $(OBJECTS1) $(LIBPATH) $(PKGCONFIGFLAG) -o $(EXEC1) $(LIBS)

# Make write_vg
OBJECTS3=$(filter-out $(exclude3),$(OBJECTS))
$(EXEC3): $(OBJECTS3)
	$(CC) $(IFLAGS) $(OBJECTS3) $(LIBPATH) $(PKGCONFIGFLAG) -o $(EXEC3) $(LIBS)

# Main solve_flow
OBJECTS2=$(filter-out $(exclude2),$(OBJECTS))
$(EXEC2):$(OBJECTS2) 
	$(CC) $(IFLAGS) $(OBJECTS2) $(LIBPATH) $(PKGCONFIGFLAG) -o $(EXEC2) $(LIBS)

OBJECTS4=$(filter-out $(exclude4),$(OBJECTS))
$(EXEC4):$(OBJECTS4) 
	$(CC) $(IFLAGS) $(OBJECTS4) $(LIBPATH) $(PKGCONFIGFLAG) -o $(EXEC4) $(LIBS)

$(info VAR="$(OBJECTS2)")


# Generic rule
%.o: %.cpp
	$(CC) $(CFLAGS) $(IFLAGS) $< -o $@

clean:
	rm *.o compute_geometry solve_flow write_viewgraph
print-%:
	@echo '$*=$($*)'

