CC=g++ 
#CFLAGS=-O2 -w -c -std=c++11 -fopenmp -Wall
CFLAGS=-O0 -w -c -g -std=c++11 -fopenmp -Wall
PKGCONFIGFLAG=`pkg-config --cflags --libs opencv`

THEIAPATH=/home/rajvi/ViewGraphSelection/theia-changes/

LIBPATH=-L$(THEIAPATH)/build/lib/ -L/usr/local/lib/ -L/usr/lib/ -L/home/rajvi/Bundler/lib/ -L/home/rajvi/Bundler/lib/zlib/ -L/home/rajvi/mosek/7/tools/platform/linux64x86/bin/ -L/usr/lib/x86_64-linux-gnu/

IFLAGS=-I/usr/local/include/ -I/usr/include/ -I/usr/include/eigen3/ -I/home/rajvi/Bundler/lib/ann_1.1_char/include/ -I$(THEIAPATH)/libraries/ -I$(THEIAPATH)/TheiaSfM/libraries/ -I/usr/local/include/theia/libraries/vlfeat/ -I/usr/local/include/theia/libraries/statx -I/usr/local/include/theia/libraries/optimo/ -I/usr/local/include/theia/libraries/cereal/include/ -I/usr/include/suitesparse/ -I/home/rajvi/mosek/7/tools/platform/linux64x86/h/ -Isrc/ 

#LIBS=-lz -lANN_char -llapack -lopenblas -latlas -lsuitesparseconfig -lcxsparse -lcholmod -lamd -lcolamd -lccolamd -lcamd -lrt -lblas -lglog -lceres -lsuitesparseconfig -lcxsparse -lsuitesparseconfig -lcholmod -lamd -lcolamd -lccolamd -lcamd -lrt -lblas -leasyexif -ltheia -lstlplus3 -leasyexif -lceres -lvlfeat -liomp5 -lmosek64 -lpthread -lcxsparse -lcholmod -lamd -lcolamd -lccolamd -lcamd -lsuitesparseconfig -fopenmp -lblas -lcblas  

#LIBS=-lz -lANN_char -llapack -lopenblas -latlas -lglog -lceres -lsuitesparseconfig -lcxsparse -lsuitesparseconfig -lcholmod -lamd -lcolamd -lccolamd -lcamd -lrt -lblas -leasyexif -ltheia -lstlplus3 -leasyexif -lceres -lvlfeat -liomp5 -lmosek64 -lpthread -lcxsparse -lcholmod -lamd -lcolamd -lccolamd -lcamd -lsuitesparseconfig -fopenmp -lblas -lcblas  

#LIBS=-lz -lANN_char -llapack -lopenblas -lglog -lceres -lblas -ltheia -leasyexif -lstlplus3 -lceres -lvlfeat -liomp5 -lmosek64 -lpthread -lcxsparse -lcholmod -lamd -lcolamd -lccolamd -lcamd -lsuitesparseconfig -fopenmp -lblas -lcblas  

#LIBS=-lz -lANN_char -lglog -lceres -ltheia -leasyexif -lstlplus3 -lceres -lvlfeat -liomp5 -lmosek64 -lpthread -llapack -lopenblas -lblas -lcholmod -lamd -lcolamd -lccolamd -lcamd -lcxsparse -lsuitesparseconfig -fopenmp  

LIBS=-lz -lANN_char -lglog -lceres -ltheia -leasyexif -lstlplus3 -lceres -lvlfeat -liomp5 -lmosek64 -lpthread -llapack -lopenblas -lcholmod -lamd -lcolamd -lccolamd -lcamd -lcxsparse -lsuitesparseconfig -fopenmp  

SOURCES = $(wildcard src/*.cpp) 
APPLICATIONS = $(wildcard applications/*.cpp)
SRC_OBJECTS = $(SOURCES:.cpp=.o)
APP_OBJECTS = $(APPLICATIONS:.cpp=.o)

BINARIES := $(patsubst %.cpp,%,$(APPLICATIONS))
$(info VAR="$(BINARIES)")
$(info VAR="$(APP_OBJECTS)")
$(info VAR="$(SRC_OBJECTS)")

all: $(BINARIES)
	mv $(BINARIES) bin/

OBJ = $(patsubst %,%.o,$@)
BIN = $@

$(BINARIES): $(APP_OBJECTS) $(SRC_OBJECTS)
	$(CC) $(IFLAGS) $(SRC_OBJECTS) $(OBJ) $(LIBPATH) $(PKGCONFIGFLAG) -o $(BIN) $(LIBS)

# Generic rule
%.o: %.cpp
	$(CC) $(CFLAGS) $(IFLAGS) $< -o $@

clean:
	rm src/*.o applications/*.o $(BINARIES)

