CC=g++ 
#CFLAGS=-O2 -w -c -std=c++11 -fopenmp -Wall
CFLAGS=-O0 -w -c -g -std=c++11 -fopenmp -Wall
PKGCONFIGFLAG=`pkg-config --cflags --libs opencv`

THEIAPATH=/home/rajvi/PhD/Projects/ViewGraphCVPR/theia-changes/

LIBPATH=-L$(THEIAPATH)/theia-build/lib/ -L/usr/local/lib/ -L/usr/lib/ -L/home/rajvi/Bundler/lib/ann_1.1_char/lib/ -L/home/rajvi/PhD/Projects/ThirdPartyCodes/GlobalSFM/ceres-solver/build/lib/ -L/home/rajvi/mosek/7/tools/platform/linux64x86/bin/

IFLAGS=-I/usr/local/include/ -I/usr/include/ -I/home/rajvi/PhD/Projects/openMVG-master/src/third_party/eigen/ -I/home/rajvi/Bundler/lib/ann_1.1_char/include/ -I$(THEIAPATH)/libraries/ -I$(THEIAPATH)/TheiaSfM/libraries/ -I/usr/local/include/theia/libraries/vlfeat/ -I/usr/local/include/theia/libraries/statx -I/usr/local/include/theia/libraries/optimo/ -I/home/rajvi/mosek/7/tools/platform/linux64x86/h/ -I/home/rajvi/PhD/Projects/ViewGraphCVPR/src/ 

LIBS=-lANN_char -lz -lglog -lceres -leasyexif -ltheia -lstlplus3 -leasyexif -lceres -lvlfeat -lmosek64


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

