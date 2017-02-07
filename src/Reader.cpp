#include "Reader.h"

using namespace reader;

ImageAttr::ImageAttr(string n, int w, int h) {
	name = n;
	width = w;
	height = h;
}

ImageListReader::ImageListReader(string& baseDir) {
	listFileName = baseDir + string("/list_images.txt");
	dimsFileName = baseDir + string("/image_dims.txt");
}

void ImageListReader::initialize(string& baseDir) {
	listFileName = baseDir + string("/list_images.txt");
	dimsFileName = baseDir + string("/image_dims.txt");
}

bool ImageListReader::read() {
	ifstream imgFile(listFileName.c_str(), ifstream::in);
	ifstream dimsFile(dimsFileName.c_str(), ifstream::in);
	if( !imgFile.is_open() || ! dimsFile.is_open()) {
        cout << "\nError opening file (images/dimensions)";
		return false;
	}

    string line;
    vector<string> imgFileNames;
    while(getline(imgFile, line)) {
        imgFileNames.push_back(line);
    }

    int numImages = imgFileNames.size();
    vector< int > widths( numImages );
    vector< int > heights( numImages );
    for(int i=0; i < imgFileNames.size(); i++) {
        dimsFile >> widths[i] >> heights[i];
    } 

    for(int i=0; i < numImages; i++) {
		ImageAttr att(imgFileNames[i], widths[i], heights[i]);
		imAttr.push_back(att);			
	}

	imgFile.close();
  dimsFile.close();
	return true;
}

KeyListReader::KeyListReader(string& baseDir) {	
	keyFileName = baseDir + string("/list_keys.txt");
}

void KeyListReader::initialize(string& baseDir){	
	keyFileName = baseDir + string("/list_keys.txt");
}

bool KeyListReader::read() {
	ifstream file(keyFileName.c_str(), ifstream::in);
	if( ! file.is_open() ) {
        printf("\nError opening key list file");
		return false;
	}

	keyNames.clear();
    string line;
    while(getline(file, line)) {	
		keyNames.push_back(line);			
	}
	file.close();
	return true;
}
	
NSiftReader::NSiftReader(string& baseDir) { 
	nSiftFileName = baseDir + string("/num_sifts.txt");
}

void NSiftReader::initialize(string& baseDir) {
	nSiftFileName = baseDir + string("/num_sifts.txt");
}

bool NSiftReader::read() {
	ifstream file(nSiftFileName.c_str(), ifstream::in);
	if( ! file.is_open() ) {
        printf("\nError opening num sift file");
		return false;
	}

	numSifts.clear();
    string line;
	while(getline(file, line)) {
		int n = atoi ( line.c_str() );	
		numSifts.push_back(n);			
	}
	file.close();
	return true;
}


