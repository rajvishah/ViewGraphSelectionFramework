#ifndef __READER_H
#define __READER_H
#include "defs.h"


namespace reader {

class ImageAttr {
public:
	string name;
	int width;
	int height;

	ImageAttr(string n, int w, int h);
};

class ImageListReader {
	string listFileName;
	string dimsFileName;
	vector< ImageAttr > imAttr;

public:
	ImageListReader(string &par);
	bool read();
	void initialize(string &par);
	const ImageAttr& getImageAttr(int idx) { return imAttr[idx]; }
	const string& getImageName(int idx) { return imAttr[idx].name; }
	int getImageWidth(int idx) { return imAttr[idx].width; }
	int getImageHeight(int idx) { return imAttr[idx].height; }
    int getNumImages() {return (int)imAttr.size(); }
};
 
class KeyListReader {
	string keyFileName;
	vector< string > keyNames;

public:
	KeyListReader(string& par);
	void initialize(string& par);
	bool read();
	const string& getKeyName(int idx) { return keyNames[idx]; }
    int getNumKeys() {return (int)keyNames.size(); }
};

class NSiftReader {
	string nSiftFileName;
	vector< int > numSifts;

public:
	NSiftReader(string& par);	
	void initialize(string& par);
	bool read();
	int at(int idx) { 
		if((int)numSifts.size() > idx) 
			return numSifts.at(idx ); 
		else 
			return -1;
	}
};

}
#endif //__READER_H 
