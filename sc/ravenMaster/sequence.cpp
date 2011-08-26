#include <iostream>
#include <fstream>
#include <string>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <vector>

using namespace std;
using boost::lexical_cast;
using boost::bad_lexical_cast;

int getSequenceFromFile() {
	ifstream infile("sequence.txt");
	if (!infile.is_open()) {
		cerr << "Unable to open file." << endl;
  	return -1;
  }
	string line;
  vector<string> line_vect;
	while (!infile.eof()) {
    getline (infile, line);
    boost::split(line_vect, line, boost::is_any_of(" "));
    if (line_vect.size()==0) {
    	cerr << "Error in getting sequence. Sequence file is empty." << endl;
    	return -1;
    }
		try {
			int result = boost::lexical_cast<int>((line_vect.front()).c_str());
			infile.close();
			return result;
	  } catch(bad_lexical_cast &) {
			cerr << "First token in sequence file is not an int." << endl;
			return -1;
		}
  }
  infile.close();
  cerr << "Reached end of file before reading something from the file." << endl;
  return -1;
}

void setSequenceToFile(int seq) {
	ofstream outfile;
  outfile.open("sequence.txt");
  if (!outfile.is_open()) {
  	cerr << "Unable to open file." << endl;
  	return;
  }
  outfile << seq;
  outfile.flush();
  outfile.close();
}
