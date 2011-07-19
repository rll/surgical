#include <iostream>
#include <fstream>
#include <string>
#include <boost/algorithm/string.hpp>
#include <vector>

using namespace std;

int main(int argc, char* argv[]) {

	if (argc != 2) {
		cout << "Wrong number of arguments." << endl;
		return -1;
	}
  
  string line;
  bool first = true;
  vector<string> first_line_vect;
  vector<string> line_vect;
  
  ifstream infile(argv[1]);
  ofstream outfile;
  char outfilename[strlen(argv[1])+7];
  strcpy(outfilename, argv[1]);
  strcat(outfilename, ".format");
  outfile.open(outfilename);
  
  if (!infile.is_open() || !outfile.is_open()) {
  	cout << "Unable to open file." << endl;
  	return -1;
  }
  while (!infile.eof()) {
    getline (infile, line);
    if (first) {
			boost::split(first_line_vect, line, boost::is_any_of(" "));
			outfile << line;
			outfile << endl;
			first = false;
			continue;
		} else {
			boost::split(line_vect, line, boost::is_any_of(" "));
			if ((line_vect.size() == 1) && (line_vect[0] == "") && infile.eof()) {
				cout << "File reformatting was successful." << endl;
				return 0;
			}
			if ((line_vect.size()>0 && line_vect[0]=="#") || (line_vect.size()>1 && line_vect[1]=="#")) {
				outfile << line;
				outfile << endl;
				continue;
			}
			if (first_line_vect.size() != line_vect.size()) {
				cout << "File not formated correctly. Different number of data." << endl;
				return -1;
			}
			for (int i=0; i<line_vect.size(); i++) {
				string fd = first_line_vect[i];
				string d = line_vect[i];
				if (fd.size() < d.size()) {
					cout << "Format data " << fd << " is smaller than data " << d << " ." << endl;
					return -1;
				}
				int space = fd.size()-d.size();
				while (space > 0) {
					outfile << " ";
					space--;
				}
				if (i != 0)
					outfile << " ";
				outfile << d;
			}
			outfile << endl;
		}
  }
  infile.close();
  outfile.close();

  return 0;
}

