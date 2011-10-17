#include <iostream>
#include <fstream>
#include <string>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <vector>

using namespace std;
using boost::lexical_cast;
using boost::bad_lexical_cast;

int getSequenceFromFile();
void setSequenceToFile(int seq);
