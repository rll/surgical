#include "CannyReader.h"
#include <json/json.h>
#include <sstream>


CannyReader::CannyReader(const char *inFile)
{
	strcpy(srcFile, inFile);
}


bool CannyReader::readFromFile()
{
	Timer aTimer;
	std::ifstream cannyFile(srcFile);
	cannyFile.precision(20);
	if (cannyFile.is_open()) {
		std::stringstream buffer;
		buffer << cannyFile.rdbuf();

		std::string contents(buffer.str());
		cout << "Time to Read: " << aTimer.elapsed() << endl;
		aTimer.restart();

		Json::Value root;
		Json::Reader reader;
		bool parsingSuccessful = reader.parse( contents, root );
		cout << "Time to parse: " << aTimer.elapsed() << endl;
		aTimer.restart();
		if (parsingSuccessful)
		{
			for( Json::ValueIterator outer = root.begin() ; outer != root.end() ; outer++ ) {
				int camNum = outer.key().asInt();

				Json::Value oneCam = root[camNum];

				for( Json::ValueIterator itr = oneCam.begin() ; itr != oneCam.end() ; itr++ ) {
					vector<Line_Segment *>* lineSegments = new vector<Line_Segment *>();

					int key = atoi(itr.key().asCString());

					Json::Value jsonSegments = oneCam[itr.key().asCString()];
					for( Json::ValueIterator itr2 = jsonSegments.begin() ; itr2 != jsonSegments.end() ; itr2++ ) {
						Json::Value jsonSeg = jsonSegments[itr2.key().asInt()];
						Line_Segment* seg = (Line_Segment *) malloc(sizeof(Line_Segment));
						seg->row1 = jsonSeg["r1"].asInt();
						seg->col1 = jsonSeg["c1"].asInt();
						seg->row2 = jsonSeg["r2"].asInt();
						seg->col2 = jsonSeg["c2"].asInt();

						lineSegments->push_back(seg);
					}
					precomputedCannySegments[camNum][key] = lineSegments;
				}
			}
			cannyFile.close();
			cout << "Time to reconstruct: " << aTimer.elapsed() << endl;

			return true;
		}
		else {
			return false;
		}
	}
	return false;
}

