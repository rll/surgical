#include "CannyWriter.h"
#include <json/json.h>
#include <sstream>


CannyWriter::CannyWriter(const char *outFile) : numCams(0)
{
	strcpy(dstFile, outFile);
}


void CannyWriter::writeToFile()
{
	std::ofstream cannyFile;
	cannyFile.precision(20);
	cannyFile.open(dstFile);

	cannyFile << jsonString();

	cannyFile.close();
}

std::string CannyWriter::jsonString ()
{
    Json::Value root(Json::arrayValue);

	for (int cam = 0; cam < numCams; cam++) {
		Json::Value camSegments(Json::objectValue);
	
		for (map<int, vector<Line_Segment*>*>::iterator i = precomputedCannySegments[cam].begin(); i != precomputedCannySegments[cam].end(); i++) {
			vector<Line_Segment*>* lineSegments = i->second;

			if (lineSegments->size() > 0) {
				Json::Value segmentVector(Json::arrayValue);
				for (int j = 0; j < lineSegments->size(); j++) {
					Json::Value pair(Json::objectValue);
					pair["r1"] = lineSegments->at(j)->row1;
					pair["c1"] = lineSegments->at(j)->col1;
					pair["r2"] = lineSegments->at(j)->row2;
					pair["c2"] = lineSegments->at(j)->col2;
					segmentVector.append(pair);
				}
				stringstream ss;
				ss << i->first;
				camSegments[ss.str()] = segmentVector;
			}
		}
		root.append(camSegments);
	}


    Json::StyledWriter writer;
    // Make a new JSON document for the configuration. Preserve original comments.
    std::string outputConfig = writer.write( root );
	return outputConfig;
}
