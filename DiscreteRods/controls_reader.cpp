#include "controls_reader.h"


Controls_Reader::Controls_Reader(const char* fileName_controls_in)
{
 sprintf(_fileName_controls, "%s", fileName_controls_in);
}


void Controls_Reader::read_controls_from_file()
{
  std::cout << "filename: " << _fileName_controls << std::endl;
  ifstream controls_playback(_fileName_controls);
  string line; 
  while(!controls_playback.eof()) {
    VectorXd ctrl(12);
    float value;
    for (int i = 0; i < 12; i++) {
      controls_playback >> value;
      ctrl(i) = value; 
    }
    _each_control.push_back(ctrl);     
  }
  _each_control.pop_back(); // last control is garbage 
}

void Controls_Reader::get_all_controls(vector<VectorXd>& controls_out) {
  controls_out.resize(0);
  controls_out.resize(_each_control.size());
  for (int i = 0; i < _each_control.size(); i++) {
    controls_out[i] = _each_control[i]; 
  }

}


