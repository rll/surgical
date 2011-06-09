# include <opencv/cv.h>
# include <iostream>
# include <fstream>

# define PI 3.14159265
# define RADIANS_PER_DEGREE (CV_PI/180)
# define GRAYSCALE_IMAGE 0
# define COLOR_IMAGE 1

using namespace cv;

void printMat(Mat& mat);
void printMatCerr(Mat& mat);
template <class T> T matMax(Mat& mat);
float floatMatMax(Mat& mat);
void sPrintMat(Mat& mat);
void iPrintMat(Mat& mat);
Mat dgauss(float sig);
Mat stupidDivide(Mat& mat, float z);
Mat draw_lines(Mat &disp, vector<Vec2f> &lines);
void printArray(float arr[], int n);
void printArray(float arr[], int m, int n);

template <typename ty>
void saveToFile(Mat &mat, std::string filename) {
    std::ofstream outfile;
    outfile.open(filename.c_str());
    for (int i=0; i<mat.size().height; i++) {
        for (int j=0; j<mat.size().width; j++) {
            ty val = mat.at<ty>(i,j);
            outfile << val << " ";   
        }
        outfile << std::endl;
    }
    outfile.close();
}

template <typename ty>
void saveArrayToFile(ty arr[], int n, std::string filename) {
    std::ofstream outfile;
    outfile.open(filename.c_str());
    for (int i=0; i<n; i++) {
        ty val = arr[i];
        outfile << val << " ";   
    }
    outfile << std::endl;
    outfile.close();
}

int sround(float x);

template <typename ty>
Mat readFromTxt(std::string filename, int ny, int nx, int type) {
    Mat result = Mat(ny, nx, type);
    
    std::ifstream infile;
    infile.open(filename.c_str());
    std::string line;
    for (int r=0; r<ny; r++) {
        getline(infile, line);
        std::istringstream linestream(line);
        std::string num;
        for (int c=0; c<nx; c++) {
            linestream >> num;
            result.at<ty>(r,c) = atof(num.c_str());
        }
    }
    return result;
}

