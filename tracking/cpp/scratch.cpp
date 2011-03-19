# include <opencv/cv.h>
# include <opencv/highgui.h>
# include <iostream> 
# include "utils.h"

using namespace cv;
int main(int argc, char** argv) {
    if (argc == 1) { 
        std::cerr << CV_8UC1 << " " << " " << CV_8SC1 << std::endl;
        std::cerr << CV_16UC1 << " "  << " " << CV_16SC1 << std::endl;
        std::cerr << " " << CV_32FC1 << " " << CV_32SC1 << std::endl;   
    }
    
    Mat im = imread("refim.ppm", 
    
    Mat img = readFromTxt<float>("Dx_locateMid.txt", 960, 1280, CV_32FC1);
    namedWindow("test", CV_WINDOW_AUTOSIZE);
    imshow("test", img);
    waitKey(0);
    saveToFile<float>(img, "test.txt");
    
    Mat A(3,3, DataType<unsigned int>::type);
    std::cout << "depth: " << DataType<unsigned char>::type << std::endl;
    std::cout << "depth: " << DataType<bool>::type << std::endl;
    std::cout << "depth: " << DataType<signed char>::type << std::endl;
    std::cout << "depth: " << DataType<unsigned short>::type << std::endl;
    std::cout << "depth: " << DataType<signed short>::type << std::endl;
    std::cout << "depth: " << DataType<int>::type << std::endl;
    std::cout << "depth: " << DataType<float>::type << std::endl;
    std::cout << "depth: " << DataType<double>::type << std::endl;
    
    float a = 3.6;
    unsigned char b = (unsigned char) round(a);
    std::cout << "roundtest: " << round(a) << " " << std::endl;
    std::cout << "rountest2: " << (int)b << std::endl;
    
    a = -1;
    int sgn = a/abs(a);
    std::cout << "sgn=" << sgn << std::endl;
    a = 3.14159;
    sgn = a/abs(a);
    std::cout << "sgn=" << sgn << std::endl;
    a = -3.14159;
    sgn = a/abs(a);
    std::cout << "sgn=" << sgn << std::endl;
}