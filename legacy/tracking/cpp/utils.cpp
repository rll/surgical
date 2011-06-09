# include "utils.h"

void printMat(Mat& mat) {
    for (int i=0; i<mat.size().height; i++) {
        for (int j=0; j<mat.size().width; j++) {
            std::cout << mat.at<float>(i,j) << " ";
        }
        std::cout << std::endl;
    }
} 

void printMatCerr(Mat& mat) {
    for (int i=0; i<mat.size().height; i++) {
        for (int j=0; j<mat.size().width; j++) {
            std::cerr<< mat.at<float>(i,j) << " ";
        }
        std::cerr << std::endl;
    }
} 

void sPrintMat(Mat& mat) {
    for (int i=0; i<mat.size().height; i++) {
        for (int j=0; j<mat.size().width; j++) {
            std::cout << mat.at<short>(i,j) << " ";
        }
        std::cout << std::endl;
    }
} 

void iPrintMat(Mat& mat) {
    for (int i=0; i<mat.size().height; i++) {
        for (int j=0; j<mat.size().width; j++) {
            std::cout << mat.at<int>(i,j) << " ";
        }
        std::cout << std::endl;
    }
} 

template <class T>
T matMax(Mat& mat) {   
    T mx = 0;
    bool first = true;
    
    for (int i=0; i<mat.size().height; i++) {
        for (int j=0; j<mat.size().width; j++) {
            if (first) {
                mx = mat.at<T>(i,j);
                first = false;
            }
            
            if (mat.at<T>(i,j) > mx) {
                mx = mat.at<T>(i,j);
            }
        }
        std::cout << std::endl;
    }
    
    return mx;
}

float floatMatMax(Mat& mat) {   
    float mx = 0;
    bool first = true;
    
    for (int i=0; i<mat.size().height; i++) {
        for (int j=0; j<mat.size().width; j++) {
            if (first) {
                mx = mat.at<float>(i,j);
                first = false;
            }
            
            if (mat.at<float>(i,j) > mx) {
                mx = mat.at<float>(i,j);
            }
        }
    }
    
    return mx;
}

Mat dgauss(float sig) {
    Mat dg(1, 2*ceil(3*sig)+1, CV_32F);
    
    for (int i=0; i<dg.size().width; i++) {
        float x = i-ceil(3*sig);
        float val = (-x/(sig*sig)) * (1/sqrt(2*PI)/sig) * exp( -x*x /(2*sig*sig) );
        dg.at<float>(0,i) =  val;
    }
    return dg;
}

Mat stupidDivide(Mat &mat, float z) {
    Mat result = Mat(mat.size(), mat.type());
    for (int i=0; i<mat.size().height; i++) {
        for (int j=0; j<mat.size().width; j++) {
            result.at<float>(i,j) = mat.at<float>(i,j)/z;
        }
    }
    
    return result;
}

Mat draw_lines(Mat &disp, vector<Vec2f> &lines) {
    Mat disp2;
    cvtColor(disp, disp2, CV_GRAY2RGB);    
    for( size_t i = 0; i < lines.size(); i++ )
    {
        float rho = lines[i][0];
        float theta = lines[i][1];
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        Point pt1(cvRound(x0 + 1000*(-b)), cvRound(y0 + 1000*(a)));
        Point pt2(cvRound(x0 - 1000*(-b)), cvRound(y0 - 1000*(a)));
        line(disp2, pt1, pt2, Scalar(0,255 ,0), 1, 4);
    }    
    return disp2;
}

void printArray(float arr[], int n) {
    for (int i=0; i<n; i++) {
        std::cout << arr[i] << " ";   
    }
    std::cout << std::endl;
}

void printArray(float* arr[], int m, int n) {
    for (int i=0; i<m; i++) {
        for (int j=0; j<n; j++) {
            std::cout << arr[i][j] << " ";   
        }
        std::cout << std::endl;
    }
}

int sround(float x) {
    int ceilx = ceil(x);
    int floorx = floor(x);
    if (x >= 0 && (ceilx-x) <= .5) {
        return ceilx;
    } else if (x >= 0) {
        return floorx;
    } else if (x < 0 && (x-floorx) <= .5) {
        return floorx;
    } else {
        return ceilx;
    }
}
