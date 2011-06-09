#include "LowPassFilter.h"
#include <iostream>

LowPassFilter::LowPassFilter(const double* a, const double* b, int order, int nfilters, const double* init_filter_values)
{
  m_initialized_filter_state = 0;

  morder = order;
  mnfilters = nfilters;

  ma = new double[morder];
  mb = new double[morder];

  for (int i=0; i<morder; ++i){
    ma[i] = a[i];
    mb[i] = b[i];
  }


  mx = new double*[mnfilters];
  my = new double*[mnfilters];

  for (int i=0; i<mnfilters; ++i)
    mx[i] = new double[morder];

  for (int i=0; i<mnfilters; ++i)
    my[i] = new double[morder];

  if(init_filter_values!=0){
    for (int i=0; i<mnfilters; ++i){
      for (int k=0; k<morder; ++k){
        mx[i][k] = init_filter_values[i];
        my[i][k] = init_filter_values[i];
      }
    }
    m_initialized_filter_state = 1;
  }

}

LowPassFilter::~LowPassFilter()
{
  delete[] ma;
  delete[] mb;

  for (int i=0; i<mnfilters; ++i){
    delete[] mx[i];
    delete[] my[i];
  }

  delete[] mx;
  delete[] my;


}

void LowPassFilter::Update(const double* x, double* y)
{

  /*	std::cout << "x: ";
	for (int k=0; k<morder-1; ++k){
        std::cout << x[k] << " ";
	}
	std::cout << std::endl;

	std::cout << "y: ";
	for (int k=0; k<morder-1; ++k){
        std::cout << y[k] << " ";
	}
	std::cout << std::endl;


	std::cout << "mx: ";
	for (int k=0; k<morder; ++k){
        std::cout << mx[0][k] << " ";
	}
	std::cout << std::endl;

	std::cout << "my: ";
	for (int k=0; k<morder; ++k){
        std::cout << my[0][k] << " ";
	}
	std::cout << std::endl;

	char c;

        //	std::cin >> c;
	*/

  if(!m_initialized_filter_state){
    for (int i=0; i<mnfilters; ++i){
      for (int k=0; k<morder; ++k){
        mx[i][k] = x[i];
        my[i][k] = x[i];
      }
    }
    m_initialized_filter_state = 1;
  }

  // shift all values:
  for (int i=0; i<mnfilters; ++i){


    for (int k=0; k<morder-1; ++k){
      mx[i][k] = mx[i][k+1];
      my[i][k] = my[i][k+1];
    }
    mx[i][morder-1] = x[i];

  }

  for (int i=0; i<mnfilters; ++i){
    my[i][morder-1] = 0;
    for (int k=0; k<morder; ++k)
      my[i][morder-1] += mb[k]*mx[i][morder-k-1];
    for (int k=1; k<morder; ++k)
      my[i][morder-1] -= ma[k]*my[i][morder-k-1];

    y[i] = my[i][morder-1];
  }

  /*
    std::cout << "mx: ";
    for (int k=0; k<morder; ++k){
    std::cout << mx[0][k] << " ";
    }
    std::cout << std::endl;

    std::cout << "my: ";
    for (int k=0; k<morder; ++k){
    std::cout << my[0][k] << " ";
    }
    std::cout << std::endl;

    //	std::cin >> c;
    */

}


void LowPassFilter::Y(double* y)
{
  for (int i=0; i<mnfilters; ++i){
    y[i] = my[i][morder-1];
  }
}




