#include "thread_minenergy.h"
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <sys/time.h>

#define num 2

int main(int, char *[])
{

	double curvature [num] = {1.0, 2.0};
	double torsion [num] = {0.5, 0.0};
	double length [num] = {1.0, 1.0};
  
  Vector3d positions[2];
  positions[0](0) = 0.0;
  positions[0](1) = 0.0;
  positions[0](2) = 0.0;
  positions[1](0) = 1.5;
  positions[1](1) = 0.0;
  positions[1](2) = 0.0;

  Vector3d tangents[2];
  tangents[0](0) = 1.0;
  tangents[0](1) = 1.0;
  tangents[0](2) = 0.0;
  tangents[1](0) = 1.0;
  tangents[1](1) = 0.0;
  tangents[1](2) = 0.0;


	/*for (int i = 0; i < 100; i++)
	{
		tangents[0](0) = 2*(rand()/(double)RAND_MAX) - 1;
		tangents[0](1) = 2*(rand()/(double)RAND_MAX) - 1;
		tangents[0](2) = 2*(rand()/(double)RAND_MAX) - 1;
		tangents[0].normalize();
		std::cout << "start tan: \n" << tangents[0] << std::endl;
		Thread thread(curvature, torsion, length, num, positions, tangents);

	}*/

  struct timeval start, end;
  gettimeofday(&start, 0x0);
	Thread thread(curvature, torsion, length, num, positions, tangents);

	thread.minimize_energy();
  gettimeofday(&end, 0x0);
  end.tv_usec += 1000000*(end.tv_sec - start.tv_sec);
  std::cout << "Elapsed time: " << (end.tv_usec - start.tv_usec)/1000 << " micro seconds" << std::endl;

	MatrixXd points(500,3);
	thread.getPoints(points);

	//std::cout << "points " << points << std::endl;

  Eigen::IOFormat fileForm(15); //set precision
	std::ofstream file;
	file.open("points.txt");
  file << points.format(fileForm);
	file.close();


}

