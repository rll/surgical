// miranova.cc
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <sys/io.h>

#include "miranova.h"
#include "util.h"


// Miranova Sensor member functions
MiranovaSensor::MiranovaSensor(int c, double r, double o, int s, int slave) : Sensor(c, r, o, s) { 
	slave_num = slave;
}

MiranovaSensor::~MiranovaSensor(void) { }

void MiranovaSensor::Init(void) {
	miranova_init_channel(channel + (slave_num-1)*num_sensors, 0x7FFFFFL, 4);
}

void MiranovaSensor::Init(double v) {
	offset = v;
	miranova_init_channel(channel + (slave_num-1)*num_sensors, 0x7FFFFFL, 4);
}

void MiranovaSensor::Adjust(double value) { 
	long new_val = (long)floor((currentInput + value - offset) * sign * ratio + 0.5);

	new_val += 0x7FFFFFL;
	miranova_init_channel(channel + (slave_num-1)*num_sensors, new_val, 4);
}

int MiranovaSensor::IsReady(void) {
	int status;
	return miranova_read(channel + (slave_num-1)*num_sensors, status) != 0;
}

double MiranovaSensor::Input(void) {
	int status;
	long v = miranova_read(channel + (slave_num-1)*num_sensors, status) - 0x7FFFFFL;

	currentInput = (double)(v) / ratio;
	currentInput *= sign;
	currentInput += offset;
	
	//printf("ratio: %4.4f, input: %4.4f\n", ratio, currentInput);

	return currentInput;
}

const char *MiranovaSensor::GetName(void) {
	return "Miranova";
}

