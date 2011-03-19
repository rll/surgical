#ifndef _Servo_Controller_h
#define _Servo_Controller_h

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>

#include "SimpleSerial.h"

#define SERVO_BAUD 57600
#define DEFAULT_SPEED 400
#define BASE_MSG_LENGTH 9

class Servo_Controller
{
	public:
		Servo_Controller(std::string port);
		~Servo_Controller();
		void go_to_pos(int id[], int pos[], int num_servos);
		void go_to_pos(int id[], int pos[], int speed[], int num_servos);

	private:
		char _msgBase[BASE_MSG_LENGTH];
		SimpleSerial * serial;

		int num_chars_from_int(int num);
};
#endif
