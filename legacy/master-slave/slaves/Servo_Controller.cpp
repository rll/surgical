#include "Servo_Controller.h"
//#include <math.h>


Servo_Controller::Servo_Controller(std::string port)
{
	serial = new SimpleSerial(port,SERVO_BAUD);
	
	const char msgBase[] = {"swr 30 4 "};
	strcpy(_msgBase, msgBase);
}

Servo_Controller::~Servo_Controller()
{
}

void Servo_Controller::go_to_pos(int id[], int pos[], int num_servos)
{
	int speed[num_servos];
	for (int i = 0; i < num_servos; i++)
		speed[i] = DEFAULT_SPEED;

	go_to_pos(id, pos, speed, num_servos);
}

void Servo_Controller::go_to_pos(int id[], int pos[], int speed[], int num_servos)
{
	int msg_length = BASE_MSG_LENGTH+3+5*4*num_servos;
	char msg[msg_length];
	strcpy(msg, _msgBase);
	
	int curr_msg_pos = BASE_MSG_LENGTH;
	for (int i = 0; i < num_servos; i++)
	{
		sprintf(msg+curr_msg_pos, "%d", id[i]);
		curr_msg_pos += num_chars_from_int(id[i])+1;
		msg[curr_msg_pos-1] = ' ';

		int low_pos = pos[i] & 0xFF;
		int high_pos = (pos[i] & 0xFF00)>>8;
		
		sprintf(msg+curr_msg_pos, "%d", low_pos);
		curr_msg_pos += num_chars_from_int(low_pos)+1;
		msg[curr_msg_pos-1] = ' ';
		sprintf(msg+curr_msg_pos, "%d", high_pos);
		curr_msg_pos += num_chars_from_int(high_pos)+1;
		msg[curr_msg_pos-1] = ' ';

		int low_speed = speed[i] & 0xFF;
		int high_speed = (speed[i] & 0xFF00)>>8;
		sprintf(msg+curr_msg_pos, "%d", low_speed);
		curr_msg_pos += num_chars_from_int(low_speed)+1;
		msg[curr_msg_pos-1] = ' ';
		sprintf(msg+curr_msg_pos, "%d", high_speed);;
		curr_msg_pos += num_chars_from_int(high_speed)+1;
		msg[curr_msg_pos-1] = ' ';
		
	}
	msg[curr_msg_pos++] = '\n';
	//msg[curr_msg_pos++] = '\r';
	//msg[curr_msg_pos++] = '\0';

	serial->writeChars(msg, curr_msg_pos);


}
		

int Servo_Controller::num_chars_from_int(int num)
{
	num = abs(num);
	/*if (num <= 1)
		return 1;
	
	return 1+(int)(log10((float)num));
	*/


	int num_chars = 1;
	while (num >= 10)
	{
		num_chars++;
		num = num/10;
	}	
	return num_chars;
}


