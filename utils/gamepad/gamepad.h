#ifndef GAMEPAD_H
#define GAMEPAD_H

// written by morgan quigley, 2 december 2005
// mquigley@cs.stanford.edu

// this code is a derivative of the standard Linux "jstest" utility

// this class works with my gamepad (a "thrustmaster vibrating gamepad")
// which has two joysticks (which double as buttons when pushed),
// 10 other buttons, and a point-of-view hat which I'm not using in this code
// with a little twiddling it should work with just about any other
// joystick that Linux recognizes and mounts in /dev/input/js0 or some 
// other similar place

class Gamepad
{
	public:
		Gamepad(const char *device_path = "/dev/input/js0");

		~Gamepad();
		
		int fd; // file descriptor of the joystick's data stream
		unsigned char num_axes, num_buttons;
		int *axes, *buttons;
		bool read_thread_alive, read_thread_must_die;
		
		void start_reading();
		void shutdown_read_thread();

		/////////////////////////////////////////////		
		static void *static_read_func(void *ptr);
		void read_func();
};

#endif
