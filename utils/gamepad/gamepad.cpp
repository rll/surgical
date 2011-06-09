#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/types.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <linux/joystick.h>
#include "Woodward.h"
#include "gamepad.h"

// on my gamepad, buttons 8 and 9 are the triggers (left&right, respectively)

Gamepad::Gamepad(const char *device_path) :
	num_axes(2),
	num_buttons(2),
	axes(NULL),
	buttons(NULL),
	read_thread_alive(false),
	read_thread_must_die(false)
{
	if ((fd = open(device_path, O_RDONLY)) < 0) {
		perror("Gamepad constructor");
		return;
	}
	
	ioctl(fd, JSIOCGAXES, &num_axes);
	ioctl(fd, JSIOCGBUTTONS, &num_buttons);

	num_axes = 5; // no matter what the driver says, I know it has 4 axes

	axes = new int[num_axes];
	buttons = new int[num_buttons];
	
	int i;
	for (i = 0; i < num_axes; i++)
		axes[i] = 0;
	for (i = 0; i < num_buttons; i++)
		buttons[i] = 0;
}

Gamepad::~Gamepad()
{
	// shut down the read thread if it hasn't died already
	shutdown_read_thread();
}

void Gamepad::shutdown_read_thread()
{
	read_thread_must_die = true;
	// give it 10ms to finish its current read
	usleep(10000);
	// although, because i'm not using select() on the read (yet),
	// if there is no incoming data, it won't get out of the read() blocking call
	// so we'll just kill the thread anyway when we dump the file descriptor.
	// the original jstest.c does it the right way, but i'm in a hurry today (and lazy)
}

void *Gamepad::static_read_func(void *ptr)
{
	Gamepad *parent = (Gamepad *)ptr;
	parent->read_func(); // so we can access non-static data members
	return NULL;
}

void Gamepad::read_func()
{
	struct js_event js;
	while (!read_thread_must_die)
	{
		if (read(fd, &js, sizeof(struct js_event)) != sizeof(struct js_event)) 
		{
			perror("\ngamepad: error reading");
			break;
		}

		switch(js.type & ~JS_EVENT_INIT) 
		{
			case JS_EVENT_BUTTON:
				buttons[js.number] = js.value;
//				printf("button %d value %d\n", js.number, js.value);
				break;
			case JS_EVENT_AXIS:
				if (js.number < num_axes) // on my gamepad, there are non-existent axes we ignore
					axes[js.number] = js.value;
				break;
		}
	}
	read_thread_alive = false;
}

void Gamepad::start_reading()
{
	wdCreateThread(static_read_func, this);
}
