#include "gamepad.h"
#include <stdio.h>
#include "Woodward.h"

static int die = 0;

void *MyThreadFunc(void *ptr) {
  while(1) {
    char ch = getchar();
		if(ch == 'q') {
      die = 1;
      break;
    }
  }
  return 0;
}

int main(int argc, char **argv) {
  printf("test_gamepad. what a killer program\n");
  wdCreateThread(MyThreadFunc, 0);
  Gamepad *gamepad = new Gamepad();
  gamepad->start_reading();

  while(!die) 
  {
//wdSleep(10);
    int i;
    printf("axes: ");
		for (i = 0; i < gamepad->num_axes; i++)
			printf("%6d ", gamepad->axes[i]);
		printf("  buttons: ");
		for (i = 0; i < gamepad->num_buttons; i++)
			printf("%d ", gamepad->buttons[i]);
		printf("\r");
  }

  delete gamepad;

  return 0;
}
