#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/time.h>
#include "shared.h"

int main(void)
{
	unsigned long kernelClock = 0;
    Actuator* digital2  = new QuatechDigitalActuator(0, 0x08);
	Actuator* digital   = new QuatechDigitalActuator(0, 0x10);

	while(1)
	{
		digital->Output(0xff);
		digital2->Output(0xff);
        sleep(1);

		digital->Output(0x00);
		digital2->Output(0x00);
        sleep(1);

		kernelClock++;
		
		printf("kernel clock now %ld\n", kernelClock);
	}
	return 0;
}
