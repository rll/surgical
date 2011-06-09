#include "shared.h"
#include <stdio.h>
#include <sys/io.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>

#define num_acs 2

int main(int argc, char** argv) {
	int i;
	QuatechDACActuator* testDAC[8];

	for (i=0; i<8; i++) {
		testDAC[i] = new QuatechDACActuator(i, 1);
		testDAC[i]->Init();
	
		/* set voltage constraints */
		testDAC[i]->SetLimit(-0.1, 0.1);
		testDAC[i]->SetDither(0, 0);
	}
	
	/* test QuatechDACActuator */
 	char c = ' ', d = ' ';
	float f = 0.0f;
  	int status = 0;
	long encoder_setpoint[num_acs] = { 0x7FFFFF, 0x7FFFFF };
	//long encoder_setpoint[3] = { 0x801259, 0x8000eb, 0x7fecc7 };
	long encoder_value;

	float kp = 0.1f / 5500;
	kp *= -1;

	/* initialize miranova */
  	long val = 0L;
  	for(i=0; i<8; i++)
  		miranova_init_channel(i, 0x7FFFFFL, 4);

	int ac_index[num_acs] = { 0, 3 };
	int sen_index[num_acs] = { 2, 1 };
	printf("enter sensor channel\n");
	
	while(1) {
		for(int i=0; i < num_acs; ++i) {
			//int i = 2;
			encoder_value = miranova_read(sen_index[i], status);
			f = kp * (encoder_value - encoder_setpoint[i]);
			//printf("s[%d] value: %lx, f: %f\t", sen_index[i], encoder_value, f);
			testDAC[ac_index[i]]->Output(f);
			printf("send %f to channel %d\t", f, ac_index[i]);
		}
		printf("\n");

	}
	return 0;
}
