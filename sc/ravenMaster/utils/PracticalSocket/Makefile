MACHINE= $(shell uname -s)
HOSTNAME=-D$(shell uname -n | sed 's/-/_/g')

LIBS = -L/usr/lib/
COMPILEFLAG=
LDFLAG=-lrt

INCLUDES = -I/usr/local/include/ -I/usr/include/
OPT_COMP = -DEIGEN_NO_DEBUG -O3
CC = g++ $(OPT_COMP) $(COMPILEFLAG) $(HOSTNAME)
LD = g++ $(OPT_COMP) $(LDFLAG) $(HOSTNAME) 
DEBUG = -ggdb

target: BroadcastSender

BroadcastSender: BroadcastSender.cpp PracticalSocket.h PracticalSocket.o
	$(LD) $(INCLUDES) $(DEBUG) -o BroadcastSender BroadcastSender.cpp PracticalSocket.o $(LIBS)
	
PracticalSocket.o: PracticalSocket.cpp PracticalSocket.h
	$(CC) $(INCLUDES) $(DEBUG) -c PracticalSocket.cpp 

clean:
	rm -r BroadcastSender *.o 
