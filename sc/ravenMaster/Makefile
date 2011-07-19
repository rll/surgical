MACHINE= $(shell uname -s)

LIBS = -L/usr/lib/
COMPILEFLAG=-lpthread
LDFLAG=-Wno-write-strings

UTILS_DIR = utils/
INCLUDES = -I/usr/local/include/ -I/usr/include/ -I/usr/include/eigen2
OPT_COMP = -DEIGEN_NO_DEBUG -O3
CC = g++ $(OPT_COMP) $(COMPILEFLAG) 
LD = g++ $(OPT_COMP) $(LDFLAG)  
DEBUG = -ggdb

target: parser

ravenMasterReceiver: ravenMasterReceiver.cpp ravenMasterReceiver.h PracticalSocket.o Log.o _kbhit.o _getch.o
	$(LD) $(INCLUDES) $(DEBUG) -o ravenMasterReceiver ravenMasterReceiver.cpp PracticalSocket.o Log.o _kbhit.o _getch.o $(LIBS)

ravenMasterSender: ravenMasterSender.cpp ravenMasterSender.h PracticalSocket.o Log.o sequence.o
	$(LD) $(INCLUDES) $(DEBUG) -o ravenMasterSender ravenMasterSender.cpp PracticalSocket.o Log.o sequence.o $(LIBS)

ravenMasterCmd: ravenMasterCmd.cpp ravenMasterCmd.h sequence.o PracticalSocket.o Log.o
	$(LD) $(INCLUDES) $(DEBUG) -o ravenMasterCmd ravenMasterCmd.cpp PracticalSocket.o Log.o $(LIBS)
	
reformat: reformat.cpp
	$(LD) $(INCLUDES) $(DEBUG) -o reformat reformat.cpp $(LIBS)

parser: parser.cpp
	$(LD) $(INCLUDES) $(DEBUG) -o parser parser.cpp $(LIBS)
resetSequence: resetSequence.cpp sequence.o
	$(LD) $(INCLUDES) $(DEBUG) -o resetSequence resetSequence.cpp sequence.o $(LIBS)

sequence.o: sequence.cpp sequence.h
	$(CC) $(INCLUDES) $(DEBUG) -c sequence.cpp

PracticalSocket.o: $(UTILS_DIR)PracticalSocket/PracticalSocket.cpp $(UTILS_DIR)PracticalSocket/PracticalSocket.h
	$(CC) $(INCLUDES) $(DEBUG) -c $(UTILS_DIR)PracticalSocket/PracticalSocket.cpp 

Log.o: $(UTILS_DIR)Log/Log.cpp $(UTILS_DIR)Log/Log.h
	$(CC) $(INCLUDES) $(DEBUG) -c $(UTILS_DIR)Log/Log.cpp
	
_kbhit.o: $(UTILS_DIR)_kbhit.cpp $(UTILS_DIR)_kbhit.h
	$(CC) $(INCLUDES) $(DEBUG) -c $(UTILS_DIR)_kbhit.cpp
	
_getch.o: $(UTILS_DIR)_getch.cpp $(UTILS_DIR)_getch.h
	$(CC) $(INCLUDES) $(DEBUG) -c $(UTILS_DIR)_getch.cpp

logclean: 
	rm logs/*

clean:
	rm -rf parser ravenMasterReceiver ravenMasterSender ravenMasterCmd reformat resetSequence *.o *.dSYM
