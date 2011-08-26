MACHINE= $(shell uname -s)
HOSTNAME=-D$(shell uname -n | sed 's/-/_/g' | sed 's/\./_/g')

ifeq ($(MACHINE),Darwin)
    LIBS = -L/usr/lib/ -lglut -lgle
    COMPILEFLAG=-DMAC -lpthread
    LDFLAG=$(COMPILEFLAG) -L"/System/Library/Frameworks/OpenGL.framework/Libraries" -L"/System/Library/Frameworks/GLUT.framework" -framework GLUT -framework OpenGL -lGL -lGLU -L/usr/X11R6/lib
    INCLUDES = -I/usr/local/include/ -I/usr/include/ -I $(UTILS_DIR) -I/opt/local/include/eigen2/ -I/opt/local/include/
else
    LIBS = -L/usr/lib/ -lglut -lgle
    INCLUDES = -I/usr/local/include/ -I/usr/include/ -I $(UTILS_DIR) -I/usr/include/eigen2
    COMPILEFLAG= -lpthread
    LDFLAG=
endif


UTILS_DIR = utils/
OPT_COMP = -DEIGEN_NO_DEBUG -O3
CC = g++ $(OPT_COMP) $(COMPILEFLAG) 
LD = g++ $(OPT_COMP) $(LDFLAG)  
DEBUG = -ggdb

target: parser visualizer

ravenMasterReceiver: ravenMasterReceiver.cpp ravenMasterReceiver.h PracticalSocket.o Log.o _kbhit.o _getch.o
	$(LD) $(INCLUDES) $(DEBUG) -o ravenMasterReceiver ravenMasterReceiver.cpp PracticalSocket.o Log.o _kbhit.o _getch.o $(LIBS)

ravenMasterSender: ravenMasterSender.cpp ravenMasterSender.h PracticalSocket.o Log.o sequence.o
	$(LD) $(INCLUDES) $(DEBUG) -o ravenMasterSender ravenMasterSender.cpp PracticalSocket.o Log.o sequence.o $(LIBS)

ravenMasterCmd: ravenMasterCmd.cpp ravenMasterCmd.h sequence.o PracticalSocket.o Log.o
	$(LD) $(INCLUDES) $(DEBUG) -o ravenMasterCmd ravenMasterCmd.cpp PracticalSocket.o Log.o $(LIBS)
	
reformat: reformat.cpp
	$(LD) $(INCLUDES) $(DEBUG) -o reformat reformat.cpp $(LIBS)

visualizer: visualizer.cpp
	$(LD) $(INCLUDES) $(DEBUG) -o visualizer visualizer.cpp $(LIBS)

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

clean:
	rm -rf parser ravenMasterReceiver ravenMasterSender ravenMasterCmd reformat resetSequence visualizer *.o *.dSYM
