CC = g++ -Wall

all: calchog

rectifyIm: rectifyIm.cpp
	$(CC) -o rectifyIm rectifyIm.cpp -lhighgui -lcv

calchog: calcHOGdescriptor.cpp utils.o
	$(CC) -o calchog calcHOGdescriptor.cpp utils.o -lhighgui -lcv -lcvaux

locateMid: locateMid.cpp utils.o
	$(CC) -o locateMid locateMid.cpp utils.o -lhighgui -lcv 

scratch: scratch.cpp utils.o
	$(CC) -o scratch scratch.cpp utils.o -lcv -lhighgui

scratchProc: scratchProc.cpp
	$(CC) -o scratchProc scratchProc.cpp -lcv -lhighgui

getT: getT.cpp
	$(CC) -o getT getT.cpp -lcv -lhighgui

utils.o: utils.cpp utils.h
	$(CC) -c utils.cpp -lcv
