
CC		= g++ 
FLAGS = -Wall
DEBUG = -ggdb
#INCLUDE_DIRS	= -I ../ 
#LINK_OPTS	= ../

SRC_SERVER = ./Socket.cpp ./example_server.cpp
SRC_CLIENT = ./Socket.cpp ./example_client.cpp

OUTPUT_SERVER = example_server
OUTPUT_CLIENT = example_client

TARGETS = server client

all: $(TARGETS)

server: $(SRC_SERVER)
	@echo "BUILDING SERVER CODE"
	$(CC) $(FLAGS) $(SRC_SERVER) -o $(OUTPUT_SERVER)

client: $(SRC_CLIENT)
	@echo "BUILDING CLIENT CODE"
	$(CC) $(FLAGS) $(SRC_CLIENT) -o $(OUTPUT_CLIENT)

debug:
	@echo "BUILDING CODE DEBUG"
	$(CC) $(FLAGS) $(DEBUG) $(LINK_OPTS) $(SOURCE) -o $(OUTPUT)

clean:
	rm -rf *.o* $(OUTPUT_SERVER) $(OUTPUT_CLIENT) *.~* *~
