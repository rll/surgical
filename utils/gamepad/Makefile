TARGETS = intercom

CC		= g++ 
FLAGS = -Wall
DEBUG = -ggdb
#INCLUDE_DIRS	= -I $(SERIAL_DIR)
LINK_OPTS = -lpthread

SOURCE = ./WdThreadsLin.cpp ./gamepad.cpp ./test_gamepad.cpp

OUTPUT = gamepad

RUN =./$(OUTPUT)

all: $(TARGETS)


intercom: $(SOURCE)
	@echo "BUILDING TAKE-OVER CODE"
	$(CC) $(FLAGS) $(INCLUDE_DIRS) $(LINK_OPTS) $(SOURCE) -o $(OUTPUT)

debug:
	@echo "BUILDING CODE DEBUG"
	$(CC) $(FLAGS) $(DEBUG) $(INCLUDE_DIRS) $(LINK_OPTS) $(SOURCE) -o $(OUTPUT)

clean:
	rm -rf $(OUTPUT) *.~* *~
