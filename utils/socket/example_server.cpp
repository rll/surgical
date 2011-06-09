#include "Socket.h"

#include <iostream>
#include <signal.h>

#define PORT 3000

volatile bool master_run = true;

void quit_prog(int sig) {
  master_run = false;
  signal(SIGINT, SIG_DFL);
}


int main(void){
  Socket socket, new_socket;

	std::string s;

  if(!socket.create()) {
    std::cout << "Server: error on create\n";
  }
  if(!socket.bind(PORT)) {
    std::cout << "Server: error on bind\n";
  }
  if(!socket.listen()) {
    std::cout << "Server: error on listen\n";
  }

	while(master_run){
		std::cout << "Server online and waiting for connections\n";
		if(!socket.accept(new_socket)) {
			//std::cout << "ServerSocket, error on accept\n"; // ?? TODO this line never gets run, does it?
			continue;
		}else{
		  std::cout << "server got a conection" << std::endl;   
		}

		int run = 1;
		while(run && master_run){
			int success = new_socket.recv(s);
		  if(success) {
		    std::cout << "received: " << s << std::endl;
		  }else{
			  run = 0;			
			}
#ifdef WIN32
			Sleep(50);
#else
			usleep(50000); //50ms
#endif
		}

	}

	return 0;
}
