#include "Socket.h"

#include <iostream>
#include <sstream>
#include <stdlib.h>


#define PORT 3000
#define IP "128.32.33.215"

int main(void){

	Socket socket;
	if(!socket.create()){
		std::cout << "Error creating socket!" << std::endl;
	}else{
		std::cout << "Success: Client created socket" << std::endl;
	}
	if(socket.connect(IP, PORT)){
		std::cout << "Success: Client connected to server on ip: " << IP << "  port: " << PORT << std::endl;
	}else{
		std::cout << "Client: error on connect to ip: " << IP << "  port: " << PORT << std::endl;
		exit(-1);
	}

	while(1){
		std::stringstream data;
		data 	<< "<This is the data sent>";
		std::cout << "sending: " << data.str() << std::endl;

		if(!socket.send(data.str())){
			std::cout << "Client: error on send\n";
			return -1;
		}	
#ifdef WIN32
		Sleep(20);
#else
		usleep(200000);
#endif
	}

	return 0;
}
