/*
* Socket.h
	* Simple wrapper for socket functions
	* Based on Rob Tougher's code at http://tldp.org/LDP/LG/issue74/tougher.html
*/

#ifndef Socket_H
#define Socket_H

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <string>
#include <string.h>

const int MAXTCPCONNECTIONS = 15;
const int MAXTCPRECV = 1024;

class Socket {
public:
	Socket();
	~Socket();

	// server init
	bool create();
	bool bind(const int port);
	bool listen() const;
	bool accept(Socket&) const;

	// client init
	bool connect(const std::string ip, const int port);

	// data transmission
	bool send(const std::string) const;
	int recv(std::string&) const;

	// misc
	void set_non_blocking(const bool);
	bool is_valid() const { return m_sock != -1; };
	
	void set_timeout(const unsigned int microseconds);

private:
	int m_sock;
	sockaddr_in m_addr;
	unsigned int m_timeout;	// in microseconds
	bool mNonBlocking;
};

#endif
