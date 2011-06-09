/*
* Socket.h
*	Simple wrapper for socket functions
*	Based on Rob Tougher's code at http://tldp.org/LDP/LG/issue74/tougher.html
*/

#ifndef Socket_H
#define Socket_H

#ifdef WIN32
#include <winsock.h>
#include <windows.h>
#else
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <unistd.h>
#endif

#include <string>

const int MAXCONNECTIONS = 15;
const int MAXRECV = 500;

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
        void printStatus() const;
	bool is_valid() const {
#ifdef WIN32
		return m_sock != NULL;
#else
		return m_sock != -1;
#endif
	};

private:
#ifdef WIN32
	SOCKET m_sock;
	WSADATA SocketInfo;
#else
	int m_sock;
#endif
	struct sockaddr_in m_addr;
	bool mNonBlocking;
};

#endif
