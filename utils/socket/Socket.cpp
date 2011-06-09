#include "Socket.h"
#include <iostream>
#include <fcntl.h>
#include <stdio.h>
#include <cstring>

#ifdef __APPLE__
#define MSG_NOSIGNAL SO_NOSIGPIPE
#endif

#ifdef WIN32
#define MSG_NOSIGNAL 0
#endif

Socket::Socket() {
#ifdef WIN32
	m_sock = NULL;
#else
    m_sock = -1;
#endif
    memset(&m_addr, 0, sizeof(m_addr));
	mNonBlocking = false;
}

Socket::~Socket() {
	if(is_valid()) {
#ifdef WIN32
		closesocket(m_sock);
		WSACleanup();
#else
        ::close(m_sock);
#endif
	}
}

/****************************************************/
/*              Server Functions                    */
/****************************************************/
bool Socket::create() {
	/*if(!is_valid()) {
        return false;
	}*/
	int on = 1;
#ifdef WIN32
    if(WSAStartup(MAKEWORD(1,1), &SocketInfo) != 0) {
		std::cout << "Cannot initialize WinSock" << std::endl;
	}
#endif
	m_sock = socket(AF_INET, SOCK_STREAM, 0);
#ifdef WIN32
	if(setsockopt(m_sock, SOL_SOCKET, SO_REUSEADDR, (const char*)&on, sizeof(on)) == SOCKET_ERROR)
		return false;
#else
    if(setsockopt(m_sock, SOL_SOCKET, SO_REUSEADDR, (const char*)&on, sizeof(on)) == -1)
        return false;
#endif
	return true;
}

bool Socket::bind(const int port) {
    if(!is_valid())
        return false;
    m_addr.sin_family = AF_INET;
    m_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    m_addr.sin_port = htons(port);

    if(::bind(m_sock, (struct sockaddr*)&m_addr, sizeof(m_addr))<0) {
		std::cout << "Socket: error on bind" << std::endl;
        return false;
	}
    return true;
}

bool Socket::listen() const {
    if(!is_valid())
        return false;
    if(::listen(m_sock, MAXCONNECTIONS)<0)
        return false;
    return true;
}

bool Socket::accept(Socket& new_sock) const {
    int addr_len = sizeof(m_addr);
#ifdef WIN32
	new_sock.m_sock = ::accept(m_sock, (sockaddr*)&m_addr, &addr_len);
	if(new_sock.m_sock == INVALID_SOCKET) {
		closesocket(m_sock);
		WSACleanup();
        return false;
	}
#else
    new_sock.m_sock = ::accept(m_sock, (sockaddr*)&m_addr, (socklen_t*)&addr_len);
    if(new_sock.m_sock <= 0)
        return false;
#endif

    return true;
}

void Socket::printStatus() const {
  std::cout << m_sock << " "  << " " << mNonBlocking << std::endl;
}

/****************************************************/
/*              Client Functions                    */
/****************************************************/
bool Socket::connect(const std::string ip, const int port) {
	if(!is_valid()) {
        return false;
	}
    m_addr.sin_family = AF_INET;
    m_addr.sin_addr.s_addr = inet_addr(ip.c_str());
    m_addr.sin_port = htons(port);
#ifdef WIN32
	if(::connect(m_sock, (sockaddr*)&m_addr, sizeof(m_addr)) == SOCKET_ERROR) {
		printf("socket failed! error code: %d\n", WSAGetLastError());
		closesocket(m_sock);
		return false;
	}
#else
    if(::connect(m_sock, (sockaddr*)&m_addr, sizeof(m_addr)) != 0)
        return false;
#endif
    return true;
}

/****************************************************/
/*          Data Transmission                       */
/****************************************************/
bool Socket::send(const std::string s) const {
    if(::send(m_sock, s.c_str(), s.size(), MSG_NOSIGNAL) < 0)
        return false;
    return true;
}

int Socket::recv(std::string& s) const {
    char buffer[MAXRECV + 1];
    s = "";
    memset(buffer,0,MAXRECV+1);
    int status = ::recv(m_sock, buffer, MAXRECV,0);

    if(status == -1) {
      if(!mNonBlocking)
        std::cout << "Socket, error receiving data\n";

      return 0;
    } else if (status == 0) {
      return 0;
    } else {
      s = buffer;
      return status;
    }
}

/****************************************************/
/*          Misc.                                   */
/****************************************************/
void Socket::set_non_blocking(const bool b) {
	mNonBlocking = b;
#ifdef WIN32
	u_long argp = b ? 1 : 0;
	ioctlsocket(m_sock, FIONBIO, &argp);
#else
    int opts = fcntl(m_sock, F_GETFL);
    if(opts < 0) return;
    if(b)
        opts |= O_NONBLOCK;
    else
        opts &= ~O_NONBLOCK;

    fcntl(m_sock, F_SETFL, opts);
    std::cout << "setting non blocking to: " << b << std::endl;
#endif
}

