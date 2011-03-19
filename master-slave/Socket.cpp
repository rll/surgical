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
	m_timeout = 20000;
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
		MessageBox(NULL, "Cannot initialize WinSock", "WSAStartup", MB_OK);
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
    if(::listen(m_sock, MAXTCPCONNECTIONS)<0)
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
    char buffer[MAXTCPRECV + 1];
	struct timeval tv;
	fd_set fdset;
	int rc, nread;
	
    //s = "";
    //memset(buffer,0,MAXTCPRECV+1);
    //int status = ::recv(m_sock, buffer, MAXTCPRECV,0);
	
	FD_ZERO(&fdset);
	FD_SET(m_sock, &fdset);
	
	tv.tv_sec = 0;
	tv.tv_usec = m_timeout;
	
	rc = select(m_sock + 1, &fdset, (fd_set *) 0, (fd_set *) 0, &tv);
	
	if(FD_ISSET(m_sock, &fdset)) {
		nread = ::recv(m_sock, buffer, MAXTCPRECV, 0);
		if(nread < 0) {
			//std::cout << "fail" << std::endl;
			return -1;
		} else if(nread == 0) {
			//std::cout << "eof" << std::endl;
			return 0;
		}
		//std::cout << "select failure" << std::endl;
		//std::cout << "received : " << buffer << std::endl;
		s = std::string(buffer);
		return nread;
	} else {
		//std::cout << "no data" << std::endl;
		return 0;
	}
	/*
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
	*/
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
#endif
}

void Socket::set_timeout(const unsigned int microseconds) {
	m_timeout = microseconds;
	return;
}

