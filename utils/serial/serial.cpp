#include "serial.h"
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <errno.h>
#include <iostream>
#include <poll.h>

using namespace std;

Serial::Serial(const char* port, int baud) :
  m_baud(baud)
{

  m_fd = open(port, O_RDWR | O_NOCTTY);
  if (m_fd < 0)
    {
      cerr << "Could not open port: " << port << endl;
      return;
    }

  struct termios oldtio, newtio;
  if (tcgetattr(m_fd, &oldtio) < 0)
    {
      cerr << "Error running tcgetattr" << endl;
      return;
    }

  memset(&newtio, 0, sizeof(newtio));
  newtio.c_iflag = IGNPAR | INPCK;
  newtio.c_oflag = 0;
  newtio.c_cflag = CS8 | CLOCAL | CREAD;
  newtio.c_lflag = 0;
  newtio.c_cc[VTIME] = 0;
  newtio.c_cc[VMIN] = 0; // poll

  cfsetspeed(&newtio, baud);
  tcflush(m_fd, TCIOFLUSH);

  if (tcsetattr(m_fd, TCSANOW, &newtio) < 0)
    {
      cerr << "Error running tcsetattr" << endl;
      return;
    }

  // flush the buffer of the serial device
  unsigned char b;
  while (this->read(&b));


}

Serial::~Serial()
{
}

bool Serial::read(unsigned char *b)
{
  unsigned long nread;
  nread = ::read(m_fd,b,1);
  if (nread < 0)
    {
      printf("ahhhhhh read returned <0\n");
      return false;
    }

  return (nread != 0);

}

bool Serial::read_block(unsigned char* block, unsigned length)
{

  unsigned long nread, total_read;

  fd_set fds;
  FD_ZERO(&fds);
  FD_SET(m_fd, &fds);
  struct timeval tv;

  pollfd pfd;

  pfd.fd = m_fd;
  pfd.events = POLLIN;

  if(poll(&pfd, 1, 0) <= 0){
    //cerr << "error: " << errno << endl;
    return false;
  }
  if(!((pfd.revents & POLLIN) == POLLIN))
    {
      //cerr << "returning false;" << endl;
      return false;
    }

  total_read = 0;

  while(total_read < length){
    nread = ::read(m_fd,block+sizeof(unsigned char) * total_read,(size_t) length - total_read);

    if (nread < 0)
      {
        cerr << "errno is: " << errno << endl;
      } else {
      total_read += nread;
    }

  }

  return true;

}

unsigned long Serial::read_non_blocking(unsigned char* block, unsigned length)
{

  unsigned long nread;

  fd_set fds;
  FD_ZERO(&fds);
  FD_SET(m_fd, &fds);

  pollfd pfd;

  pfd.fd = m_fd;
  pfd.events = POLLIN;

  if(poll(&pfd, 1, 0) <= 0){
    return 0;
  }
  if(!((pfd.revents & POLLIN) == POLLIN)) {
    return 0;
  }

  nread = ::read(m_fd,block,(size_t) length);
  return nread;

}

void Serial::flush_input_buffer()
{
  tcflush(m_fd, TCIFLUSH);
}

void Serial::write(unsigned char b)
{
  if (m_fd >= 0 && ::write(m_fd, &b, 1) < 0)
    printf(" ahhhhhhhhhh write failed\n");
}

void Serial::write_block(unsigned char *block, unsigned short block_len)
{
  if (m_fd >= 0 && ::write(m_fd, block, block_len) < 0)
    printf(" write_block failed\n");
  else
    tcflush(m_fd, TCOFLUSH);
}
