#include "lightweightserial.h"
#include <stdio.h>
#ifdef WIN32
	#include <windows.h>
#else
	#include <unistd.h>
	#include <fcntl.h>
	#include <termios.h>
	#include <string.h>
#endif

LightweightSerial::LightweightSerial(const char *c_port, int c_baud) :
	baud(c_baud),
	is_ok(false)
{
#ifdef WIN32
  // open port
  hCom = CreateFileA((LPCSTR)c_port, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
  if(hCom == INVALID_HANDLE_VALUE) 
  {
    printf("CreateFile failed\n");
    return;
  }
  else
	  printf("[%s] opened successfully\n", c_port);

  COMMTIMEOUTS cto;
  cto.ReadIntervalTimeout = MAXDWORD;
  cto.ReadTotalTimeoutConstant = 0;
  cto.ReadTotalTimeoutMultiplier = 0;
  cto.WriteTotalTimeoutConstant = 0;
  cto.WriteTotalTimeoutMultiplier = 0;
  if (!SetCommTimeouts(hCom, &cto))
	  printf("ahhhhhh error in SetCommTimeouts\n");

  DCB dcb;
  if (!GetCommState(hCom, &dcb))
  {
    printf("ahhhhhhhhhhh GetCommState failed\n");
    return;
  }

  dcb.BaudRate = baud;
  dcb.ByteSize = 8;
  dcb.Parity = NOPARITY;
  dcb.StopBits = ONESTOPBIT;

  if (!SetCommState(hCom, &dcb))
  {
    printf("ahhhhhhhhhhhh SetCommState failed\n");
    return;
  }
  else
  {
	  printf("set baud rate to %d\n", baud);
	  is_ok = true;
  }
#else

	fd = open(c_port, O_RDWR | O_NOCTTY);
	if (fd < 0)
	{
		printf(" ahhhhhhhhh couldn't open port [%s]\n", c_port);
		return;
	}
	struct termios oldtio, newtio;
	if (tcgetattr(fd, &oldtio) < 0)
	{
		printf("ahhhhhhh couldn't run tcgetattr()\n");
		return;
	}
	bzero(&newtio, sizeof(newtio));
	newtio.c_iflag = IGNPAR | INPCK;
	newtio.c_oflag = 0;
	newtio.c_cflag = CS8 | CLOCAL | CREAD;
	newtio.c_lflag = 0;
	newtio.c_cc[VTIME] = 0;
	newtio.c_cc[VMIN] = 0; // poll
	cfsetspeed(&newtio, baud);
	tcflush(fd, TCIOFLUSH);
	if (tcsetattr(fd, TCSANOW, &newtio) < 0)
	{
		printf(" ahhhhhhhhhhh tcsetattr failed\n");
		return;
	}

	// flush the buffer of the serial device
	unsigned char b;
	while (read(&b) > 0);
	is_ok = true;
#endif
}

LightweightSerial::~LightweightSerial()
{
#ifdef WIN32
  if (hCom != INVALID_HANDLE_VALUE)
	  CloseHandle(hCom);
#else
#endif
}

bool LightweightSerial::read(unsigned char *b)
{
  unsigned long nread;
#ifdef WIN32
  if (!ReadFile(hCom, b, 1, &nread, 0))
  {
	printf("ahhhhhhh readfile failed in LightweightSerial\n");
    return false;
  }
  if (nread == 0)
		return false;
  else
		return true;
#else
	nread = ::read(fd,b,1);
	if (nread < 0)
	{
		printf("ahhhhhh read returned <0\n");
		return false;
	}
	if (nread == 0)
		return false;
	else	
	{
	//	printf("read %d\n", *b);
		return true;
	}
#endif
}

int LightweightSerial::read_block(unsigned char *block, unsigned max_read_len)
{
	unsigned long nread;
#ifdef WIN32
	if (!ReadFile(hCom, block, (DWORD)max_read_len, &nread, 0))
	{
		printf("ahhhhhhh readfile failed in LightweightSerial\n");
		return 0;
	}
	return nread;
#else
	nread = ::read(fd,block,(size_t)max_read_len);
	if (nread < 0)
	{
		printf("ahhhhhh read returned <0\n");
		return 0;
	}
	return nread;
#endif
}

void LightweightSerial::write(unsigned char b)
{
#ifdef WIN32
	static DWORD nwritten = 0;
	if (hCom != INVALID_HANDLE_VALUE && !WriteFile(hCom, &b, 1, &nwritten, 0))
		printf(" ahhhhhhhhhhh writefile failed\n");
#else
	if (fd >= 0 && ::write(fd, &b, 1) < 0)
		printf(" ahhhhhhhhhh write failed\n");
#endif
}

void LightweightSerial::write_block(unsigned char *block, unsigned short block_len)
{
#ifdef WIN32
	static DWORD nwritten = 0;
	if (hCom != INVALID_HANDLE_VALUE && !WriteFile(hCom, block, block_len, &nwritten, 0))
		printf(" ahhhhhhhhhhh writefile failed in write_block\n");	
#else
	if (fd >= 0 && ::write(fd, block, block_len) < 0)
		printf(" write_block failed\n");
	else
		tcflush(fd, TCOFLUSH);
#endif
}

