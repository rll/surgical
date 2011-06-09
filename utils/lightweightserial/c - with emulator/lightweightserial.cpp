#define _CRT_SECURE_NO_DEPRECATE
#include "lightweightserial.h"
#include <stdio.h>
//#ifdef WIN32
//	#include <windows.h>
//#else
	#include <unistd.h>
	#include <fcntl.h>
	#include <termios.h>
	#include <string.h>
//#endif
//#include <conio.h>
#include <iostream>
#include <fstream>
#include "_kbhit.h"
#include "_getch.h"

using namespace std;

struct LightweightSerial* LightweightSerial(const char *c_port, unsigned int c_baud)
{
#ifdef WIN32
	  COMMTIMEOUTS cto;
	    DCB dcb;
#endif

	struct LightweightSerial* serial = (struct LightweightSerial*) malloc(sizeof(struct LightweightSerial));
	serial->baud = c_baud;
	serial->is_ok = 0;
#ifdef WIN32
  // open port
  serial->hCom = CreateFileA(c_port, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
  if(serial->hCom == INVALID_HANDLE_VALUE) 
  {
    printf("CreateFile failed\n");
    return 0;
  }
  else
	  printf("[%s] opened successfully\n", c_port);

  cto.ReadIntervalTimeout = MAXDWORD;
  cto.ReadTotalTimeoutConstant = 0;
  cto.ReadTotalTimeoutMultiplier = 0;
  cto.WriteTotalTimeoutConstant = 0;
  cto.WriteTotalTimeoutMultiplier = 0;
  if (!SetCommTimeouts(serial->hCom, &cto))
	  printf("ahhhhhh error in SetCommTimeouts\n");

  if (!GetCommState(serial->hCom, &dcb))
  {
    printf("ahhhhhhhhhhh GetCommState failed\n");
    return 0;
  }

  dcb.BaudRate = serial->baud;
  dcb.ByteSize = 8;
  dcb.Parity = NOPARITY;
  dcb.StopBits = ONESTOPBIT;

  if (!SetCommState(serial->hCom, &dcb))
  {
    printf("ahhhhhhhhhhhh SetCommState failed\n");
    return 0;
  }
  else
  {
	  printf("set baud rate to %d\n", serial->baud);
	  serial->is_ok = 1;
  }
#else
	serial->fd = open(c_port, O_RDWR | O_NOCTTY);
	if (serial->fd < 0)
	{
		printf(" ahhhhhhhhh couldn't open port [%s]\n", c_port);
		return 0;
	}
	struct termios oldtio, newtio;
	if (tcgetattr(serial->fd, &oldtio) < 0)
	{
		printf("ahhhhhhh couldn't run tcgetattr()\n");
		return 0;
	}
	bzero(&newtio, sizeof(newtio));
	newtio.c_iflag = IGNPAR | INPCK;
	newtio.c_oflag = 0;
	newtio.c_cflag = CS8 | CLOCAL | CREAD;
	newtio.c_lflag = 0;
	newtio.c_cc[VTIME] = 0;
	newtio.c_cc[VMIN] = 0; // poll

	cfsetspeed(&newtio, serial->baud);
	tcflush(serial->fd, TCIOFLUSH);
	if (tcsetattr(serial->fd, TCSANOW, &newtio) < 0)
	{
		printf(" ahhhhhhhhhhh tcsetattr failed\n");
		return 0;
	}

	// flush the buffer of the serial device
	unsigned char b;
	while (read(serial,&b) > 0);
	serial->is_ok = true;
#endif

	return serial;
}

void DestroyLightweightSerial(struct LightweightSerial* serial)
{
#ifdef WIN32
  if (serial->hCom != INVALID_HANDLE_VALUE)
	  CloseHandle(serial->hCom);
#else
#endif
	free(serial);
}

int read(struct LightweightSerial* serial, unsigned char *b)
{
  //printf("in\n");
  unsigned long nread;
#ifdef WIN32
  //printf("win32\n");
  if (!ReadFile(serial->hCom, b, 1, &nread, 0))
  {
	DWORD dwError;
	switch (dwError = GetLastError()) { 
	case ERROR_HANDLE_EOF: 
		// WCode to handle the end of the file 
		// during the call to ReadFile 
		printf("reached EOF\r\n");
		break;
	case ERROR_IO_PENDING: 
		printf("reached IO_PENDING\r\n");
		break;
	case 995 :
		printf("serial port typical error 995 ... \r\n");
		break;
	default : ;
	}
	printf("ahhhhhhh readfile failed in LightweightSerial\n");
	return 0;
  }
  if (nread == 0)
		return 0;
  else
		return 1;
#else
	nread = ::read(serial->fd,b,1);
	if (nread < 0)
	{
		printf("ahhhhhh read returned <0\n");
		return false;
	}
	if (nread == 0)
		return false;
	else	
	{
		//printf("read %d\n", *b);
		return true;
	}
#endif
}

int read_block(struct LightweightSerial* serial, unsigned char *block, unsigned max_read_len)
{
	unsigned long nread;
#ifdef WIN32
	if (!ReadFile(serial->hCom, block, (DWORD)max_read_len, &nread, 0))
	{
		printf("ahhhhhhh readfile failed in LightweightSerial\n");
		return 0;
	}

	return nread;
#else
	nread = ::read(serial->fd,block,(size_t)max_read_len);
	if (nread < 0)
	{
		printf("ahhhhhh read returned <0\n");
		return 0;
	}
	return nread;
#endif
}

void write(struct LightweightSerial* serial, unsigned char b)
{
#ifdef WIN32
	static DWORD nwritten = 0;
	if (serial->hCom != INVALID_HANDLE_VALUE && !WriteFile(serial->hCom, &b, 1, &nwritten, 0))
		printf(" ahhhhhhhhhhh writefile failed\n");
#else
	if (serial->fd >= 0 && ::write(serial->fd, &b, 1) < 0)
		printf(" ahhhhhhhhhh write failed\n");
#endif
}

void write_block(struct LightweightSerial* serial, unsigned char *block, unsigned short block_len)
{
#ifdef WIN32
	static DWORD nwritten = 0;
	if (serial->hCom != INVALID_HANDLE_VALUE && !WriteFile(serial->hCom, block, block_len, &nwritten, 0))
		printf(" ahhhhhhhhhhh writefile failed in write_block\n");	
#else
	if (serial->fd >= 0 && ::write(serial->fd, block, block_len) < 0)
		printf(" write_block failed\n");
	else
		tcflush(serial->fd, TCOFLUSH);
#endif
}


void EmulateSerialTerminal(struct LightweightSerial* serial, int tofile_flag)
{
	FILE *dump;// = fopen("dumpserial.dat", "wb");
	FILE *dump_ascii;// = fopen("C:\\documents and Settings\\pieter Abbeel\\desktop\\dumpascii.dat","w");
	int print_serial_stream = 1;
	int num_bytes_received = 0;
	int max_read_len = 256;
	unsigned char block[257]; block[256] = 0;

	printf("Emulating Terminal.\n");

	while (1)
	{
		unsigned char b;
		unsigned num_read;
		while (num_read = read_block(serial, block, max_read_len)) 
		{
			if (print_serial_stream)
				if(tofile_flag)
					fwrite(block, 1, num_read, dump);
				else {
					block[num_read] = 0;
					printf("%s", block);
				}
			else
			{
				num_bytes_received++;
//				printf("received %d bytes (%d)\n", num_bytes_received, b);
				if (num_bytes_received % 50000 == 0)
					printf("received %d bytes\n", num_bytes_received);
				fwrite(&b, 1, 1, dump);
			}
		}

		if (_kbhit())
		{
			char ch = _getchar();
			if (ch == 'q')
				break;
			else if (ch == 'd') // dump card (binary data...)
				print_serial_stream = 0;
			else
				print_serial_stream = 1;
			write(serial, ch);
		}
		//Sleep(100);
	}

	fclose(dump);

}
