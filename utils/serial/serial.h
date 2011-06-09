#ifndef SERIAL_H
#define SERIAL_H

#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

class Serial
{
  public:
    Serial(const char* port, int baud);
    ~Serial();

    bool read(unsigned char *b);
    bool read_block(unsigned char* block, unsigned length);
    unsigned long read_non_blocking(unsigned char* block, unsigned length);

    void flush_input_buffer();

    void write(unsigned char b);
    void write_block(unsigned char* block, unsigned short length);

    int m_baud;
    int m_fd;


};

#endif

