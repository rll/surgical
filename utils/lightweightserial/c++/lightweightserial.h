#ifndef LIGHTWEIGHTSERIAL_H
#define LIGHTWEIGHTSERIAL_H

class LightweightSerial
{
	public:
		LightweightSerial(const char *c_port, int c_baud);
		LightweightSerial(){};
		~LightweightSerial();

		bool read(unsigned char *b);
		int read_block(unsigned char *block, unsigned max_read_len);

		void write(unsigned char b);
		void write_block(unsigned char *block, unsigned short block_len);

		int baud;
		void *hCom;
		int fd;
		bool is_ok;
};

#endif

