#ifndef LIGHTWEIGHTSERIAL_H
#define LIGHTWEIGHTSERIAL_H


struct LightweightSerial
{
		int baud;
		void *hCom;
		int fd;
		int is_ok;
};

struct LightweightSerial* LightweightSerial(const char *c_port, unsigned int c_baud);
void DestroyLightweightSerial(struct LightweightSerial* serial);

int read(struct LightweightSerial* serial, unsigned char *b);
int read_block(struct LightweightSerial* serial, unsigned char *block, unsigned max_read_len);

void write(struct LightweightSerial* serial, unsigned char b);
void write_block(struct LightweightSerial* serial, unsigned char *block, unsigned short block_len);

void EmulateSerialTerminal(struct LightweightSerial* serial, int tofile_flag);

#endif

