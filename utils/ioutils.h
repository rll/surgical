#ifndef IO_UTILS_H
#define IO_UTILS_H

#include <termios.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * These emulate the behavior of some QNX IO functions in the situations
 * that existed in previous versions of the code.
 */

/**
 * Attempts to read a single byte before a timeout expires.
 * Timeout is in 1/10ths of a second.  If a byte is read before
 * the timeout, 1 is returned.  If the timeout expires before
 * data is available, 0 is returned.  It returns an error < 0
 * otherwise.
 */
int read_byte_timed(int file, char* byte, int timeout);

/**
 * Reads at least 'min' bytes before returning, unless an error
 * occurs.  'max' is the maximum number of bytes to read.  This
 * returns the number of bytes read or a negative number on error;
 * essentially like read().
 */
int read_at_least(int file, void* buff, int max, int min);

  /**
   * Reads until the endchar is read or until 'max' number of bytes
   * have been read. The function returns the number of bytes read or 
   * a negative number on error; 
   */

int read_till_end(int file, void * buf, int max, char endchar);

/**
 * Returns non-zero if input available on STDIN.  Similar to
 * tcischars() [QNX] or kbhit() [Win32].
 */
int key_hit(void);

/* Other utilities */

/**
 * Initializes the terminal settings for the descriptor 'fd'
 * to put it into raw mode.  'baudrate' is one of the Bxxxx
 * constants from termios.h.  'nonblocking' specifies whether
 * the descriptor should be put into nonblocking or blocking
 * mode.  vmin and vtime are the values of the VMIN and VTIME
 * elements of the c_cc[] array; if the parameter is 255, the
 * current settings are left untouched for that value. 'oldstate'
 * is an optional pointer to a termios structure which will 
 * store the previous settings.  This function returns 0 on 
 * success or non-zero on any error.
 */
int init_raw_terminal(int fd, speed_t baudrate, int nonblocking,
		      unsigned char vmin, unsigned char vtime,
		      struct termios* oldstate);

/**
 * Sets the given file descriptor into blocking or non-blocking
 * mode.  If 'nonblocking' is 1 this will set the O_NONBLOCK flag
 * for the descriptor, otherwise it will clear it explicitly. Returns
 * 0 on success or a result from fcntl()
 */
int set_file_nonblocking(int fd, int nonblocking);

#ifdef __cplusplus
} // extern "C"
#endif


#endif /* IO_UTILS_H */
