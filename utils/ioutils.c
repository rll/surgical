#include "ioutils.h"
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/time.h>
#include <errno.h>

/**
 * Attempts to read a single byte before a timeout expires.
 * Timeout is in 1/10ths of a second.  If a byte is read before
 * the timeout, 1 is returned.  If the timeout expires before
 * data is available, 0 is returned.  It returns an error < 0
 * otherwise.
 */
int read_byte_timed(int file, char* buf, int timeout) {
  struct termios ts, newts;
  int    result = 0;

  tcgetattr(file, &ts);
  newts = ts;

  if (timeout > 0) {
    newts.c_cc[VTIME] = timeout;
    newts.c_cc[VMIN] = 0;
    tcsetattr(file, TCSANOW, &newts);

    result = read(file, buf, 1);

    tcsetattr(file, TCSANOW, &ts);
  }

  return result;
}

/**
 * Reads at least 'min' bytes before returning, unless an error
 * occurs.  'max' is the maximum number of bytes to read.  This
 * returns the number of bytes read or a negative number on error;
 * essentially like read().
 */
int read_at_least(int file, void* buf, int max, int min) {
  struct termios ts, newts;
  int    result = 0;
  int    read_err = 0;

  tcgetattr(file, &ts);
  newts = ts;

  if (max > 0) {
    
    newts.c_cc[VTIME] = 0;
    if (min < 0) min = 0; //sanity check
    newts.c_cc[VMIN] = min;
    tcsetattr(file, TCSANOW, &newts);

    result = read(file, buf, max);
    if (result) {
      read_err = errno;
    }

    tcsetattr(file, TCSANOW, &ts);
  }

  errno = read_err;
  return result;
}

int read_till_end(int file, void * buff, int max, char endchar){
  struct termios ts,newts;
  int result = -1;
  int read_err=0;

  fd_set readset;
  FD_ZERO(&readset);
  FD_SET(file, &readset);

  tcgetattr(file,&ts);
  newts = ts;
  if(max > 0){
    newts.c_lflag |= ICANON;
    newts.c_cc[VTIME] = 0;
    newts.c_cc[VMIN] = 1;

    
    select(file+1, &readset, 0, 0, 0);

    // newts.c_cc[VSTOP] = endchar;
    tcsetattr(file,TCSANOW,&newts);
    result = read(file,buff,max);
    if(result){
      read_err = errno;
    }

    //    tcsetattr(file,TCSANOW,&ts);
  }
  errno = read_err;
  return result;
}

/**
 * Returns non-zero if input is available on STDIN.
 */
int key_hit(void) {
  fd_set read_set;
  struct timeval timeout;

  FD_ZERO(&read_set);
  FD_SET(STDIN_FILENO, &read_set);

  // set zero timeout -- poll only.
  timeout.tv_sec = 0;
  timeout.tv_usec = 0;

  return (select(1, &read_set, NULL, NULL, &timeout) > 0);
}

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
		      struct termios* oldstate) {
  struct termios term, prevTerm;
  
  if (isatty(fd)) {
    //fill structure with current settings
    tcgetattr(fd, &term); 
    prevTerm = term;
    
    // copy out current settings
    if (oldstate != 0)
      *oldstate = prevTerm;
    
    //turn off ICANON, etc.
    term.c_lflag &= ~(ICANON | ECHO | ISIG | IEXTEN);
    
    //no special input/output processing
    term.c_iflag = 0;
    term.c_oflag = 0;
    
    //8 bit chars + enable receiver
    term.c_cflag &= ~CSIZE;
    term.c_cflag |= CS8 | CREAD | HUPCL;
    
    //set input/output baud rate
    cfsetispeed(&term, baudrate);
    cfsetospeed(&term, baudrate);
    
    // set read minimum and timeout flags
    if (vmin != 255)
      term.c_cc[VMIN] = vmin;
    if (vtime != 255)
      term.c_cc[VTIME] = vtime;
    
    //set terminal attributes
    if (tcsetattr(fd, TCSANOW, &term) == -1)
      return -1;
    
    // switch back to blocking mode
    if (set_file_nonblocking(fd, nonblocking) != 0) {
      // try to switch back.
      tcsetattr(fd, TCSANOW, &prevTerm);
      return -2;
    }
  }

  return 0; //success!
}

/**
 * Sets the given file descriptor into blocking or non-blocking
 * mode.  If 'nonblocking' is 1 this will set the O_NONBLOCK flag
 * for the descriptor, otherwise it will clear it explicitly. Returns
 * 0 on success or a result from fcntl()
 */
int set_file_nonblocking(int fd, int nonblocking) {
  int status;
  // need to clear O_NONBLOCK to get out of non-blocking mode
  if ((status = fcntl(fd, F_GETFL, 0)) != -1) {
    // change non-block bit flag
    if (nonblocking != 0)
      status |= O_NONBLOCK; // set nonblock
    else
      status &= ~O_NONBLOCK; // clear nonblock

    // re-set flags
    if (fcntl(fd, F_SETFL, status) == -1) {
      return -1;
    }
  } else {
    return -1;
  }

  // success!
  return 0;
}
