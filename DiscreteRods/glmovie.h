#ifndef _glmovie_
#define _glmovie_

/****	Simple code to capture OpenGL frames and save to
	files while program is running, so you can make
	a movie from it.
	
	Usage:
	1. NewMovie
	2. Change any of the file pattern, frame rate, rectangle
	   captured parameters if you want/need.
	3. In your display code, call
		MovieFrame(&my-movie);
	   just before swapping buffers. If the movie is 'live' and
	   enough time has elapsed, it will capture the current frame
	   to a new file. Internally MovieFrame does a glFinish(), so
	   you don't need to.
	4. Use MovieLive to turn capture on or off. (It starts off)
	
	The output is stored as numbered uncompressed PPM files. Make
	a movie with something like
		convert -flip /tmp/u9011925*.ppm capture.mpg
	
	The output files are upside down because the native OpenGL
	image order is bottom to top. Correcting this would take
	extra time, and it's easily fixed by ImageMagick anyway.
	
	Internally the file IO is done by a background thread, so on
	a fast machine you capture at a reasonable rate without the
	main program slowing down. But if frames are being dropped,
	you will have to reduce the window size.
	
	Written by Hugh Fisher.
	Released under BSD/MIT license
							****/

#include <pthread.h>
#include <semaphore.h>

#include <GL/gl.h>

#define TRUE true
#define FALSE false

typedef struct {
	int	enabled;
	char *	filePath;
	int	fps;
	int	frameNumber;
	GLenum	buffer;
	GLuint	x, y, w, h;
	/* Internal timing for deciding when to capture */
	int	firstFrame;
	double	timeBase, nextFrameTime;
	/* The IO thread */
	GLbyte *	pixels;
	pthread_t	tid;
	pthread_mutex_t access;
	sem_t		ready;
	} GLMovieRec;

extern void NewMovie (GLMovieRec * movie);
extern void FreeMovie (GLMovieRec * movie);

/* Path needs %d format somewhere, or you will just overwrite the
   same file each frame! Default is /tmp/your-username%04d.ppm   */
extern void MoviePath (GLMovieRec * movie, char * newPath);

/* Default is 24. 0 means capture every frame */
extern void MovieFrameRate (GLMovieRec * movie, int newRate);

/* Default is back buffer, change for stereo or single buffered */
extern void MovieReadBuffer (GLMovieRec * movie, GLenum newBuffer);

/* Default is entire viewport at time of first frame. Probably
   not a good idea to resize the window once started. */
extern void MovieRect (GLMovieRec * movie, int x, int y, int w, int h);

/* TRUE means capture frames */
extern void MovieLive (GLMovieRec * movie, int state);

extern void MovieFrame (GLMovieRec * movie);

#endif
