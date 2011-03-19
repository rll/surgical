// miranova.h
// 4 Channel Counter (For quadrature decoding)
#ifndef __MIRANOVA_H__
#define __MIRANOVA_H__

#include "sensor.h"
#include "shared.h"
#include "LowPassFilter.h"


/** Low level driver for initializing Miranova Counter board.

    <p>Sets channel's counter to <i>ncount</i>, and <i>quadmode</i>.
*/
void miranova_init_channel(int channel, long ncount, int quadmode);

/** Low level driver for reading Miranova Counter board.

    <p>Reads and returns a 24-bit value form channel on the Miranova
    Counter board.  Sets <i>status</i> to the current status of the board.

    <p>Check board's documentation for more information.
*/
long miranova_read(int channel, int &status);


/** Miranova Sensor class.

    <p>This class implements the interface to the Miranova Encoder
    Counter board.
*/
class MiranovaSensor : public Sensor {
public:
    int slave_num;
    
    MiranovaSensor(int channel, double ratio, double offset, int sign, int slave);
    ~MiranovaSensor(void);
    
    /** Init where the slave is to be 0 */
    void Init(void);
    
    /** Init where the slave is now to be the value given in sensor's units */
    void Init(double);
    
    /** Deprecated */
    void Adjust(double value);
    
    /** Deprecated */
    int IsReady(void);
    
    /** Where the slave is now in its units */
    double Input(void);
    
    /** Returns string "Miranova"; Deprecated */
    const char *GetName(void);
};

#endif
