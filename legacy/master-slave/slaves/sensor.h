// sensor.h

#ifndef SENSOR_H
#define SENSOR_H

/** Virtual Base Sensor class.

    <p>This class provides an interface to a hardware input device,
    such as a encoder counter.  Several layers are usually involved in
    this mechanism (bottom up):  physical device(encoder), counter or ADC
    board, and software driver for the card.  Each card has several
    channels. A channel number, passed to the driver and on to the card
    itself, controls which input is being adressed.  Children of this
    class will implement, or use exisitng software drivers for each input
    card.  Each instance of each child class (object) will have a unique
    channel, though which that object will access the same output board.
*/
class Sensor {
public:
  
  /** Last value input from the channel.

      <p>The value of this variable should be set by method <i>Input()</i>.
  */
  double currentInput;

  /** Channel on an input board this object uses.

      <p>The value of this variable is set in the constructor, and
      should be used in methods <i>Init()</i>, <i>Adjust()</i>,
      <i>IsReady()</i>, and <i>Input()</i>.
  */
  int channel;

  /** Converts raw input to the desired internal units (counts per rad or mm).

      <p>The value of this variable is set in the constructor, and
      should be used in methods <i>Adjust()</i> and <i>Input()</i>.
  */
  double ratio;

  /** Offsets the input value.

      <p>The value of this variable is set in the constructor, and
      should be used in methods <i>Adjust()</i> and <i>Input()</i>.
  */
  double offset;

  /** Reverses the sign of the input value.
      
      <p>The value of this variable is set in the constructor, and
      should be used in methods <i>Adjust()</i> and <i>Input()</i>.
  */
  float sign;
  
  /** Constructor for Sensor class.
      
      <p>Set channel number, ratio, offset, and sign(optional).
  */
  Sensor(int c, double r, double o, int s = 1);

  /** Destractor for Sensor class.
  */
  virtual ~Sensor(void);

  /** Initializes this channel.
  */
  virtual void Init(void) = 0;
  virtual void Init(double) = 0;

//  /** Adjusts the hardware counter by value.
//
//      <p>This is accomplished by resetting the channel with some
//      caclulated value.
//  */
//  virtual void Adjust(double value) = 0;
// 
//  /** Checks the status of the channel.
//
//      <p>Returns 1 if board is initialized, 0 if its not.
//  */
//  virtual int IsReady(void) = 0;  
//
//  /** Inputs a value form the channel.
//
//      <p>Each child class should (but is not required to) implement
//      these features: read a value from the channel, divide it by field
//      <i>ratio</i>, multiply it by field <i>sign</i>, add field
//      <i>offset</i>, assign the resulting value to field
//      <i>currentInput</i>, and then return this value.
//  */
  virtual double Input(void) = 0;

//  /** Returns the name of this class.
//  */
//  virtual char *GetName(void) = 0;
};


/** DummySensor class.

    <p>This class implements a dummy sensor, that always returns the
    value of a field <i>offset</i>.
*/
class DummySensor : public Sensor {
public:
  
  /** Constructor for DummySensor class.
      
      <p>Passes arguments to the constuctor for the Sensor class.
  */
  DummySensor(int c, double r, double o, int s = 1) : Sensor(c, r, o, s) {}
  
  /** Destructor for DummySensor class.
   */
  ~DummySensor(void) {}

  /** Does nothing.
  */
  void Init(void) {}
  void Init(double) {}

  /** Does nothing.
   */
  void Adjust(double value) {}

  /** Sets and returns the value of filed <i>currentInput</i> to the
      value of filed <i>offset</i>.
  */
  double Input(void) { return (currentInput = offset); }

  /** Always returns 1.
  */
  int IsReady(void) { return 1; }

  /** Returns string "Dummy Sensor".
  */
  const char *GetName(void) { return "Dummy Sensor"; }
};

#endif // SENSOR_H

