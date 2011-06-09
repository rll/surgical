#ifndef __QUATECH_H__
#define __QUATECH_H__
#include "actuator.h"

class QuatechDACActuator : public Actuator {
public:
    int slave_num;
      
    QuatechDACActuator(int channel, int sign, double offset, int slave);
    
    ~QuatechDACActuator(void);
    
    void Init(void);
    
    void Reset(void);
    
    void Output(double);

    void SendAll(int slave) { }
};

class QuatechDigitalActuator : public Actuator {
public:
    int mask;
    QuatechDigitalActuator(int channel, int mask);
    ~QuatechDigitalActuator(void);
    void Init();
    void Reset();
    void Output(double volt);
    
    void SendAll(int slave) { }

};

class QuatechDualValveActuator : public Actuator {
public:
    Actuator *valve1, *valve2;

    QuatechDualValveActuator(Actuator*, Actuator*);

    ~QuatechDualValveActuator(void) { };

    void Init(void);

    void Reset(void);

    void Output(double value);  // test value's sign

    void SendAll(int slave) { }
};

#endif
