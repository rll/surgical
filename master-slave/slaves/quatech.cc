#include "quatech.h"
#include "util.h"
#include <stdio.h>

QuatechDACActuator::QuatechDACActuator(int channel, int sign, double offset, int slave)
: Actuator(channel, sign) {
	m_offset = offset;
    slave_num = slave;
	
    // The outb below were fucking things up...
    if (slave == 1) {
	    getperm(QUATECH_DABASE1,4);
        outb(0x80, QUATECH_DABASE1 + 3);              // setup the address
    } else if (slave == 2) {
        getperm(QUATECH_DABASE2,4);
        outb(0x80, QUATECH_DABASE2 + 3);              // setup the address
    }

	Output(m_offset);	// make sure actuator is stopped

	return;
}

QuatechDACActuator::~QuatechDACActuator(void) {
	Reset();
	
	return;
}

// deprecated
void QuatechDACActuator::Init(void)
{
  getperm(QUATECH_DABASE1,4);
  getperm(QUATECH_DABASE2,4);
  outb(0x80, QUATECH_DABASE1 + 3);              // setup the address
  outb(0x80, QUATECH_DABASE2 + 3);              // setup the address
/*
  for(int i = 0; i < 16; i++)
    quatech_outdac8_s(i, QUATECH_ZEROVOLT);    // send out 0 Volt
*/
}

void QuatechDACActuator::Reset(void) 
{
  // ToggleDither(false);
    Output(0.0);
	//Output(m_offset);
}


void QuatechDACActuator::Output(double value)
{
	double outV = value;

    if(value > m_posLimit) outV = m_posLimit;
    else if(value < m_negLimit) outV = m_negLimit;
    // else lastOutput = value;

    outV *= m_sign;
    //printf("a[%d]->%4.4f\n", m_channel, m_offset + outV);
	quatech_vout(m_channel, m_offset + outV, slave_num);
    // quatech_vout(channel, (lastOutput + DitherTerm()) * sign);
}

QuatechDigitalActuator::QuatechDigitalActuator(int channel, int _mask) : Actuator(channel, 1) {
    mask = _mask;

    getperm(QUATECH_DIGBASE,4);
    outb(0x80, QUATECH_DIGBASE + 0x03);     // set up the address
    for(int i = 0; i < 3; i++)
        outb(0x00, QUATECH_DIGBASE + i);    // send out 0x0
}

QuatechDigitalActuator::~QuatechDigitalActuator() { 
    
};

void QuatechDigitalActuator::Init() { 
}

void QuatechDigitalActuator::Reset() {
    Output(0.0);
}

void QuatechDigitalActuator::Output(double value) {
    unsigned char v = (unsigned char)value;
    v &= mask;
    getperm(QUATECH_DIGBASE, 4);
    outb((inb(QUATECH_DIGBASE) & ~mask) | v, QUATECH_DIGBASE);
    //for(int i=0;i<4;i++)
    //    printf("%x ", inb(QUATECH_DIGBASE+i));
    //printf("\n");
}

QuatechDualValveActuator::QuatechDualValveActuator(Actuator *p, Actuator *q): Actuator (0, 1) {
    valve1 = p;
    valve2 = q;
    return;
}

void QuatechDualValveActuator::Init(void) {
    valve1->Init();
    valve2->Init();
    return;
}

void QuatechDualValveActuator::Reset(void) {
    valve1->Reset();
    valve2->Reset();
    return;
}

void QuatechDualValveActuator::Output(double value) {
    if(value > 0) {
        valve1->Output(0xff);
        valve2->Output(0xff);
    } else {
        valve1->Output(0x00);
        valve2->Output(0x00);
    }
    return;
}

