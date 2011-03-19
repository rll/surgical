#include "actuator.h"

Actuator::Actuator(int channel, int sign) {
	m_channel = channel;
	m_sign = sign;
    m_offset = 0;
    
    return;
}

Actuator::~Actuator(void) {
	
}

void Actuator::setLimit(double lower, double upper) {
	m_negLimit = lower;
	m_posLimit = upper;

	return;
}

