#ifndef __ACTUATOR_H__
#define __ACTUATOR_H__

class Actuator { 
public:
	int m_channel, m_sign;
	double m_posLimit, m_negLimit;
    double m_offset;
	
	Actuator(int channel, int sign);
	
	virtual ~Actuator(void);
	
	void setLimit(double lower, double upper);
	
	// all subclasses must implement these
	virtual void Init(void) = 0;
	virtual void Reset(void) = 0;
	virtual void Output(double) = 0;
};

#endif
