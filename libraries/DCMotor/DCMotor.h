#ifndef _DCMOTOR_H
#define _DCMOTOR_H

class DCMotor 
{
	int m_motor;

	int m_outpin1;
	int m_outpin2;
	int m_pwmpin;
	
	int m_max;
	int m_min;

public:

	DCMotor( ) 
	: m_motor(0), m_outpin1(-1), m_outpin2(-1), m_pwmpin(-1) 
	{ }
	DCMotor( int pin1, int pin2, int pwmpin ) 
	: m_motor(0), m_outpin1(pin1), m_outpin2(pin2), m_pwmpin(pwmpin) 
	{ }

	~DCMotor() { }

public:
	void attach( int pin1, int pin2, int pwmpin );
	void attach( int pin1, int pin2, int pwmpin, int imin, int imax );

	bool attached();
	void write( int value );

};

#endif
