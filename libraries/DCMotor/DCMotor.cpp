#include "DCMotor.h"

#include <Arduino.h>

#define _DCM_USE_PWM_


void DCMotor::attach( int pin1, int pin2, int pwmpin ) 
{
	attach( pin1, pin2, pwmpin, -100, 100 );
}

void DCMotor::attach( int pin1, int pin2, int pwmpin, int imin, int imax ) 
{
	m_outpin1 = pin1;
	m_outpin2 = pin2;
	m_pwmpin  = pwmpin;
	m_min     = imin;
	m_max     = imax;
}

bool DCMotor::attached() 
{
	return ((m_outpin1!=-1)&&(m_outpin2!=-1)&&(m_pwmpin!=-1));
}

void DCMotor::write( int value ) 
{
	int cvalue = constrain( value, m_min, m_max );
#ifdef _DCM_USE_PWM_
	if( cvalue == 0 ) { // 
		digitalWrite(m_outpin1, LOW);
		digitalWrite(m_outpin2, LOW);
	} else if ( 0 < cvalue ) {
		digitalWrite(m_outpin1, LOW);
		digitalWrite(m_outpin2, HIGH);
		analogWrite(m_pwmpin,cvalue);
	} else { 
		digitalWrite(m_outpin1, LOW);
		digitalWrite(m_outpin2, HIGH);
		analogWrite(m_pwmpin,-cvalue);
	}
#else
	if( cvalue == 0 ) { // 
		digitalWrite(m_outpin1, LOW);
		digitalWrite(m_outpin2, LOW);
	} else if( 0 < cvalue ) {
		analogWrite(m_outpin1, cvalue);
		analogWrite(m_outpin2, 0);
	} else {
		analogWrite(m_outpin1, 0);
		analogWrite(m_outpin2, -cvalue);
	}
#endif
}

