#ifndef CONTACTSENSOR_H_
#define CONTACTSENSOR_H_

/******************************************************************************/
/******************************************************************************/

#define CONTACT_MIN_DISTANCE	0.00
#define CONTACT_MAX_DISTANCE	0.01
#define APERTURE_ANGLE		0.17

#define NUM_CONTACT_SENSORS	8

#include "sensor.h"
#include "arena.h"

/******************************************************************************/
/******************************************************************************/

class CContactSensor : public CSensor
{
public:
    CContactSensor(const char* pch_name);
    ~CContactSensor();

    virtual double* ComputeSensorReadings(CEpuck* pc_epuck, CSimulator* pc_simulator); 
    
    // Get the sensor type:
    virtual unsigned int GetType( void );
		virtual int GetSensorIndex ( double fDirection );
		double* GetSensorReading (CEpuck *p_pcEpuck); 
		
		double GetMaxRange ( void ); 
		const double* GetSensorDirections( void );

		static unsigned int SENSOR_NUMBER;
protected:
	
		static double m_fRange;
		static double      	  m_fContactSensorDir[NUM_CONTACT_SENSORS];
		static double      	  m_fContactSensorSector[NUM_CONTACT_SENSORS];

};

/******************************************************************************/
/******************************************************************************/

#endif 
