#ifndef PROXSENSOR_H_
#define PROXSENSOR_H_

/******************************************************************************/
/******************************************************************************/

#define PROX_MIN_DISTANCE	0.00
#define PROX_MAX_DISTANCE	0.01

#define NUM_PROX_SENSORS	8

#include "sensor.h"
#include "arena.h"

/******************************************************************************/
/******************************************************************************/

class CProxSensor : public CSensor
{
public:
    CProxSensor(const char* pch_name);
    ~CProxSensor();

    virtual double* ComputeSensorReadings(CEpuck* pc_epuck, CSimulator* pc_simulator); 
    
    // Get the sensor type:
    virtual unsigned int GetType( void );
		virtual int GetSensorIndex ( double fDirection );
		double* GetSensorReading (CEpuck *p_pcEpuck); 
		
		double GetMaxRange ( void ); 
		const double* GetSensorDirections( void );

protected:
	
		static double m_fRange;
		static double      	  m_fProxSensorDir[NUM_PROX_SENSORS];
		static double      	  m_fProxSensorSector[NUM_PROX_SENSORS];

};

/******************************************************************************/
/******************************************************************************/

#endif 
