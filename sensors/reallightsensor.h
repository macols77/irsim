#ifndef REALLIGHTSENSOR_H_
#define REALLIGHTSENSOR_H_

/******************************************************************************/
/******************************************************************************/

class CRealLightSensor;

/******************************************************************************/
/******************************************************************************/

#include "sensor.h"
#include "arena.h"

#define NUM_REAL_LIGHT_SENSORS 8

/******************************************************************************/
/******************************************************************************/

class CRealLightSensor : public CSensor
{
public:
    CRealLightSensor(const char* pch_name, double f_rangeLightSensor);
    ~CRealLightSensor();

    // Get the sensor type:
    virtual unsigned int GetType();

		//Compute Readings
		double* ComputeSensorReadings(CEpuck* p_pcEpuck, CSimulator* p_pcSimulator);
	 	
		//Get Reading. Returns if hot spot is visible.
		double* GetSensorReading( CEpuck *p_pcEpuck);

		double GetMaxRange ( void ); 
		const double* GetSensorDirections( void );
		
		void SwitchNearestLight ( int n_value );

		static unsigned int SENSOR_NUMBER;
	
protected:
		CArena* m_pcArena;
		CEpuck* m_pcEpuck;

		double m_fRangeLightSensor;
		static double m_fLightSensorDir[NUM_REAL_LIGHT_SENSORS];
};

/******************************************************************************/
/******************************************************************************/

#endif
