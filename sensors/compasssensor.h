#ifndef COMPASSSENSOR_H_
#define COMPASSSENSOR_H_

/******************************************************************************/
/******************************************************************************/


/* This is the class of the compass sensor */
/* It returns the absolute direction of the robot (as a compass) between 0 - 2*PI*/
class CCompassSensor;

/******************************************************************************/
/******************************************************************************/

#include "sensor.h"
#include "arena.h"

#define NUM_COMPASS_SENSORS 1

/******************************************************************************/
/******************************************************************************/

class CCompassSensor : public CSensor
{
public:
    CCompassSensor(const char* pch_name,CArena* pc_Arena);
    ~CCompassSensor();

    // Get the sensor type:
    virtual unsigned int GetType();

		//Compute Readings
		double* ComputeSensorReadings(CEpuck* p_pcEpuck, CSimulator* p_pcSimulator);
	 	
		//Get Reading
		double* GetSensorReading( CEpuck *p_pcEpuck );
    
protected:
    CArena* m_pcArena;
};

/******************************************************************************/
/******************************************************************************/

#endif
