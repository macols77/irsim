#ifndef GROUNDSENSOR_H_
#define GROUNDSENSOR_H_

/******************************************************************************/
/******************************************************************************/


/* This is the class of the ground sensor */
/* In the real robot there is 3 ground sensors but here we will use just one, that will correspond to the 
 * central one */
class CGroundSensor;

/******************************************************************************/
/******************************************************************************/

#include "sensor.h"
#include "arena.h"

#define NUM_GROUND_SENSORS 3

/******************************************************************************/
/******************************************************************************/

class CGroundSensor : public CSensor
{
public:
    CGroundSensor(const char* pch_name);
    ~CGroundSensor();

    // Get the sensor type:
    virtual unsigned int GetType();

		//Compute Readings
		double* ComputeSensorReadings(CEpuck* pc_epuck, CSimulator* pc_simulator);
	 	
		//Get Reading
		double* GetSensorReading( CEpuck *pc_epuck);
		
		char* getGroundAreaName( CEpuck *pc_epuck );
    
		static unsigned int SENSOR_NUMBER;
protected:
};

/******************************************************************************/
/******************************************************************************/

#endif
