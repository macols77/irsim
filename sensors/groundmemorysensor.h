#ifndef GROUNDMEMORYSENSOR_H_
#define GROUNDMEMORYSENSOR_H_

/******************************************************************************/
/******************************************************************************/


/* This is the class of the ground sensor */
/* In the real robot there is 3 ground sensors but here we will use just one, that will correspond to the 
 * central one */
class CGroundMemorySensor;

/******************************************************************************/
/******************************************************************************/

#include "sensor.h"
#include "arena.h"

#define NUM_GROUND_MEMORY_SENSORS 1

/******************************************************************************/
/******************************************************************************/

class CGroundMemorySensor : public CSensor
{
public:
    CGroundMemorySensor(const char* pch_name);
    ~CGroundMemorySensor();

    // Get the sensor type:
    virtual unsigned int GetType();

		//Compute Readings
		double* ComputeSensorReadings(CEpuck* pc_epuck, CSimulator* pc_simulator);
	 	
		//Get Reading
		double* GetSensorReading( CEpuck *pc_epuck);
		
		char* getGroundAreaName( CEpuck *pc_epuck );

		void Reset();
    
protected:
		float m_fStatus;
};

/******************************************************************************/
/******************************************************************************/

#endif
