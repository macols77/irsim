#ifndef COMSENSOR_H_
#define COMSENSOR_H_

/******************************************************************************/
/******************************************************************************/


class CComSensor;

/******************************************************************************/
/******************************************************************************/

#include "sensor.h"
#include "arena.h"

#define NUM_COM_SENSORS 1
#define COM_MAX_RANGE 0.2 //20 cm
/******************************************************************************/
/******************************************************************************/

class CComSensor : public CSensor
{
public:
    CComSensor(const char* pch_name);
    ~CComSensor();

    // Get the sensor type:
    virtual unsigned int GetType();

		//Compute Readings
		double* ComputeSensorReadings(CEpuck* p_pcEpuck, CSimulator* p_pcSimulator);
	 	
		/* Output */
		void SetData( int data);
		
		/* Input */
		int GetNeighbourData ( int* data, double* range, double* bearing);


    
protected:
    CArena* m_pcArena;
		CEpuck* m_pcEpuck;
		CSimulator* m_pcSim;
};

/******************************************************************************/
/******************************************************************************/

#endif
