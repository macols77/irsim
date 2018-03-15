#ifndef RANDBSENSOR_H_
#define RANDBSENSOR_H_

/******************************************************************************/
/******************************************************************************/


class CRandbSensor;

/******************************************************************************/
/******************************************************************************/

#include "sensor.h"
#include "arena.h"

#define NUM_RANDB_SENSORS 1
#define RANDB_MAX_RANGE 0.2 //10 cm
/******************************************************************************/
/******************************************************************************/

class CRandbSensor : public CSensor
{
public:
    CRandbSensor(const char* pch_name);
    ~CRandbSensor();

    // Get the sensor type:
    virtual unsigned int GetType();

		//Compute Readings
		double* ComputeSensorReadings(CEpuck* p_pcEpuck, CSimulator* p_pcSimulator);
	 	
		/* Output */
		void ResetNestPosition ( void );
		void ResetPreyPosition ( void );
		void SetNestPosition ( dVector2 goal_estimated, dVector2 me_pos_estimated, double me_orien_estimated, long int confidence_level );
		void SetPreyPosition ( dVector2 goal_estimated, dVector2 me_pos_estimated, double me_orien_estimated, long int confidence_level );
		
		/* Input */
		int GetNeighbourNest ( dVector2* position, double* range, double* bearing , dVector2 me_pos_estimated, double me_orien_estimated, long int* confidence_level );
		int GetNeighbourPrey ( dVector2* position, double* range, double* bearing , dVector2 me_pos_estimated, double me_orien_estimated, long int* confidence_level );


    
protected:
    CArena* m_pcArena;
		CEpuck* m_pcEpuck;
		CSimulator* m_pcSim;
};

/******************************************************************************/
/******************************************************************************/

#endif
