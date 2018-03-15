#ifndef LOOKUPIR_H_
#define LOOKUPIR_H_

/******************************************************************************/
/******************************************************************************/

#define NUM_SENSORS 8
#define LOOKUP_MAXRANGE 0.05
#define MINRANGE 0.0
#define LOOKUP_DISTANCE_UNIT 100

//Data related to sample files
#define NUMDISTANCES 11 //From 0.0 to 5 cm
#define NUMANGLES 36 //10 deg angle step
#define DISTANCE_STEP 0.005 //Sampling distance (m) interval
#define ANGLE_STEP 10 //Sampling angle (deg) interval

class CLookupProximity;

/******************************************************************************/
/******************************************************************************/

#include "sensor.h"
#include "arena.h"
#include "random.h"

#include <iostream>
#include <fstream>

/******************************************************************************/
/******************************************************************************/

class CLookupProximity : public CSensor
{
public:
    CLookupProximity(const char* pch_name);
    ~CLookupProximity();

    virtual double* ComputeSensorReadings(CEpuck* pc_epuck, CSimulator* pc_simulator); 
    // Get the sensor type:
    virtual unsigned int GetType();
	//Reading values
    const double* GetIRReadings();
	//Heading of sensors with respect to the robot
	const double* GetIRSensorDirections();

	virtual void SetupSensor(const char* fileName,const char* fileName);

	void NormalizeReadings();
	const float GetMaxRange(){return LOOKUP_MAXRANGE;}
	
protected:
    double m_fIRReadings[NUM_SENSORS];
    static double m_fIRSensorDir[NUM_SENSORS];
	double m_fTable1[NUMDISTANCES][NUMANGLES][8]; //Readings while other epuck is in front
	double m_fTable2[NUMDISTANCES][NUMANGLES][8]; //Readings with other epuck not pointing at the current one
	double m_fMaxReadings[8]; //Maximum delta reading for each sensor, used to normalize the reading
};

/******************************************************************************/
/******************************************************************************/

#endif
