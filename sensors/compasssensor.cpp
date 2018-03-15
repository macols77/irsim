#include "compasssensor.h"

/******************************************************************************/
/******************************************************************************/

CCompassSensor::CCompassSensor(const char* pch_name,CArena* pc_arena) :
    CSensor(pch_name, NUM_COMPASS_SENSORS )
{
	m_pcArena=pc_arena;
}

/******************************************************************************/
/******************************************************************************/

CCompassSensor::~CCompassSensor()
{
}

/******************************************************************************/
/******************************************************************************/

double* CCompassSensor::ComputeSensorReadings(CEpuck* p_pcEpuck, CSimulator* p_pcSimulator){
  SetInput(0, p_pcEpuck->GetRotation());
	return GetInputs();
}


/******************************************************************************/
/******************************************************************************/

unsigned int CCompassSensor::GetType(){
	return SENSOR_COMPASS;
}

/******************************************************************************/
/******************************************************************************/

double* CCompassSensor::GetSensorReading( CEpuck *p_pcEpuck){
	/* return rotation of the robot */
	return GetInputs(); 
}
