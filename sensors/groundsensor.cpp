#include "groundsensor.h"

unsigned int CGroundSensor::SENSOR_NUMBER = NUM_GROUND_SENSORS;
/******************************************************************************/
/******************************************************************************/

CGroundSensor::CGroundSensor(const char* pch_name) :
    CSensor(pch_name, NUM_GROUND_SENSORS )
{
}

/******************************************************************************/
/******************************************************************************/

CGroundSensor::~CGroundSensor()
{
}

/******************************************************************************/
/******************************************************************************/

double* CGroundSensor::ComputeSensorReadings(CEpuck* pc_epuck, CSimulator* pc_simulator){
	
	/* Get Position of the epuck */
	dVector2 vPosition;
	vPosition.x = (pc_epuck->GetPosition()).x; 
	vPosition.y = (pc_epuck->GetPosition()).y;
	double fOrientation;
	fOrientation = (pc_epuck->GetRotation());
	
	/* Get Color of the place where the epuck is */
	CArena* pcArena = pc_simulator->GetArena();
	double* fSensor;
	fSensor = pcArena->GetGroundAreaColor(vPosition,fOrientation);

	for ( int i = 0 ; i < NUM_GROUND_SENSORS ; i++ )
	{
		SetInput(i,fSensor[i]);
	}

	delete [] fSensor;
}

/******************************************************************************/
/******************************************************************************/

double* CGroundSensor::GetSensorReading( CEpuck *p_pcEpuck){

	return GetInputs();
}

/******************************************************************************/
/******************************************************************************/

unsigned int CGroundSensor::GetType(){
	return SENSOR_GROUND;
}

/******************************************************************************/
/******************************************************************************/

//char* CGroundSensor::getGroundAreaName ( CEpuck *p_pcEpuck){
	///* Get Position of the epuck */
	//dVector2 vPosition;
	//vPosition.x = (p_pcEpuck->GetPosition()).x; 
	//vPosition.y = (p_pcEpuck->GetPosition()).y;
	//double fOrientation;
	//fOrientation = (p_pcEpuck->GetRotation());
	///* Get NAME of the place where the epuck is */
	//return m_pcArena->GetGroundAreaName(vPosition,fOrientation);
//}
