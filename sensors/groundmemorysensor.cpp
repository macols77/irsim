#include "groundmemorysensor.h"

/******************************************************************************/
/******************************************************************************/

CGroundMemorySensor::CGroundMemorySensor(const char* pch_name) :
    CSensor(pch_name, NUM_GROUND_MEMORY_SENSORS )
{
	m_fStatus = 0.0;
}

/******************************************************************************/
/******************************************************************************/

CGroundMemorySensor::~CGroundMemorySensor()
{
}

/******************************************************************************/
/******************************************************************************/

double* CGroundMemorySensor::ComputeSensorReadings(CEpuck* pc_epuck, CSimulator* pc_simulator){

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

	//double* groundSensor = new double[NUM_GROUND_SENSOR];
	
	if ( fSensor[1] < 0.8)
	{
		if ( m_fStatus == 0.0 & fSensor[1] == 0.5 ) 
		{
			m_fStatus = 1.0;
		}

		else if ( m_fStatus == 1.0  & fSensor[1] == 0.0 )
		{
			m_fStatus = 0.0;
		}
	}

	delete [] fSensor;
	//groundSensor[0]=m_fStatus;
	SetInput(0,m_fStatus);
}

/******************************************************************************/
/******************************************************************************/

double* CGroundMemorySensor::GetSensorReading( CEpuck *p_pcEpuck){
	return GetInputs();
}

/******************************************************************************/
/******************************************************************************/

unsigned int CGroundMemorySensor::GetType(){
	return SENSOR_GROUND_MEMORY;
}

/******************************************************************************/
/******************************************************************************/

//char* CGroundMemorySensor::getGroundAreaName ( CEpuck *p_pcEpuck){
	///* Get Position of the epuck */
	//dVector2 vPosition;
	//vPosition.x = (p_pcEpuck->GetPosition()).x; 
	//vPosition.y = (p_pcEpuck->GetPosition()).y;
	//double fOrientation;
	//fOrientation = (p_pcEpuck->GetRotation());
	///* Get NAME of the place where the epuck is */
	//return m_pcArena->GetGroundAreaName(vPosition,fOrientation);
//}

void CGroundMemorySensor::Reset()
{
	m_fStatus = 0.0;
	SetInput(0,m_fStatus);
}
