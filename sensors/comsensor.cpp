#include "comsensor.h"

/******************************************************************************/
/******************************************************************************/

CComSensor::CComSensor(const char* pch_name) :
    CSensor(pch_name, NUM_COM_SENSORS )
{
}

/******************************************************************************/
/******************************************************************************/

CComSensor::~CComSensor()
{
}

/******************************************************************************/
/******************************************************************************/

double* CComSensor::ComputeSensorReadings(CEpuck* pc_epuck, CSimulator* pc_simulator)
{

	m_pcEpuck = pc_epuck;
	m_pcSim = pc_simulator;
	return GetInputs();
}


/******************************************************************************/
/******************************************************************************/

unsigned int CComSensor::GetType(){
	return SENSOR_COM;
}

/******************************************************************************/
/******************************************************************************/

int CComSensor::GetNeighbourData ( int* data , double* range, double* bearing )
{
	*range = 1000;
	*bearing = 0;

	int uNeighbourFound = false;

	double xCurrentEpuck=0.0, yCurrentEpuck=0.0;
	m_pcEpuck->GetPosition(&xCurrentEpuck, &yCurrentEpuck);

	TEpuckVector* epucks = m_pcSim->GetEpucks();
	TEpuckIterator i;
	for (i = epucks->begin(); i != epucks->end(); i++)
	{
		// avoid sensing oneself
		if( (*i) != m_pcEpuck )
		{
			double xTargetEpuck, yTargetEpuck;
			(*i)->GetPosition(&xTargetEpuck, &yTargetEpuck);
			double fDistance;
			double dx = xTargetEpuck - xCurrentEpuck;
			double dy = yTargetEpuck - yCurrentEpuck;
			fDistance = sqrt(dx*dx + dy*dy);

			//subtract robot radius
			if( fDistance < COM_MAX_RANGE && fDistance < *range)
			{
				double fAngleToTarget;
				fAngleToTarget = atan2(dy,dx);

				/* Get Bearing */
				*bearing = NormalizeAngle (fAngleToTarget - m_pcEpuck->GetRotation());
				/* Get Range */
			 	*range = fDistance;
				/* Get Com Data */
				*data = (*i)->GetComData();
				
				/* Flag true */
				uNeighbourFound = true;
			}
		}
	}
	return uNeighbourFound;
}

/******************************************************************************/
/******************************************************************************/

void CComSensor::SetData ( int data )
{
	m_pcEpuck->SetComData( data );
}

/******************************************************************************/
/******************************************************************************/

