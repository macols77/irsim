#include "randbsensor.h"

/******************************************************************************/
/******************************************************************************/

CRandbSensor::CRandbSensor(const char* pch_name) :
    CSensor(pch_name, NUM_RANDB_SENSORS )
{
}

/******************************************************************************/
/******************************************************************************/

CRandbSensor::~CRandbSensor()
{
}

/******************************************************************************/
/******************************************************************************/

double* CRandbSensor::ComputeSensorReadings(CEpuck* pc_epuck, CSimulator* pc_simulator)
{

	m_pcEpuck = pc_epuck;
	m_pcSim = pc_simulator;
	return GetInputs();
}


/******************************************************************************/
/******************************************************************************/

unsigned int CRandbSensor::GetType(){
	return SENSOR_RANDB;
}

/******************************************************************************/
/******************************************************************************/

int CRandbSensor::GetNeighbourNest ( dVector2* position, double* range, double* bearing , dVector2 me_pos_estimated, double me_orien_estimated, long int* confidence_level)
{
	*range = 10;
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
			if( fDistance < RANDB_MAX_RANGE && fDistance < *range)
			{
				double fAngleToTarget;
				fAngleToTarget = atan2(dy,dx);

				*bearing = NormalizeAngle (fAngleToTarget - m_pcEpuck->GetRotation());
			 	*range = fDistance;

				/* Get Nest and Prey Coordinates */
				dVector2 vNestObtained;
				if ( (*i)->GetNestPosition( &vNestObtained, confidence_level))
				{
					/* Calc Rotation of the relative axis of the robot */
					double fThetaAxis = m_pcEpuck->GetRotation() - me_orien_estimated;

					/* Calc the position of the Robot oriented to the inertial system */
					dVector2 vPosEstimatedOrien;
					vPosEstimatedOrien.x = me_pos_estimated.x * cos ( fThetaAxis) - me_pos_estimated.y * sin ( fThetaAxis);
					vPosEstimatedOrien.y = me_pos_estimated.x * sin ( fThetaAxis) + me_pos_estimated.y * cos ( fThetaAxis);

					/* Calc the x,y position of the relative axis */
					dVector2 vAxisPos;
					vAxisPos.x = xCurrentEpuck - vPosEstimatedOrien.x;
					vAxisPos.y = yCurrentEpuck - vPosEstimatedOrien.y;

					/* Translate the Nest Position to the correct relative axis */
					dVector2 vNestPosEstimatedOrien;
					vNestPosEstimatedOrien.x = vNestObtained.x - vAxisPos.x;
					vNestPosEstimatedOrien.y = vNestObtained.y - vAxisPos.y;

					/* Transform the intertial orientaiton to the relative one */
					(*position).x = vNestPosEstimatedOrien.x * cos (fThetaAxis) + vNestPosEstimatedOrien.y * sin (fThetaAxis);
					(*position).y = (-vNestPosEstimatedOrien.x) * sin (fThetaAxis) + vNestPosEstimatedOrien.y * cos (fThetaAxis);
					
					uNeighbourFound = true;
				}
			}
		}
	}
	if (uNeighbourFound)	
		return true;
	else
		return false;
}

/******************************************************************************/
/******************************************************************************/

int CRandbSensor::GetNeighbourPrey ( dVector2* position, double* range, double* bearing , dVector2 me_pos_estimated, double me_orien_estimated, long int* confidence_level)
{

	*range = 10;
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
			if( fDistance < RANDB_MAX_RANGE && fDistance < *range)
			{
				double fAngleToTarget;
				fAngleToTarget = atan2(dy,dx);

				*bearing = NormalizeAngle (fAngleToTarget - m_pcEpuck->GetRotation());
			 	*range = fDistance;

				/* Get Prey Coordinates */
				dVector2 vPreyObtained;
				if ( (*i)->GetPreyPosition( &vPreyObtained , confidence_level))
				{
					/* Calc Rotation of the relative axis of the robot */
					double fThetaAxis = m_pcEpuck->GetRotation() - me_orien_estimated;

					/* Calc the position of the Robot oriented to the inertial system */
					dVector2 vPosEstimatedOrien;
					vPosEstimatedOrien.x = me_pos_estimated.x * cos ( fThetaAxis) - me_pos_estimated.y * sin ( fThetaAxis);
					vPosEstimatedOrien.y = me_pos_estimated.x * sin ( fThetaAxis) + me_pos_estimated.y * cos ( fThetaAxis);

					/* Calc the x,y position of the relative axis */
					dVector2 vAxisPos;
					vAxisPos.x = xCurrentEpuck - vPosEstimatedOrien.x;
					vAxisPos.y = yCurrentEpuck - vPosEstimatedOrien.y;

					/* Translate the Nest Position to the correct relative axis */
					dVector2 vPreyPosEstimatedOrien;
					vPreyPosEstimatedOrien.x = vPreyObtained.x - vAxisPos.x;
					vPreyPosEstimatedOrien.y = vPreyObtained.y - vAxisPos.y;

					/* Transform the intertial orientaiton to the relative one */
					(*position).x = vPreyPosEstimatedOrien.x * cos (fThetaAxis) + vPreyPosEstimatedOrien.y * sin (fThetaAxis);
					(*position).y = (-vPreyPosEstimatedOrien.x) * sin (fThetaAxis) + vPreyPosEstimatedOrien.y * cos (fThetaAxis);
					
					uNeighbourFound = true;
				}
			}
		}
	}
	if (uNeighbourFound)	
		return true;
	else
		return false;
}

void CRandbSensor::ResetNestPosition ( void )
{
	m_pcEpuck->ResetNestPosition();
}

/******************************************************************************/
/******************************************************************************/

void CRandbSensor::ResetPreyPosition ( void )
{
	m_pcEpuck->ResetPreyPosition();
}

/******************************************************************************/
/******************************************************************************/

void CRandbSensor::SetNestPosition ( dVector2 goal_estimated , dVector2 me_pos_estimated, double me_orien_estimated, long int confidence_level)
{
	m_pcEpuck->SetNestPosition(goal_estimated, me_pos_estimated, me_orien_estimated, confidence_level);
}

/******************************************************************************/
/******************************************************************************/

void CRandbSensor::SetPreyPosition ( dVector2 goal_estimated , dVector2 me_pos_estimated, double me_orien_estimated, long int confidence_level)
{
	m_pcEpuck->SetPreyPosition(goal_estimated, me_pos_estimated, me_orien_estimated, confidence_level);
}

/******************************************************************************/
/******************************************************************************/

