#include "contactsensor.h"

/******************************************************************************/
/******************************************************************************/

double CContactSensor::m_fRange = 0.05;
double   CContactSensor::m_fContactSensorDir[NUM_CONTACT_SENSORS] 	= {0.2967, 0.8727, 1.5708, 2.6180, 3.6652, 4.7124, 5.4105, 5.9865};
double   CContactSensor::m_fContactSensorSector[NUM_CONTACT_SENSORS] 	= {0.0,0.5847,1.614465,2.0944,M_PI,4.1808,5.06145,5.6985};
unsigned int CContactSensor::SENSOR_NUMBER = NUM_CONTACT_SENSORS;

/******************************************************************************/
/******************************************************************************/

CContactSensor::CContactSensor(const char* pch_name) : CSensor(pch_name, NUM_CONTACT_SENSORS )
{
}

/******************************************************************************/
/******************************************************************************/

CContactSensor::~CContactSensor()
{
}

/******************************************************************************/
/******************************************************************************/

double* CContactSensor::ComputeSensorReadings(CEpuck* pc_epuck, CSimulator* pc_simulator)
{
	// Initialization to 0
	for (int i = 0; i < NUM_CONTACT_SENSORS ; i++)
	{// Initialization to 0
		SetInput(i,0);
	}

	double fPosX=0.0, fPosY=0.0;
	pc_epuck->GetPosition(&fPosX, &fPosY);

	// first check proximity to epucks
	TEpuckVector* epucks = pc_simulator->GetEpucks();
	TEpuckIterator i;
	for (i = epucks->begin(); i != epucks->end(); i++)
	{
		// avoid sensing oneself
		if( (*i) != pc_epuck )
		{
			double xCurrentEpuck, yCurrentEpuck, xTargetEpuck, yTargetEpuck;
			pc_epuck->GetPosition(&xCurrentEpuck, &yCurrentEpuck);
			(*i)->GetPosition(&xTargetEpuck, &yTargetEpuck);
			double fDistance;
			double dx = xTargetEpuck - xCurrentEpuck;
			double dy = yTargetEpuck - yCurrentEpuck;
			fDistance = sqrt(dx*dx + dy*dy);

			//subtract robot radius
			double fTurretRadius = CEpuck::CHASSIS_RADIUS;
			fDistance -= fTurretRadius;
			if( fDistance < m_fRange )
			{
				double fAngleToTarget = atan2(dy,dx);
				double fDirection = NormalizeAngle (fAngleToTarget - pc_epuck->GetRotation());
			
				//int sensorIndex = GetSensorIndex(fDirection);
				//SetInput(sensorIndex,1);	
				
				/* Calc angles to the robot's body */
				double fTanToRobot = NormalizeAngle (atan2(CEpuck::CHASSIS_RADIUS, fDistance));
				//printf("fDirection: %2f - fTanToRobot: %2f\n\n", fDirection,fTanToRobot);

				for ( int sensorIndex = 0 ; sensorIndex < NUM_CONTACT_SENSORS ; sensorIndex++)
				{
					/* If within a 10º of aperture angle */
					//if ( (fDirection > ( m_fContactSensorDir[sensorIndex] - 0.17 )) && (fDirection < ( m_fContactSensorDir[sensorIndex] + 0.17 )))
					
					/* If no problem with 0º */
					if (fDirection > fTanToRobot ){
						if ( ( ( fDirection + fTanToRobot ) > ( m_fContactSensorDir[sensorIndex] - APERTURE_ANGLE ) && ( fDirection - fTanToRobot ) < ( m_fContactSensorDir[sensorIndex] - APERTURE_ANGLE ) ) ||
							 ( ( ( fDirection + fTanToRobot ) > ( m_fContactSensorDir[sensorIndex] + APERTURE_ANGLE ) && ( fDirection - fTanToRobot ) < ( m_fContactSensorDir[sensorIndex] + APERTURE_ANGLE ) ) ) )
						{
							SetInput(sensorIndex,fDistance);	
						}
					}
					/* If problems on 0º */
					else
					{
						if ( ( ( fDirection + fTanToRobot ) > ( m_fContactSensorDir[sensorIndex] - APERTURE_ANGLE ) && ( fDirection - fTanToRobot + 2*M_PI) < ( m_fContactSensorDir[sensorIndex] - APERTURE_ANGLE + 2*M_PI) ) ||
							 ( ( ( fDirection + fTanToRobot + 2*M_PI) > ( m_fContactSensorDir[sensorIndex] + APERTURE_ANGLE ) && ( fDirection - fTanToRobot + 2*M_PI) < ( m_fContactSensorDir[sensorIndex] + APERTURE_ANGLE ) ) ) )
						{
							SetInput(sensorIndex,1);	
						}
					}
				}
			}
		}
	}

	// then check for wall proximity
	CArena* pcArena = pc_simulator->GetArena();
	
	/* NEW */
	if( pcArena->GetArenaType() == ARENA_TYPE_SQUARE )
	{
		double fSizeX, fSizeY;
		unsigned int unNumCellsX, unNumCellsY;
		pcArena->GetSize(&fSizeX,&fSizeY);
		pcArena->GetResolution(&unNumCellsX,&unNumCellsY);
		double fCellSizeX = fSizeX/(double)unNumCellsX;
		double fCellSizeY = fSizeY/(double)unNumCellsY;
		if( fabs(fCellSizeX - fCellSizeY)>1.23456e-7 )
		{
			printf("ERROR: in CContactSensor::ComputeSensorReadings wrong cell size: fSizeX/unNumCellsX:%f , fSizeY/unNumCellsY:%f , diff:%f\n",fCellSizeX,fCellSizeY,fCellSizeX-fCellSizeY);
		}
		double fCellSize = fCellSizeX;


		double fX = fPosX;
		double fY = fPosY;


		double epuckRotation = pc_epuck->GetRotation();
		/* For EverySensor */
		for ( int sensorIndex = 0 ; sensorIndex < NUM_CONTACT_SENSORS ; sensorIndex++)
		{
			/* Init values */
			bool bWallEncountered = false;
			fX = fPosX;
			fY = fPosY;
			double fRayLength = 0;

			/* Get Ray of sensor  */
			double fRayAngle = NormalizeAngle ( epuckRotation + m_fContactSensorDir[sensorIndex] );
		
			//printf("Robot PosX: %2f, Robot PosY: %2f, Robot Orien: %2f, fRayAngle: %2f\n", fPosX, fPosY, epuckRotation, fRayAngle);
			/* Whle not wall and ray less than range */
			while( !bWallEncountered && fRayLength < m_fRange)
			{
				/* Calc ray on next cell */
				fX += (fCellSize/50) * cos(fRayAngle);
				fY += (fCellSize/50) * sin(fRayAngle);
			
				/* If obstacle on that position */
				if( pcArena->GetHeight(fX,fY) >= HEIGHT_OBSTACLE )
				{
					SetInput(sensorIndex,1);	
					bWallEncountered = true;
				}
				
				/* Calc fRayLength */
				fRayLength = sqrt ( pow (fX - fPosX, 2) + pow (fY - fPosY, 2 ) );
				//printf("fRayLength: %2f\n", fRayLength);
			}
		}
	}

	return GetInputs();
}

/******************************************************************************/
/******************************************************************************/

int CContactSensor::GetSensorIndex(double fDirection)
{
	int index;
	for ( index = 1 ; index < (NUM_CONTACT_SENSORS - 1); index++)
	{
		if ( fDirection < m_fContactSensorSector[index])
		{
			return (index-1);
		}
	}
	return index;
}

/******************************************************************************/
/******************************************************************************/

unsigned int CContactSensor::GetType()
{
    return SENSOR_CONTACT;
}

/******************************************************************************/
/******************************************************************************/

double* CContactSensor::GetSensorReading ( CEpuck *p_pcEpuck )
{
	return GetInputs();
}
/******************************************************************************/
/******************************************************************************/

double CContactSensor::GetMaxRange ( void ) 
{
	return m_fRange; 
}

/******************************************************************************/
/******************************************************************************/

const double* CContactSensor::GetSensorDirections()
{
    return m_fContactSensorDir;
}
