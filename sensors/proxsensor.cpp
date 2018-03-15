#include "proxsensor.h"

/******************************************************************************/
/******************************************************************************/

double CProxSensor::m_fRange = 0.30;
double   CProxSensor::m_fProxSensorDir[NUM_PROX_SENSORS] 	= {0.2967, 0.8727, 1.5708, 2.6180, 3.6652, 4.7124, 5.4105, 5.9865};
double   CProxSensor::m_fProxSensorSector[NUM_PROX_SENSORS] 	= {0.0,0.5847,1.614465,2.0944,M_PI,4.1808,5.06145,5.6985};

/******************************************************************************/
/******************************************************************************/

CProxSensor::CProxSensor(const char* pch_name) : CSensor(pch_name, NUM_PROX_SENSORS )
{
}

/******************************************************************************/
/******************************************************************************/

CProxSensor::~CProxSensor()
{
}

/******************************************************************************/
/******************************************************************************/

double* CProxSensor::ComputeSensorReadings(CEpuck* pc_epuck, CSimulator* pc_simulator)
{
	// Initialization to 0
	for (int i = 0; i < NUM_PROX_SENSORS ; i++)
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
				double fAngleToTarget;
				fAngleToTarget = atan2(dy,dx);

				double fDirection = NormalizeAngle (fAngleToTarget - pc_epuck->GetRotation());
			
				int sensorIndex = GetSensorIndex(fDirection);
				SetInput(sensorIndex,1);	

			}
		}
	}

	// then check for wall proximity
	CArena* pcArena = pc_simulator->GetArena();

	// then check for wall proximity in a squared arena
	// and for rectangular obstacles
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
			printf("ERROR: in CProxSensor::ComputeSensorReadings wrong cell size: fSizeX/unNumCellsX:%f , fSizeY/unNumCellsY:%f , diff:%f\n",fCellSizeX,fCellSizeY,fCellSizeX-fCellSizeY);
		}
		double fCellSize = fCellSizeX;

		bool bWallEncountered = false;
		double fX = fPosX;
		double fY = fPosY;

		double fDistanceToLeftCellBorder 	= ((fX+fSizeX/2)/fCellSize)*fCellSize-(unsigned int)((fX+fSizeX/2)/fCellSize)*fCellSize;
		double fDistanceToRightCellBorder 	= fCellSize - fDistanceToLeftCellBorder;
		double fDistanceToBottomCellBorder 	= ((fY+fSizeY/2)/fCellSize)*fCellSize-(unsigned int)((fY+fSizeY/2)/fCellSize)*fCellSize;
		double fDistanceToTopCellBorder   	= fCellSize - fDistanceToBottomCellBorder;

		// check to the left
		while( !bWallEncountered &&
				fX > fPosX-m_fRange )
		{
			fX -= fCellSize;
			if( pcArena->GetHeight(fX,fY) == HEIGHT_OBSTACLE )
			{
				bWallEncountered = true;
				double fDistance = fPosX - fX - fDistanceToRightCellBorder;
				if( fDistance < m_fRange )
				{
					double fDirection = NormalizeAngle( M_PI-pc_epuck->GetRotation() );
					
					int sensorIndex = GetSensorIndex(fDirection);
					SetInput(sensorIndex,fDistance);	
					printf("A: %d - %2f\n",sensorIndex,fDistance);
				}
			}
		}

		// check to the right
		fX = fPosX;
		bWallEncountered = false;
		while( !bWallEncountered &&
				fX < fPosX+m_fRange )
		{
			fX += fCellSize;
			if( pcArena->GetHeight(fX,fY) == HEIGHT_OBSTACLE )
			{
				bWallEncountered = true;
				double fDistance = fX - fPosX - fDistanceToLeftCellBorder;
				//printf("fDistance:%f , fX:%f , ownX:%f , toRight:%f \n",fDistance,fX,fPosX,fDistanceToRightCellBorder);
				if( fDistance < m_fRange )
				{
					double fDirection = NormalizeAngle( -pc_epuck->GetRotation());
				
					int sensorIndex = GetSensorIndex(fDirection);
					SetInput(sensorIndex,fDistance);	
					printf("B: %d - %2f\n",sensorIndex,fDistance);
				}
			}
		}

		// check to the top
		fX = fPosX;
		bWallEncountered = false;
		while( !bWallEncountered &&
				fY < fPosY+m_fRange )
		{
			fY += fCellSize;
			if( pcArena->GetHeight(fX,fY) == HEIGHT_OBSTACLE )
			{
				bWallEncountered = true;
				double fDistance = fY - fPosY - fDistanceToBottomCellBorder;
				printf("fDistance:%f , fY:%f , ownY:%f , toTop:%f \n", fDistance,fY,fPosY,fDistanceToTopCellBorder);
				if( fDistance < m_fRange )
				{
					double fDirection = NormalizeAngle( M_PI/2.0 -pc_epuck->GetRotation());
	
					int sensorIndex = GetSensorIndex(fDirection);
					SetInput(sensorIndex,fDistance);	
					printf("C: %d - %2f -- %2f\n",sensorIndex,fDistance,fDirection);
				}
			}
		}

		// check to the bottom
		fY = fPosY;
		bWallEncountered = false;
		while( !bWallEncountered &&
				fY > fPosY-m_fRange )
		{
			fY -= fCellSize;
			if( pcArena->GetHeight(fX,fY) == HEIGHT_OBSTACLE )
			{
				bWallEncountered = true;
				double fDistance = fPosY - fY - fDistanceToTopCellBorder;
				// 	  printf("fDistance:%f , fY:%f , ownY:%f , toBottom:%f \n",fDistance,fY,fPosY,fDistanceToBottomCellBorder);
				if( fDistance < m_fRange )
				{
					double fDirection = NormalizeAngle( -M_PI/2.0 -pc_epuck->GetRotation());
				
					int sensorIndex = GetSensorIndex(fDirection);
					SetInput(sensorIndex,fDistance);	
					printf("D: %d - %2f\n",sensorIndex,fDistance);
				}
			}
		}
	}

	return GetInputs();
}

/******************************************************************************/
/******************************************************************************/

int CProxSensor::GetSensorIndex(double fDirection)
{
	int index;
	for ( index = 1 ; index < (NUM_PROX_SENSORS - 1); index++)
	{
		if ( fDirection < m_fProxSensorSector[index])
		{
			return (index-1);
		}
	}
	return index;
}

/******************************************************************************/
/******************************************************************************/

unsigned int CProxSensor::GetType()
{
    return SENSOR_PROX;
}

/******************************************************************************/
/******************************************************************************/

double* CProxSensor::GetSensorReading ( CEpuck *p_pcEpuck )
{
	return GetInputs();
}
/******************************************************************************/
/******************************************************************************/

double CProxSensor::GetMaxRange ( void ) 
{
	return m_fRange; 
}

/******************************************************************************/
/******************************************************************************/

const double* CProxSensor::GetSensorDirections()
{
    return m_fProxSensorDir;
}
