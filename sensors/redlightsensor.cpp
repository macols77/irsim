#include "redlightsensor.h"

/******************************************************************************/
/******************************************************************************/

double CRedLightSensor::m_fLightSensorDir[NUM_LIGHT_SENSORS] 	= {0.2967, 0.8727, 1.5708, 2.6180, 3.6652, 4.7124, 5.4105, 5.9865};
unsigned int CRedLightSensor::SENSOR_NUMBER = NUM_LIGHT_SENSORS;

/******************************************************************************/
/******************************************************************************/

CRedLightSensor::CRedLightSensor(const char* pch_name, double f_rangeLightSensor) :
    CSensor(pch_name, NUM_LIGHT_SENSORS )
{
	m_fRangeLightSensor = f_rangeLightSensor;
}

/******************************************************************************/
/******************************************************************************/

CRedLightSensor::~CRedLightSensor()
{
}

/******************************************************************************/
/******************************************************************************/

double* CRedLightSensor::ComputeSensorReadings(CEpuck* pc_epuck, CSimulator* pc_simulator){

	double * lightMeasures = new double[NUM_LIGHT_SENSORS];
	/* Update Sensor array with null values*/
	for ( int i = 0 ; i < NUM_LIGHT_SENSORS ; i++)
	{
		lightMeasures[i] = 0.0;
	}
	
	dVector2 vPosition;
	vPosition.x = (pc_epuck->GetPosition()).x; 
	vPosition.y = (pc_epuck->GetPosition()).y;

  CArena* pcArena = pc_simulator->GetArena();
	m_pcArena = pc_simulator->GetArena();
	
	m_pcEpuck = pc_epuck;

	/* Get Nearest Light Object in range*/
	dVector2 vLightPosition;
	double fDistance;
	if ( pcArena->GetNearestRedLight(vPosition, m_fRangeLightSensor, &vLightPosition, &fDistance))
	{
		
		/* Get Actual Orientation of the robot */
		double fOrientation = pc_epuck->GetRotation();
	
		/* Calc absolute incidence light beam angle */
		double fAbsAngle = atan2((vLightPosition.y - vPosition.y),(vLightPosition.x - vPosition.x));

		/* Calc angle relative to robot orientation */
		double fRelAngle = fAbsAngle - fOrientation;

		fRelAngle = NormalizeAngle(fRelAngle);

		/* Normalize Distance */
		fDistance = ( m_fRangeLightSensor - fDistance ) / m_fRangeLightSensor;

		/* Update Sensor array */
		for ( int i = 0 ; i < NUM_LIGHT_SENSORS ; i ++)
		{
			/* If the last sensor */
			if ( fRelAngle >= m_fLightSensorDir[(NUM_LIGHT_SENSORS - 1)] )
			{
				
				/* Calc distance between the angles */
				double Delta_j = fabs(fRelAngle - ( m_fLightSensorDir[0] + 2*M_PI) );
				double Delta_j1 = fabs(fRelAngle - m_fLightSensorDir[NUM_LIGHT_SENSORS-1]);
				
				lightMeasures[0] = ( fDistance * Delta_j1 ) / (Delta_j + Delta_j1);
				lightMeasures[(NUM_LIGHT_SENSORS -1)] = ( fDistance * Delta_j ) / (Delta_j + Delta_j1);
			}
			
			/* else If the first sensor */
			else if ( fRelAngle <= m_fLightSensorDir[0] )
			{

				/* Calc distance between the angles */
				double Delta_j = fabs(fRelAngle - m_fLightSensorDir[0] );
				double Delta_j1 = fabs(fRelAngle - ( m_fLightSensorDir[NUM_LIGHT_SENSORS-1] - 2*M_PI) );
				
				lightMeasures[0] = ( fDistance * Delta_j1 ) / (Delta_j + Delta_j1);
				lightMeasures[(NUM_LIGHT_SENSORS -1)] = ( fDistance * Delta_j ) / (Delta_j + Delta_j1);

			}

			/* Any other sensor */
			else
			{
				int j;
				/* Check if in between of 2 sensors */
				for (j = 0 ; j < NUM_LIGHT_SENSORS ; j++)
				{
					if ( fRelAngle < m_fLightSensorDir[j] )
						break;
				}

				/* Calc distance between the angles */
				double Delta_j = fabs(fRelAngle - m_fLightSensorDir[j]);
				double Delta_j1 = fabs(fRelAngle - m_fLightSensorDir[j-1]);

				lightMeasures[j] = ( fDistance * Delta_j1 ) / (Delta_j + Delta_j1 );
				lightMeasures[j-1] = ( fDistance * Delta_j ) / (Delta_j + Delta_j1);
			}
		}
	}

	for ( int i = 0 ; i < NUM_LIGHT_SENSORS ; i++)
	{
		SetInput(i,lightMeasures[i]);
	}

	delete [] lightMeasures;
	return GetInputs();
}


/******************************************************************************/
/******************************************************************************/

unsigned int CRedLightSensor::GetType(){
	return SENSOR_RED_LIGHT;
}

/******************************************************************************/
/******************************************************************************/

double* CRedLightSensor::GetSensorReading( CEpuck *p_pcEpuck){

	return GetInputs();
}

/******************************************************************************/
/******************************************************************************/

double CRedLightSensor::GetMaxRange ( void ) 
{
	return m_fRangeLightSensor; 
}

/******************************************************************************/
/******************************************************************************/

const double* CRedLightSensor::GetSensorDirections()
{
    return m_fLightSensorDir;
}

/******************************************************************************/
/******************************************************************************/
void CRedLightSensor::SwitchNearestLight ( int n_value )
{
	dVector2 vPosition;
	vPosition.x = (m_pcEpuck->GetPosition()).x; 
	vPosition.y = (m_pcEpuck->GetPosition()).y;

	m_pcArena->SwitchNearestRedLight(vPosition, n_value);
}

/******************************************************************************/
/******************************************************************************/
