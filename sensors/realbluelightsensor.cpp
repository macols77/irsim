#include "realbluelightsensor.h"

/******************************************************************************/
/******************************************************************************/

double CRealBlueLightSensor::m_fLightSensorDir[NUM_REAL_BLUE_LIGHT_SENSORS] 	= {0.2967, 0.8727, 1.5708, 2.6180, 3.6652, 4.7124, 5.4105, 5.9865};
unsigned int CRealBlueLightSensor::SENSOR_NUMBER = NUM_REAL_BLUE_LIGHT_SENSORS;
double BLUE_LIGHT_SENSOR_APERTURE_ANGLE = 60 * M_PI / 180;

/******************************************************************************/
/******************************************************************************/

CRealBlueLightSensor::CRealBlueLightSensor(const char* pch_name, double f_rangeLightSensor) :
    CSensor(pch_name, NUM_REAL_BLUE_LIGHT_SENSORS )
{
	m_fRangeLightSensor = f_rangeLightSensor;
}

/******************************************************************************/
/******************************************************************************/

CRealBlueLightSensor::~CRealBlueLightSensor()
{
}

/******************************************************************************/
/******************************************************************************/

double* CRealBlueLightSensor::ComputeSensorReadings(CEpuck* pc_epuck, CSimulator* pc_simulator)
{
  double * lightMeasures = new double[NUM_REAL_BLUE_LIGHT_SENSORS];
  
	/* Update Sensor array with null values*/
  for ( int i = 0 ; i < NUM_REAL_BLUE_LIGHT_SENSORS ; i++)
  {
    lightMeasures[i] = 0.0;
  }

	CArena* pcArena = pc_simulator->GetArena();
  m_pcArena = pc_simulator->GetArena();

  m_pcEpuck = pc_epuck;

	/* Get Actual Position of the robot */
  dVector2 vPosition;
  vPosition.x = (pc_epuck->GetPosition()).x; 
  vPosition.y = (pc_epuck->GetPosition()).y;
  
  /* Get Actual Orientation of the robot */
  double fOrientation = pc_epuck->GetRotation();
    
  /* Update Sensor array */
  for ( int i = 0 ; i < NUM_REAL_BLUE_LIGHT_SENSORS ; i ++)
  {
	  dVector2 vLightPosition;
	  double fDistance;
	  double fRelativeAngle =0.0;
	  if ( pcArena->GetNearestBlueLightInSector(vPosition, NormalizeAngle(fOrientation + m_fLightSensorDir[i]), BLUE_LIGHT_SENSOR_APERTURE_ANGLE, m_fRangeLightSensor, &vLightPosition, &fDistance, &fRelativeAngle))
	  {
		  /* Normalize with Distance */
		  lightMeasures[i] = ( m_fRangeLightSensor - fDistance ) / m_fRangeLightSensor;
		  
			/* Regulate with angle */
			lightMeasures[i] *= 1 - fabs(fRelativeAngle)/BLUE_LIGHT_SENSOR_APERTURE_ANGLE;
	  }
  } 

  for ( int i = 0 ; i < NUM_REAL_BLUE_LIGHT_SENSORS ; i++)
  {
	  SetInput(i,lightMeasures[i]);
  }

  delete [] lightMeasures;


  /* Used for the ON/OFF function */
  return GetInputs();
}


/******************************************************************************/
/******************************************************************************/

unsigned int CRealBlueLightSensor::GetType(){
	return SENSOR_REAL_BLUE_LIGHT;
}

/******************************************************************************/
/******************************************************************************/

double* CRealBlueLightSensor::GetSensorReading( CEpuck *p_pcEpuck){

	return GetInputs();
}

/******************************************************************************/
/******************************************************************************/

double CRealBlueLightSensor::GetMaxRange ( void ) 
{
	return m_fRangeLightSensor; 
}

/******************************************************************************/
/******************************************************************************/

const double* CRealBlueLightSensor::GetSensorDirections()
{
    return m_fLightSensorDir;
}


/******************************************************************************/
/******************************************************************************/
void CRealBlueLightSensor::SwitchNearestLight ( int n_value )
{
	dVector2 vPosition;
	vPosition.x = (m_pcEpuck->GetPosition()).x; 
	vPosition.y = (m_pcEpuck->GetPosition()).y;

	m_pcArena->SwitchNearestBlueLight(vPosition, n_value);
}

/******************************************************************************/
/******************************************************************************/
