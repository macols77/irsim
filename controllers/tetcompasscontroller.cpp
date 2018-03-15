
/******************* INCLUDES ******************/
/***********************************************/

/******************** General ******************/
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <sys/time.h>
#include <iostream>

/******************** Simulator ****************/
/******************** Sensors ******************/
#include "encodersensor.h"
#include "compasssensor.h"

/******************** Actuators ****************/
#include "wheelsactuator.h"

/******************** Controller **************/
#include "testcompasscontroller.h"



extern gsl_rng* rng;
extern long int rngSeed;

using namespace std;

CTestCompassController::CTestCompassController (const char* pch_name, CEpuck* pc_epuck) : CController (pch_name, pc_epuck)

{
	/* Set epuck */
	m_pcEpuck = pc_epuck;
	/* Set Wheels */
	m_acWheels = (CWheelsActuator*) m_pcEpuck->GetActuator(ACTUATOR_WHEELS);
	/* Set encoder Sensor */
	m_seEncoder = (CEncoderSensor*) m_pcEpuck->GetSensor(SENSOR_ENCODER);
  m_seEncoder->InitEncoderSensor(m_pcEpuck);
	/* Set compass Sensor */
	m_seCompass = (CCompassSensor*) m_pcEpuck->GetSensor(SENSOR_COMPASS);

  /* Init variables to use for odometry calculation */
  m_fOrientation = 0.0;
  m_vPosition.x = 0.0;
  m_vPosition.y = 0.0;

  maxErrorA.x = 0.0;
  maxErrorA.y = 0.0;
}

/******************************************************************************/
/******************************************************************************/

CTestCompassController::~CTestCompassController()
{
}


/******************************************************************************/
/******************************************************************************/

void CTestCompassController::SimulationStep(unsigned n_step_number, double f_time, double f_step_interval)
{

	/* Read Encoder */
	double* encoder = m_seEncoder->GetSensorReading(m_pcEpuck);
	/* Read Compass */
	double* compass = m_seCompass->GetSensorReading(m_pcEpuck);
  /* Get distance between wheels */
  double fWheelsDistance = CEpuck::WHEELS_DISTANCE;

  /* STUDENTS MUST IMPLEMENT THE CODE HERE - START 
   * IMPORTANT!!: The code must exclusively depend of "encoder" variable */
  m_vPosition.x = 0.0;
  m_vPosition.y = 0.0;
  m_fOrientation  = 0.0;
  
  /* STUDENTS MUST IMPLEMENT THE CODE HERE - END*/
  
  /* Move Robot */
  m_acWheels->SetSpeed(500,500);
  
  /* DEBUG */ 
  printf("REAL: %2f,%2f,%2f  -- ODOM: %2f,%2f,%2f -- ENC: %2f,%2f \n", (m_pcEpuck->GetPosition()).x, (m_pcEpuck->GetPosition()).y, compass[0], m_vPosition.x,m_vPosition.y,m_fOrientation,encoder[0], encoder[1]);
  
  if ( fabs((m_pcEpuck->GetPosition()).x - m_vPosition.x ) > maxErrorA.x )
    maxErrorA.x = fabs((m_pcEpuck->GetPosition()).x - m_vPosition.x );

  if ( fabs((m_pcEpuck->GetPosition()).y - m_vPosition.y ) > maxErrorA.y )
    maxErrorA.y = fabs((m_pcEpuck->GetPosition()).y - m_vPosition.y );

  printf("ERROR: %2f,%2f,%2f\n", maxErrorA.x, maxErrorA.y, compass[0] - m_fOrientation);
  /* DEBUG */ 
 



}

/******************************************************************************/
/******************************************************************************/

