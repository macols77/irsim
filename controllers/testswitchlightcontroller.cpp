
/******************* INCLUDES ******************/
/***********************************************/

/******************** General ******************/
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <sys/time.h>
#include <iostream>

/******************** Simulator ****************/
/******************** Sensors ******************/
#include "reallightsensor.h"

/******************** Actuators ****************/
#include "wheelsactuator.h"

/******************** Controller **************/
#include "testswitchlightcontroller.h"



extern gsl_rng* rng;
extern long int rngSeed;

using namespace std;

CTestSwitchLightController::CTestSwitchLightController (const char* pch_name, CEpuck* pc_epuck) : CController (pch_name, pc_epuck)

{
	/* Set epuck */
	m_pcEpuck = pc_epuck;
	/* Set Wheels */
	m_acWheels = (CWheelsActuator*) m_pcEpuck->GetActuator(ACTUATOR_WHEELS);
	/* Set light Sensor */
	m_seLight = (CRealLightSensor*) m_pcEpuck->GetSensor(SENSOR_REAL_LIGHT);
}

/******************************************************************************/
/******************************************************************************/

CTestSwitchLightController::~CTestSwitchLightController()
{
}


/******************************************************************************/
/******************************************************************************/

void CTestSwitchLightController::SimulationStep(unsigned n_step_number, double f_time, double f_step_interval)
{

	double* light = m_seLight->GetSensorReading(m_pcEpuck);

	printf("Light Sensor Value: ");
	double totalLight = 0;
  for ( int i = 0 ; i < m_seLight->GetNumberOfInputs() ; i++)
  {
    printf("%2f ", light[i]);
  }
  
  totalLight = light[0]+light[7];
	printf("-- %2f ", totalLight);
	printf("\n");

	if ( totalLight >= 0.9)
	{
		m_seLight->SwitchNearestLight(0);
	}

  //if ( totalLight == 0 )
  //m_seLight->SwitchNearestLight(1);
	/* GO Light */
	if ( light[0] * light[7] == 0.0 )
	{
		/* Calc light intensity at the left and right */
		double lightLeft 	= light[0] + light[1] + light[2] + light[3];
		double lightRight = light[4] + light[5] + light[6] + light[7];

		/* If light on the left */
		if ( lightLeft > lightRight )
		{
			/* Turn left */
			m_acWheels->SetSpeed(-500,500);
		}
		else
		{
			/* Turn right */
			m_acWheels->SetSpeed(500,-500);
		}
	}
	else
		m_acWheels->SetSpeed(500,500);
}

/******************************************************************************/
/******************************************************************************/

