
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
#include "testlightcontroller.h"



extern gsl_rng* rng;
extern long int rngSeed;

using namespace std;

CTestLightController::CTestLightController (const char* pch_name, CEpuck* pc_epuck) : CController (pch_name, pc_epuck)

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

CTestLightController::~CTestLightController()
{
}


/******************************************************************************/
/******************************************************************************/

void CTestLightController::SimulationStep(unsigned n_step_number, double f_time, double f_step_interval)
{

	double* light = m_seLight->GetSensorReading(m_pcEpuck);

	printf("Light Sensor Value: ");
	for ( int i = 0 ; i < m_seLight->GetNumberOfInputs() ; i++)
	{
		printf("%2f ", light[i]);
	}
	printf("\n");

	/* Option 1: Speed between -1000, 1000*/ 
	//m_acWheels->SetSpeed(500,500);

	/* Option 2: Speed between 0,1*/
	m_acWheels->SetOutput(0,0.25);
	m_acWheels->SetOutput(1,0.75);


}

/******************************************************************************/
/******************************************************************************/

