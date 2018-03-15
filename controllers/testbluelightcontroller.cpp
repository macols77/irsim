
/******************* INCLUDES ******************/
/***********************************************/

/******************** General ******************/
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <sys/time.h>
#include <iostream>

/******************** Simulator ****************/
/******************** Sensors ******************/
#include "realbluelightsensor.h"

/******************** Actuators ****************/
#include "wheelsactuator.h"

/******************** Controller **************/
#include "testbluelightcontroller.h"



extern gsl_rng* rng;
extern long int rngSeed;

using namespace std;

CTestBlueLightController::CTestBlueLightController (const char* pch_name, CEpuck* pc_epuck) : CController (pch_name, pc_epuck)

{
	/* Set epuck */
	m_pcEpuck = pc_epuck;
	/* Set Wheels */
	m_acWheels = (CWheelsActuator*) m_pcEpuck->GetActuator(ACTUATOR_WHEELS);
	/* Set light Sensor */
	m_seLight = (CRealBlueLightSensor*) m_pcEpuck->GetSensor(SENSOR_REAL_BLUE_LIGHT);
}

/******************************************************************************/
/******************************************************************************/

CTestBlueLightController::~CTestBlueLightController()
{
}


/******************************************************************************/
/******************************************************************************/

void CTestBlueLightController::SimulationStep(unsigned n_step_number, double f_time, double f_step_interval)
{

	double* light = m_seLight->GetSensorReading(m_pcEpuck);

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

