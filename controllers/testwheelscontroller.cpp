
/******************* INCLUDES ******************/
/***********************************************/

/******************** General ******************/
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <sys/time.h>
#include <iostream>

/******************** Simulator ****************/
/******************** Sensors ******************/
/******************** Actuators ****************/
#include "wheelsactuator.h"

/******************** Controller **************/
#include "testwheelscontroller.h"



extern gsl_rng* rng;
extern long int rngSeed;

using namespace std;

CTestWheelsController::CTestWheelsController (const char* pch_name, CEpuck* pc_epuck) : CController (pch_name, pc_epuck)

{
	/* Set epuck */
	m_pcEpuck = pc_epuck;
	/* Set Wheels */
	m_acWheels = (CWheelsActuator*) m_pcEpuck->GetActuator(ACTUATOR_WHEELS);
}

/******************************************************************************/
/******************************************************************************/

CTestWheelsController::~CTestWheelsController()
{
}


/******************************************************************************/
/******************************************************************************/

void CTestWheelsController::SimulationStep(unsigned n_step_number, double f_time, double f_step_interval)
{

	/* Option 1: Speed between -1000, 1000*/ 
	m_acWheels->SetSpeed(1000,1000);

	/* Option 2: Speed between 0,1*/
	//m_acWheels->SetOutput(0,0.75);
	//m_acWheels->SetOutput(1,0.25);

}

/******************************************************************************/
/******************************************************************************/

