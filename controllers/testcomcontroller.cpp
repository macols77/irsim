
/******************* INCLUDES ******************/
/***********************************************/

/******************** General ******************/
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <sys/time.h>
#include <iostream>

/******************** Simulator ****************/
/******************** Sensors ******************/
#include "comsensor.h"

/******************** Actuators ****************/
#include "wheelsactuator.h"

/******************** Controller **************/
#include "testcomcontroller.h"



extern gsl_rng* rng;
extern long int rngSeed;

using namespace std;

CTestComController::CTestComController (const char* pch_name, CEpuck* pc_epuck) : CController (pch_name, pc_epuck)

{
	/* Set epuck */
	m_pcEpuck = pc_epuck;
	/* Set Wheels */
	m_acWheels = (CWheelsActuator*) m_pcEpuck->GetActuator(ACTUATOR_WHEELS);
	/* Set light Sensor */
	m_seCom = (CComSensor*) m_pcEpuck->GetSensor(SENSOR_COM);
	m_nComData = 0;
}

/******************************************************************************/
/******************************************************************************/

CTestComController::~CTestComController()
{
}


/******************************************************************************/
/******************************************************************************/

void CTestComController::SimulationStep(unsigned n_step_number, double f_time, double f_step_interval)
{

	int 		data		=	0;
	double 	range 	= 0;
	double 	bearing 	= 0;

	/* Read Neighbour Data */
	int neigh = m_seCom->GetNeighbourData(&data, &range, &bearing);
	/* Set own Data */
	m_seCom->SetData(m_nComData);
	m_nComData++;

	/* Get Robot Name */
	char label[10]="";
	m_pcEpuck->GetLabel(label);
	double posX, posY;
	m_pcEpuck->GetPosition(&posX, &posY);
	double orien = m_pcEpuck->GetRotation();
	//label[10]='0';
	/* Print Data */
	printf("%s", label);
	printf(" -- Me: Pos: %2f,%2f Orien: %2f ", posX, posY,orien);
	printf(" -- Neigh: %d Data: %d Range: %2f Bearing: %2f\n", neigh, data, range, bearing);

	/* Set Speed */
	m_acWheels->SetOutput(0,0.5);
	m_acWheels->SetOutput(1,0.5);


}

/******************************************************************************/
/******************************************************************************/

