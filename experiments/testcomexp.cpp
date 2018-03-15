

/******************* INCLUDES ******************/
/***********************************************/

/******************** General ******************/
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <sys/time.h>


/******************** Simulator ****************/
#include <vector>
#include "simmath.h"
#include "testcomexp.h"
#include "programmedarena.h"


/******************** Sensors ******************/
#include "comsensor.h"

/******************** Actuators ****************/
#include "wheelsactuator.h"

/******************** Controllers **************/
#include "testcomcontroller.h"



using namespace std;

/* Create Area */
static char* pchHeightMap = 
"%%%%%%%%%%%%%%%%%%%%"
"%##################%"
"%##################%"
"%##################%"
"%##################%"
"%##################%"
"%##################%"
"%##################%"
"%##################%"
"%##################%"
"%##################%"
"%##################%"
"%##################%"
"%##################%"
"%##################%"
"%##################%"    
"%##################%"
"%##################%"
"%##################%"
"%%%%%%%%%%%%%%%%%%%%";


extern gsl_rng* rng;
extern long int rngSeed;

/*******************************************************************************/
/*******************************************************************************/

CTestComExp::CTestComExp(const char* pch_name, const char* paramsFile) :
	CExperiment(pch_name, COLLISION_MODEL_SIMPLE, COLLISION_HANDLER_POSITION)
{

	/* If there is not a parameter file input get default values*/
	m_nRobotsNumber = 2;
	SetNumberOfEpucks(m_nRobotsNumber);
}

/******************************************************************************/
/******************************************************************************/

CArena* CTestComExp::CreateArena()
{
	/* Create Arena */
	CArena* pcArena = NULL;
	pcArena = new CProgrammedArena("CProgrammedArena", 20, 20, 3.0, 3.0);
	((CProgrammedArena*)pcArena)->SetHeightPixelsFromChars(pchHeightMap, ' ', '#', '%');

	return pcArena;
}

/******************************************************************************/
/******************************************************************************/

void CTestComExp::AddActuators(CEpuck* pc_epuck)
{
	/* Create and Add Wheels */
	char pchTemp[128];
	sprintf(pchTemp, "wheels_%s", pc_epuck->GetName());
	CActuator* pcActuator = NULL;
	pcActuator = new CWheelsActuator(pchTemp, pc_epuck);
	pc_epuck->AddActuator(pcActuator);
}

/******************************************************************************/
/******************************************************************************/

void CTestComExp::AddSensors(CEpuck* pc_epuck)
{

	/* Create and add Light Sensor */
	CSensor* pcComSensor = NULL;
	pcComSensor = new CComSensor("Com Sensor");
	pc_epuck->AddSensor(pcComSensor);
}

/******************************************************************************/
/******************************************************************************/

void CTestComExp::SetController(CEpuck* pc_epuck)
{
	/* Create and add test light controller */
	char pchTemp[128];
	sprintf(pchTemp, "testcom");
	CController* pcController = new CTestComController(pchTemp, pc_epuck);
	pc_epuck->SetControllerType( CONTROLLER_TEST_COM );
	pc_epuck->SetController(pcController);
}

/******************************************************************************/
/******************************************************************************/

void CTestComExp::CreateAndAddEpucks(CSimulator* pc_simulator)
{
	/* Create and add epucks */
	char label[100] = "epuck";    
	for (int i = 0; i < m_nRobotsNumber; i++)
	{
		sprintf(label, "epuck%0.4d", i);
		CEpuck* pcEpuck = CreateEpuck(label, Random::nextDouble(), Random::nextDouble(), Random::nextDouble() * 2 * M_PI);
		pc_simulator->AddEpuck(pcEpuck);
	}
}


/******************************************************************************/
/******************************************************************************/

