
/******************* INCLUDES ******************/
/***********************************************/

/******************** General ******************/

#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <sys/time.h>

/******************** Simulator ****************/
#include <vector>
#include "simmath.h"
#include "testcontactexp.h"
#include "programmedarena.h"

/******************** Sensors ******************/
#include "contactsensor.h"

/******************** Actuators ****************/
#include "wheelsactuator.h"

/******************** Controllers **************/
#include "testcontactcontroller.h"


using namespace std;

/*Create Arena */
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

CTestContactExp::CTestContactExp(const char* pch_name, const char* paramsFile) :
	CExperiment(pch_name, COLLISION_MODEL_SIMPLE, COLLISION_HANDLER_POSITION)
{

	/* If there is not a parameter file input get default values*/
	if (paramsFile == NULL )
	{
		m_nRobotsNumber = 1;
		SetNumberOfEpucks(m_nRobotsNumber);
	}
	/* Else, extract info from the file */
	/* (NOTE: STILL NOT IMPLEMENTED */
	else{
		/* I SHOULD WORK ON THIS */
		m_nRobotsNumber = 1;
		SetNumberOfEpucks(m_nRobotsNumber);
	}
}

/******************************************************************************/
/******************************************************************************/

CArena* CTestContactExp::CreateArena()
{
	/* Create Arena */
	CArena* pcArena = NULL;
	pcArena = new CProgrammedArena("CProgrammedArena", 20, 20, 3.0, 3.0);
	((CProgrammedArena*)pcArena)->SetHeightPixelsFromChars(pchHeightMap, ' ', '#', '%');

	return pcArena;
}

/******************************************************************************/
/******************************************************************************/

void CTestContactExp::AddActuators(CEpuck* pc_epuck)
{
	/* Create and Add Wheels */
	char pchTemp[128];
	sprintf(pchTemp, "actuators_%s", pc_epuck->GetName());
	CActuator* pcActuator = NULL;
	pcActuator = new CWheelsActuator(pchTemp, pc_epuck);
	pc_epuck->AddActuator(pcActuator);
}

/******************************************************************************/
/******************************************************************************/

void CTestContactExp::AddSensors(CEpuck* pc_epuck)
{

	/* Create and add Contact Sensor */
	CSensor* pcContactSensor = NULL;
	pcContactSensor = new CContactSensor("Contact Sensor");
	pc_epuck->AddSensor(pcContactSensor);

}

/******************************************************************************/
/******************************************************************************/

void CTestContactExp::SetController(CEpuck* pc_epuck)
{
	/* Create and add test light controller */
	char pchTemp[128];
	sprintf(pchTemp, "testcontact");
	CController* pcController = new CTestContactController(pchTemp, pc_epuck);
	pc_epuck->SetControllerType( CONTROLLER_TEST_CONTACT );
	pc_epuck->SetController(pcController);
}

/******************************************************************************/
/******************************************************************************/

void CTestContactExp::CreateAndAddEpucks(CSimulator* pc_simulator)
{
	/* Create and add epucks */
	char label[100] = "epuck";    
	for (int i = 0; i < m_nRobotsNumber; i++)
	{
		sprintf(label, "epuck%0.4d", i);
		CEpuck* pcEpuck = CreateEpuck(label, 0, 0, 0.0);
		pc_simulator->AddEpuck(pcEpuck);
	}
}


/******************************************************************************/
/******************************************************************************/

