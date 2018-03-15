

/******************* INCLUDES ******************/
/***********************************************/

/******************** General ******************/
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <sys/time.h>


/******************** Simulator ****************/
#include <vector>
#include "simmath.h"
#include "testswitchlightexp.h"
#include "programmedarena.h"


/******************** Sensors ******************/
#include "reallightsensor.h"

/******************** Actuators ****************/
#include "wheelsactuator.h"

/******************** Controllers **************/
#include "testswitchlightcontroller.h"



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

CTestSwitchLightExp::CTestSwitchLightExp(const char* pch_name, const char* paramsFile) :
	CExperiment(pch_name, COLLISION_MODEL_SIMPLE, COLLISION_HANDLER_POSITION)
{

	/* If there is not a parameter file input get default values*/
	if (paramsFile == NULL )
	{
		m_nRobotsNumber = 1;
		SetNumberOfEpucks(m_nRobotsNumber);

		m_nLightObjectNumber = 4;
		m_pcvLightObjects = new dVector2[m_nLightObjectNumber];
		m_pcvLightObjects[0].x = 0.5;
		m_pcvLightObjects[0].y = 0.5;
		m_pcvLightObjects[1].x = -0.5;
		m_pcvLightObjects[1].y = -0.5;
		m_pcvLightObjects[2].x = -0.5;
		m_pcvLightObjects[2].y = 0.5;
		m_pcvLightObjects[3].x = 0.5;
		m_pcvLightObjects[3].y = -0.5;

	}
	/* Else, extract info from the file */
	/* (NOTE: STILL NOT IMPLEMENTED */
	else{
		/* I SHOULD WORK ON THIS */
		m_nRobotsNumber = 1;
		SetNumberOfEpucks(m_nRobotsNumber);

		m_nLightObjectNumber = 1;
		m_pcvLightObjects = new dVector2[m_nLightObjectNumber];
		for ( int i = 0 ; i < m_nLightObjectNumber; i++){
			m_pcvLightObjects[i].x = 0.0;
			m_pcvLightObjects[i].y = 0.0;
		}
	}
}

/******************************************************************************/
/******************************************************************************/

CArena* CTestSwitchLightExp::CreateArena()
{
	/* Create Arena */
	CArena* pcArena = NULL;
	pcArena = new CProgrammedArena("CProgrammedArena", 20, 20, 3.0, 3.0);
	((CProgrammedArena*)pcArena)->SetHeightPixelsFromChars(pchHeightMap, ' ', '#', '%');

	/* Create and add Light Object */
	char pchTemp[128];
	CLightObject* pcLightObject = NULL;
	for( int i = 0 ; i < m_nLightObjectNumber ; i++){
		sprintf(pchTemp, "LightObject%d", i);
		CLightObject* pcLightObject = new CLightObject (pchTemp);
		pcLightObject->SetCenter(m_pcvLightObjects[i]);
		pcArena->AddLightObject(pcLightObject);
	}

	return pcArena;
}

/******************************************************************************/
/******************************************************************************/

void CTestSwitchLightExp::AddActuators(CEpuck* pc_epuck)
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

void CTestSwitchLightExp::AddSensors(CEpuck* pc_epuck)
{

	/* Create and add Light Sensor */
	CSensor* pcLightSensor = NULL;
	pcLightSensor = new CRealLightSensor("Light Sensor", 3);
	pc_epuck->AddSensor(pcLightSensor);
}

/******************************************************************************/
/******************************************************************************/

void CTestSwitchLightExp::SetController(CEpuck* pc_epuck)
{
	/* Create and add test light controller */
	char pchTemp[128];
	sprintf(pchTemp, "testswitchlight");
	CController* pcController = new CTestSwitchLightController(pchTemp, pc_epuck);
	pc_epuck->SetControllerType( CONTROLLER_TEST_LIGHT );
	pc_epuck->SetController(pcController);
}

/******************************************************************************/
/******************************************************************************/

void CTestSwitchLightExp::CreateAndAddEpucks(CSimulator* pc_simulator)
{
	/* Create and add epucks */
	char label[100] = "epuck";    
	for (int i = 0; i < m_nRobotsNumber; i++)
	{
		sprintf(label, "epuck%0.4d", i);
		CEpuck* pcEpuck = CreateEpuck(label, 0.0, 0.0, 0.0);
		pc_simulator->AddEpuck(pcEpuck);
	}
}


/******************************************************************************/
/******************************************************************************/

