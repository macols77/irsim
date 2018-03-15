

/******************* INCLUDES ******************/
/***********************************************/

/******************** General ******************/
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <sys/time.h>


/******************** Simulator ****************/
#include <vector>
#include "simmath.h"
#include "testredlightexp.h"
#include "programmedarena.h"


/******************** Sensors ******************/
#include "realredlightsensor.h"

/******************** Actuators ****************/
#include "wheelsactuator.h"

/******************** Controllers **************/
#include "testredlightcontroller.h"



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

CTestRedLightExp::CTestRedLightExp(const char* pch_name, const char* paramsFile) :
	CExperiment(pch_name, COLLISION_MODEL_SIMPLE, COLLISION_HANDLER_POSITION)
{

	/* If there is not a parameter file input get default values*/
	if (paramsFile == NULL )
	{
		m_nRobotsNumber = 1;
		SetNumberOfEpucks(m_nRobotsNumber);

		m_nLightObjectNumber = 1;
		m_pcvLightObjects = new dVector2[m_nLightObjectNumber];
		for ( int i = 0 ; i < m_nLightObjectNumber; i++){
			m_pcvLightObjects[i].x = 0.0;
			m_pcvLightObjects[i].y = 0.0;
		}
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

CArena* CTestRedLightExp::CreateArena()
{
	/* Create Arena */
	CArena* pcArena = NULL;
	pcArena = new CProgrammedArena("CProgrammedArena", 20, 20, 3.0, 3.0);
	((CProgrammedArena*)pcArena)->SetHeightPixelsFromChars(pchHeightMap, ' ', '#', '%');

	/* Create and add Light Object */
	char pchTemp[128];
	CLightObject* pcLightObject = NULL;
	for( int i = 0 ; i < m_nLightObjectNumber ; i++){
		sprintf(pchTemp, "RedLightObject%d", i);
		CRedLightObject* pcRedLightObject = new CRedLightObject (pchTemp);
		pcRedLightObject->SetCenter(m_pcvLightObjects[i]);
		pcArena->AddRedLightObject(pcRedLightObject);
	}

	return pcArena;
}

/******************************************************************************/
/******************************************************************************/

void CTestRedLightExp::AddActuators(CEpuck* pc_epuck)
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

void CTestRedLightExp::AddSensors(CEpuck* pc_epuck)
{

	/* Create and add Light Sensor */
	CSensor* pcRedLightSensor = NULL;
	pcRedLightSensor = new CRealRedLightSensor("Red Light Sensor", 1);
	pc_epuck->AddSensor(pcRedLightSensor);
}

/******************************************************************************/
/******************************************************************************/

void CTestRedLightExp::SetController(CEpuck* pc_epuck)
{
	/* Create and add test red light controller */
	char pchTemp[128];
	sprintf(pchTemp, "testredlight");
	CController* pcController = new CTestRedLightController(pchTemp, pc_epuck);
	pc_epuck->SetControllerType( CONTROLLER_TEST_RED_LIGHT );
	pc_epuck->SetController(pcController);
}

/******************************************************************************/
/******************************************************************************/

void CTestRedLightExp::CreateAndAddEpucks(CSimulator* pc_simulator)
{
	/* Create and add epucks */
	char label[100] = "epuck";    
	for (int i = 0; i < m_nRobotsNumber; i++)
	{
		sprintf(label, "epuck%0.4d", i);
		CEpuck* pcEpuck = CreateEpuck(label, 0.5, 0.5, 0.0);
		pc_simulator->AddEpuck(pcEpuck);
	}
}


/******************************************************************************/
/******************************************************************************/

