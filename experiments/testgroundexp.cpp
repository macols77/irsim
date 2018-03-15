
/******************* INCLUDES ******************/
/***********************************************/

/******************** General ******************/

#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <sys/time.h>

/******************** Simulator ****************/
#include <vector>
#include "simmath.h"
#include "random.h"
#include "programmedarena.h"

#include "testgroundexp.h"

/******************** Sensors ******************/
#include "contactsensor.h"
#include "epuckproximitysensor.h"
#include "lightsensor.h"
#include "groundsensor.h"
#include "groundmemorysensor.h"

/******************** Actuators ****************/
#include "wheelsactuator.h"

/******************** Controllers **************/
#include "testgroundcontroller.h"

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
///*******************************************************************************/
//
CTestGroundExp::CTestGroundExp(const char* pch_name, const char* paramsFile) :
	CExperiment(pch_name, COLLISION_MODEL_SIMPLE, COLLISION_HANDLER_POSITION)
{

	/* If there is not a parameter file input get default values*/
	if (paramsFile == NULL )
	{
		m_nRobotsNumber = 1;
		SetNumberOfEpucks(m_nRobotsNumber);

		preyCenter = new dVector2[2];
		preyExternalRadius = new double[2];
		preyInternalRadius = new double[2];
		preyColor = new double[2];
		for ( int i = 0 ; i < 2 ; i++)
		{
			//Create Prey
			preyCenter[i].x = i+0.1; 
			preyCenter[i].y = i+0.1;
			preyInternalRadius[i] = 0.0;
			if ( i == 0 )
			{
				preyColor[i] = 0.5;
				preyExternalRadius[i] = 0.1;
			}
			else{
				preyColor[i] = 0.0;
				preyExternalRadius[i] = 0.5;
			}
		}


	}
	/* Else, extract info from the file */
	/* (NOTE: STILL NOT IMPLEMENTED */
	else
	{
		/* I SHOULD WORK ON THIS */
		m_nRobotsNumber = 1;
		SetNumberOfEpucks(m_nRobotsNumber);
		
		m_fLightSensorRange = 1.0; // 1 meter

		m_nLightObjectNumber = 1;
		m_pcvLightObjects = new dVector2[m_nLightObjectNumber];
		for ( int i = 0 ; i < m_nLightObjectNumber; i++){
			m_pcvLightObjects[i].x = 0.25;
			m_pcvLightObjects[i].y = 0.25;
		}
	}
}

/******************************************************************************/
/******************************************************************************/

CTestGroundExp::~CTestGroundExp ( void )
{
}

	/******************************************************************************/
/******************************************************************************/
CArena* CTestGroundExp::CreateArena()
{
	/* Create Arena */
	CArena* pcArena = NULL;
	pcArena = new CProgrammedArena("CProgrammedArena", 20, 20, 3.0, 3.0);
	((CProgrammedArena*)pcArena)->SetHeightPixelsFromChars(pchHeightMap, ' ', '#', '%');

	/* Create Prey */
	char preyName[100]="epuck";

	//dVector2 preyCenter;
	//preyCenter.x = 0.0;
	//preyCenter.y = 0.0;
	//double preyExternalRadius = 0.2;
	//double preyInternalRadius = 0.0;

	for ( int i = 0 ; i < 2 ; i++)
	{
		//Create Prey
		sprintf(preyName,"prey%d",i);
		CGroundArea* prey = new CGroundArea(preyName);
		prey->SetCenter(preyCenter[i]);
		prey->SetExtRadius(preyExternalRadius[i]);
		prey->SetIntRadius(preyInternalRadius[i]);
		prey->SetColor(preyColor[i]);
		prey->SetHeight(0.20);
		pcArena->AddGroundArea(prey);
	}

	return pcArena;
}

/******************************************************************************/
/******************************************************************************/

void CTestGroundExp::AddActuators(CEpuck* pc_epuck)
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

void CTestGroundExp::AddSensors(CEpuck* pc_epuck)
{
	//
	/* Create and add Proximity Sensor */
	CSensor* pcProxSensor = NULL;
	pcProxSensor = new CEpuckProximitySensor(252);
	pc_epuck->AddSensor(pcProxSensor);

	//Light Sensor
	CSensor* pcLightSensor = NULL;
	pcLightSensor = new CLightSensor("Light Sensor", m_fLightSensorRange);
	pc_epuck->AddSensor(pcLightSensor);
	
	//Contact Sensor
	CSensor* pcContactSensor = NULL;
	pcContactSensor = new CContactSensor("Contact Sensor");
	pc_epuck->AddSensor(pcContactSensor);
	
	//Ground Sensor
	CSensor* pcGroundSensor = NULL;
	pcGroundSensor = new CGroundSensor("Ground Sensor");
	pc_epuck->AddSensor(pcGroundSensor);
	
	//Ground iMemory Sensor
	CSensor* pcGroundMemorySensor = NULL;
	pcGroundMemorySensor = new CGroundMemorySensor("Ground Memory Sensor");
	pc_epuck->AddSensor(pcGroundMemorySensor);
}

/******************************************************************************/
/******************************************************************************/

void CTestGroundExp::SetController(CEpuck* pc_epuck)
{
	char pchTemp[128];
	sprintf(pchTemp, "Iri1");
	CController* pcController = new CTestGroundController(pchTemp, pc_epuck);
	pc_epuck->SetControllerType( CONTROLLER_TEST_GROUND );
	pc_epuck->SetController(pcController);

}

/******************************************************************************/
/******************************************************************************/

void CTestGroundExp::CreateAndAddEpucks(CSimulator* pc_simulator)
{
	/* Create and add epucks */
	char label[100] = "epuck";    
	for (int i = 0; i < m_nRobotsNumber; i++)
	{
		sprintf(label, "epuck%0.4d", i);
		CEpuck* pcEpuck = CreateEpuck(label, 0.0, 0.0, M_PI/4);
		pc_simulator->AddEpuck(pcEpuck);
	}

	Reset();
}


/******************************************************************************/
/******************************************************************************/

void CTestGroundExp::Reset ( void )
{
}
