
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

#include "testbatteryexp.h"

/******************** Sensors ******************/
#include "contactsensor.h"
#include "epuckproximitysensor.h"
#include "reallightsensor.h"
#include "groundsensor.h"
#include "groundmemorysensor.h"
#include "batterysensor.h"

/******************** Actuators ****************/
#include "wheelsactuator.h"

/******************** Controllers **************/
#include "testbatterycontroller.h"

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
CTestBatteryExp::CTestBatteryExp(const char* pch_name, const char* paramsFile) :
	CExperiment(pch_name, COLLISION_MODEL_SIMPLE, COLLISION_HANDLER_POSITION)
{

	/* If there is not a parameter file input get default values*/
	if (paramsFile == NULL )
	{
		m_nRobotsNumber = 1;
		SetNumberOfEpucks(m_nRobotsNumber);

		m_fLightSensorRange = 1.0; //1 meter

		m_nLightObjectNumber = 1;
		m_pcvLightObjects = new dVector2[m_nLightObjectNumber];
		for ( int i = 0 ; i < m_nLightObjectNumber; i++){
			m_pcvLightObjects[i].x = 0.25;
			m_pcvLightObjects[i].y = 0.25;
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

CTestBatteryExp::~CTestBatteryExp ( void )
{
}

	/******************************************************************************/
/******************************************************************************/
CArena* CTestBatteryExp::CreateArena()
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

void CTestBatteryExp::AddActuators(CEpuck* pc_epuck)
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

void CTestBatteryExp::AddSensors(CEpuck* pc_epuck)
{
	//
	/* Create and add Proximity Sensor */
	CSensor* pcProxSensor = NULL;
	pcProxSensor = new CEpuckProximitySensor(252);
	pc_epuck->AddSensor(pcProxSensor);

	//Light Sensor
	CSensor* pcLightSensor = NULL;
	pcLightSensor = new CRealLightSensor("Light Sensor", m_fLightSensorRange);
	pc_epuck->AddSensor(pcLightSensor);
	
	//Contact Sensor
	CSensor* pcContactSensor = NULL;
	pcContactSensor = new CContactSensor("Contact Sensor");
	pc_epuck->AddSensor(pcContactSensor);
	
	//Ground Sensor
	CSensor* pcGroundSensor = NULL;
	pcGroundSensor = new CGroundSensor("Ground Sensor");
	pc_epuck->AddSensor(pcGroundSensor);
	
	//Ground Memory Sensor
	CSensor* pcGroundMemorySensor = NULL;
	pcGroundMemorySensor = new CGroundMemorySensor("Ground Memory Sensor");
	pc_epuck->AddSensor(pcGroundMemorySensor);
	
	//Battery Sensor
	CSensor* pcBatterySensor = NULL;
	pcBatterySensor = new CBatterySensor("Battery Sensor", 0.5, 0.01, 0.01);
	pc_epuck->AddSensor(pcBatterySensor);
}

/******************************************************************************/
/******************************************************************************/

void CTestBatteryExp::SetController(CEpuck* pc_epuck)
{
	char pchTemp[128];
	sprintf(pchTemp, "Iri1");
	CController* pcController = new CTestBatteryController(pchTemp, pc_epuck);
	pc_epuck->SetControllerType( CONTROLLER_TEST_BATTERY );
	pc_epuck->SetController(pcController);

}

/******************************************************************************/
/******************************************************************************/

void CTestBatteryExp::CreateAndAddEpucks(CSimulator* pc_simulator)
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

void CTestBatteryExp::Reset ( void )
{
}
