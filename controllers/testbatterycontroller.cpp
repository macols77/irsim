/******************* INCLUDES ******************/
/***********************************************/

/******************** General ******************/
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <sys/time.h>
#include <iostream>

/******************** Simulator ****************/
/******************** Sensors ******************/
#include "epuckproximitysensor.h"
#include "contactsensor.h"
#include "reallightsensor.h"
#include "groundsensor.h"
#include "groundmemorysensor.h"
#include "batterysensor.h"

/******************** Actuators ****************/
#include "wheelsactuator.h"

/******************** Controller **************/
#include "testbatterycontroller.h"

extern gsl_rng* rng;
extern long int rngSeed;

using namespace std;

CTestBatteryController::CTestBatteryController (const char* pch_name, CEpuck* pc_epuck) : CController (pch_name, pc_epuck)

{
	/* Set epuck */
	m_pcEpuck = pc_epuck;
	/* Set Wheels */
	m_acWheels = (CWheelsActuator*) m_pcEpuck->GetActuator(ACTUATOR_WHEELS);
	/* Set Prox Sensor */
	m_seProx = (CEpuckProximitySensor*) m_pcEpuck->GetSensor(SENSOR_PROXIMITY);
	/* Set light Sensor */
	m_seLight = (CRealLightSensor*) m_pcEpuck->GetSensor(SENSOR_REAL_LIGHT);
	/* Set contact Sensor */
	m_seContact = (CContactSensor*) m_pcEpuck->GetSensor (SENSOR_CONTACT);
	/* Set ground Sensor */
	m_seGround = (CGroundSensor*) m_pcEpuck->GetSensor (SENSOR_GROUND);
	/* Set ground memory Sensor */
	m_seGroundMemory = (CGroundMemorySensor*) m_pcEpuck->GetSensor (SENSOR_GROUND_MEMORY);
	/* Set battery Sensor */
	m_seBattery = (CBatterySensor*) m_pcEpuck->GetSensor (SENSOR_BATTERY);

	m_dStatus = 0;
}

/******************************************************************************/
/******************************************************************************/

CTestBatteryController::~CTestBatteryController()
{
}


/******************************************************************************/
/******************************************************************************/

void CTestBatteryController::SimulationStep(unsigned n_step_number, double f_time, double f_step_interval)
{


	/* FASE 1: LECTURA DE SENSORES */

	/* Leer Battery Sensores de Suelo Memory */
	double* battery = m_seBattery->GetSensorReading(m_pcEpuck);
	
	/* FASE 2: CONTROLADOR */
	
	printf(" BATTERY: ");
	for ( int i = 0 ; i < m_seBattery->GetNumberOfInputs() ; i ++ )
	{
		printf("%1.3f ", battery[i]);
	}

	printf("\n");

	/* Fase 3: ACTUACIÓN */

	if ( m_dStatus == 0 & battery[0] < 0.5 )
		m_dStatus = 1;
	else if ( m_dStatus == 1 & battery[0] > 0.9 )
		m_dStatus = 0;

	if ( m_dStatus == 0 )
		m_acWheels->SetSpeed(200,200);
	else
		m_acWheels->SetSpeed(-500,-500);
}

/******************************************************************************/
/******************************************************************************/

