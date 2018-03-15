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
#include "lightsensor.h"
#include "groundsensor.h"
#include "groundmemorysensor.h"

/******************** Actuators ****************/
#include "wheelsactuator.h"

/******************** Controller **************/
#include "testgroundcontroller.h"

extern gsl_rng* rng;
extern long int rngSeed;

using namespace std;

CTestGroundController::CTestGroundController (const char* pch_name, CEpuck* pc_epuck) : CController (pch_name, pc_epuck)

{
	/* Set epuck */
	m_pcEpuck = pc_epuck;
	/* Set Wheels */
	m_acWheels = (CWheelsActuator*) m_pcEpuck->GetActuator(ACTUATOR_WHEELS);
	/* Set Prox Sensor */
	m_seProx = (CEpuckProximitySensor*) m_pcEpuck->GetSensor(SENSOR_PROXIMITY);
	/* Set light Sensor */
	m_seLight = (CLightSensor*) m_pcEpuck->GetSensor(SENSOR_LIGHT);
	/* Set contact Sensor */
	m_seContact = (CContactSensor*) m_pcEpuck->GetSensor (SENSOR_CONTACT);
	/* Set ground Sensor */
	m_seGround = (CGroundSensor*) m_pcEpuck->GetSensor (SENSOR_GROUND);
	/* Set ground memory Sensor */
	m_seGroundMemory = (CGroundMemorySensor*) m_pcEpuck->GetSensor (SENSOR_GROUND_MEMORY);
}

/******************************************************************************/
/******************************************************************************/

CTestGroundController::~CTestGroundController()
{
}


/******************************************************************************/
/******************************************************************************/

void CTestGroundController::SimulationStep(unsigned n_step_number, double f_time, double f_step_interval)
{


	/* FASE 1: LECTURA DE SENSORES */

	/* Leer Sensores de Contacto */
	//double* contact = m_seContact->GetSensorReading(m_pcEpuck);
	/* Leer Sensores de Proximidad */
	//double* prox = m_seProx->GetSensorReading(m_pcEpuck);
	/* Leer Sensores de Luz */
	//double* light = m_seLight->GetSensorReading(m_pcEpuck);
	/* Leer Sensores de Suelo */
	double* ground = m_seGround->GetSensorReading(m_pcEpuck);
	/* Leer Sensores de Suelo Memoery */
	double* groundMemory = m_seGroundMemory->GetSensorReading(m_pcEpuck);

	
	/* FASE 2: CONTROLADOR */
	
	/* Inicio Incluir las ACCIONES/CONTROLADOR a implementar */
	//printf("CONTACT: ");
	//for ( int i = 0 ; i < m_seContact->GetNumberOfInputs() ; i ++ )
	//{
		//printf("%1.3f ", contact[i]);
	//}
	
	//printf(" -- PROX: ");
	//for ( int i = 0 ; i < m_seProx->GetNumberOfInputs() ; i ++ )
	//{
		//printf("%1.3f ", prox[i]);
	//}

	printf(" GROUND: ");
	for ( int i = 0 ; i < m_seGround->GetNumberOfInputs() ; i ++ )
	{
		printf("%1.3f ", ground[i]);
	}
	
	printf(" GROUND MEMORY: ");
	for ( int i = 0 ; i < m_seGroundMemory->GetNumberOfInputs() ; i ++ )
	{
		printf("%1.3f ", groundMemory[i]);
	}

	printf("\n");
	
	/* Fin: Incluir las ACCIONES/CONTROLADOR a implementar */


	
	

	/* Fase 3: ACTUACIÓN */
	/* Option 1: Speed between -1000, 1000*/ 
	m_acWheels->SetSpeed(200,200);

	/* Option 2: Speed between 0,1*/
	//m_acWheels->SetOutput(0,0.5);
	//m_acWheels->SetOutput(1,0.5);
	
}

/******************************************************************************/
/******************************************************************************/

