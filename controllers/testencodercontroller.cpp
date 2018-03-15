
/******************* INCLUDES ******************/
/***********************************************/

/******************** General ******************/
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <sys/time.h>
#include <iostream>

/******************** Simulator ****************/
/******************** Sensors ******************/
#include "encodersensor.h"

/******************** Actuators ****************/
#include "wheelsactuator.h"

/******************** Controller **************/
#include "testencodercontroller.h"



extern gsl_rng* rng;
extern long int rngSeed;

using namespace std;

CTestEncoderController::CTestEncoderController (const char* pch_name, CEpuck* pc_epuck) : CController (pch_name, pc_epuck)

{
	/* Set epuck */
	m_pcEpuck = pc_epuck;
	/* Set Wheels */
	m_acWheels = (CWheelsActuator*) m_pcEpuck->GetActuator(ACTUATOR_WHEELS);
	/* Set light Sensor */
	m_seEncoder = (CEncoderSensor*) m_pcEpuck->GetSensor(SENSOR_ENCODER);
  m_seEncoder->InitEncoderSensor(m_pcEpuck);

  m_fOrientation = 0.0;
  m_vPosition.x = 0.0;
  m_vPosition.y = 0.0;
}

/******************************************************************************/
/******************************************************************************/

CTestEncoderController::~CTestEncoderController()
{
}


/******************************************************************************/
/******************************************************************************/

void CTestEncoderController::SimulationStep(unsigned n_step_number, double f_time, double f_step_interval)
{

	double* encoder = m_seEncoder->GetSensorReading(m_pcEpuck);

	printf("Encoder Sensor Value: ");
	for ( int i = 0 ; i < m_seEncoder->GetNumberOfInputs() ; i++)
	{
		printf("%2f ", encoder[i]);
	}
	printf("\n");
  
  printf("Robot Real Position: %2f,%2f\n", (m_pcEpuck->GetPosition()).x, (m_pcEpuck->GetPosition()).y);

	/* Option 2: Speed between 0,1*/
  //m_acWheels->SetOutput(0,0.75);
  //m_acWheels->SetOutput(1,0.75);
	m_acWheels->SetSpeed(-500,-500);


}

/******************************************************************************/
/******************************************************************************/

