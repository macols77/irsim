/******************* INCLUDES ******************/
/***********************************************/

/******************** General ******************/
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <sys/time.h>
#include <iostream>
#include <math.h> 

/******************** Simulator ****************/
/******************** Sensors ******************/
#include "epuckproximitysensor.h"
#include "contactsensor.h"
#include "reallightsensor.h"
#include "realbluelightsensor.h"
#include "realredlightsensor.h"
#include "groundsensor.h"
#include "groundmemorysensor.h"
#include "batterysensor.h"
#include "bluebatterysensor.h"
#include "redbatterysensor.h"
#include "encodersensor.h"
#include "compasssensor.h"

/******************** Actuators ****************/
#include "wheelsactuator.h"

/******************** Controller **************/
#include "braitenbergvehicle2controller.h"

extern gsl_rng* rng;
extern long int rngSeed;

using namespace std;

CBraitenbergVehicle2Controller::CBraitenbergVehicle2Controller (const char* pch_name, CEpuck* pc_epuck, int n_write_to_file) : CController (pch_name, pc_epuck)

{
	/* Set Write to File */
	m_nWriteToFile = n_write_to_file;

	/* Set epuck */
	m_pcEpuck = pc_epuck;
	/* Set Wheels */
	m_acWheels = (CWheelsActuator*) m_pcEpuck->GetActuator(ACTUATOR_WHEELS);
	/* Set Prox Sensor */
	m_seProx = (CEpuckProximitySensor*) m_pcEpuck->GetSensor(SENSOR_PROXIMITY);
	/* Set light Sensor */
	m_seLight = (CRealLightSensor*) m_pcEpuck->GetSensor(SENSOR_REAL_LIGHT);
	/* Set Blue light Sensor */
	m_seBlueLight = (CRealBlueLightSensor*) m_pcEpuck->GetSensor(SENSOR_REAL_BLUE_LIGHT);
	/* Set Red light Sensor */
	m_seRedLight = (CRealRedLightSensor*) m_pcEpuck->GetSensor(SENSOR_REAL_RED_LIGHT);
	/* Set contact Sensor */
	m_seContact = (CContactSensor*) m_pcEpuck->GetSensor (SENSOR_CONTACT);
	/* Set ground Sensor */
	m_seGround = (CGroundSensor*) m_pcEpuck->GetSensor (SENSOR_GROUND);
	/* Set ground memory Sensor */
	m_seGroundMemory = (CGroundMemorySensor*) m_pcEpuck->GetSensor (SENSOR_GROUND_MEMORY);
	/* Set battery Sensor */
	m_seBattery = (CBatterySensor*) m_pcEpuck->GetSensor (SENSOR_BATTERY);
	/* Set blue battery Sensor */
	m_seBlueBattery = (CBlueBatterySensor*) m_pcEpuck->GetSensor (SENSOR_BLUE_BATTERY);
	/* Set red battery Sensor */
	m_seRedBattery = (CRedBatterySensor*) m_pcEpuck->GetSensor (SENSOR_RED_BATTERY);
}

/******************************************************************************/
/******************************************************************************/

CBraitenbergVehicle2Controller::~CBraitenbergVehicle2Controller()
{
}


/******************************************************************************/
/******************************************************************************/

void CBraitenbergVehicle2Controller::SimulationStep(unsigned n_step_number, double f_time, double f_step_interval)
{
	/* Read Light */
	double* light = m_seLight->GetSensorReading(m_pcEpuck);

  /* Print Light */	
  printf("LIGHT: ");
  for ( int i = 0 ; i < m_seLight->GetNumberOfInputs() ; i ++ )
  {
    printf("%1.3f ", light[i]);
  }
  printf ("\n");

  /* Define speed for left and right wheel */
  double speed[2];
  
  /* STUDENTS MUST IMPLEMENT THE CODE HERE - START 
   * IMPORTANT!!: The code must exclusively depend on "light" variable */

  // 
  double radius = 0.35;
  speed[0] = light[0] + light[1] + light[2] + light[3] + light[4] + light[5] + light[6] + light[7] - radius;
  speed[1] = pow(speed[0], 3);
  
  /* STUDENTS MUST IMPLEMENT THE CODE HERE - END*/
  
  /* Set speed to wheels */
  m_acWheels->SetOutput(0, speed[0]);
  m_acWheels->SetOutput(1, speed[1]);
}

/******************************************************************************/
/******************************************************************************/

