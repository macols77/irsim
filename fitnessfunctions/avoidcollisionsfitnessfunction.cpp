#include "avoidcollisionsfitnessfunction.h"
#include "collisionmanager.h"

/******************************************************************************/
/******************************************************************************/

CAvoidCollisionsFitnessFunction::CAvoidCollisionsFitnessFunction(const char* pch_name, 
                                                                 CSimulator* pc_simulator, 
                                                                 unsigned int un_collisions_allowed_per_epuck)
    :
    CFitnessFunction(pch_name, pc_simulator)
{

	/* Check number of robots */
	m_pcSimulator = pc_simulator;
	TEpuckVector* pvecEpucks=m_pcSimulator->GetEpucks();
	
	if ( pvecEpucks->size() == 0 )
	{
		printf("No Robot, so fitness function can not be computed.\n Exiting...\n");
		fflush(stdout);
		exit(0);
	}
	else if  (pvecEpucks->size()>1)
	{
		printf("More than 1 robot, and fitness is not prepared for it.\n Exiting...\n");
	}
    
	m_pcEpuck=(*pvecEpucks)[0];

	m_unNumberOfSteps 		= 0;
	m_fComputedFitness 		= 0.0;
	m_unCollisionsNumber 	= 0;
	
}


/******************************************************************************/
/******************************************************************************/

double CAvoidCollisionsFitnessFunction::GetFitness()
{    
	double fit = ( m_fComputedFitness / (double) m_unNumberOfSteps ) * (1 - ((double) (fmin(m_unCollisionsNumber,10.0)/10.0)));
	//double fit = ( m_fComputedFitness / (double) m_unNumberOfSteps );
	
  if ( fit < 0.0 ) fit = 0.0;

	return fit;
}

/******************************************************************************/
/******************************************************************************/
void CAvoidCollisionsFitnessFunction::SimulationStep(unsigned int n_simulation_step, double f_time, double f_step_interval)
{
	/* See Evolutionary Robotics Book */
	/* This is the function to be implemented */
	/* f = V * ( 1 - sqrt(Delta(v)) ) * (1 - i)
	 * V relates to the maximum speed
	 * Delta(v) relates to the movement on the same direction
	 * i relates to the maximum sensor value
	 */
	
	double leftSpeed = 0.0;
	double rightSpeed = 0.0;
	
	m_pcEpuck->GetWheelSpeed(&leftSpeed,&rightSpeed);
	leftSpeed 	= 0.5 + ( leftSpeed  / ( 2.0 *  m_pcEpuck->GetMaxWheelSpeed()) );
	rightSpeed 	= 0.5 + ( rightSpeed / ( 2.0 *  m_pcEpuck->GetMaxWheelSpeed()) );

	/* Eval maximum speed partial fitness */
	double maxSpeedEval = (fabs(leftSpeed - 0.5) + fabs(rightSpeed - 0.5));

	/* Eval same direction partial fitness */
	double sameDirectionEval = 1 - sqrt(fabs(leftSpeed - rightSpeed));
	
	/* Eval minimum sensor reading partial fitness */
	double maxProxSensorEval = 0;
	double maxLightSensorEval = 0;
	
	TSensorVector vecSensors = m_pcEpuck->GetSensors();
	for (TSensorIterator i = vecSensors.begin(); i != vecSensors.end(); i++)
	{
		if ( (*i)->GetType() == SENSOR_PROXIMITY)
		{
			unsigned int unThisSensorsNumberOfInputs = (*i)->GetNumberOfInputs();
			double* pfThisSensorInputs = (*i)->GetComputedSensorReadings();

			for (int j = 0; j < unThisSensorsNumberOfInputs; j++)
			{
				if ( pfThisSensorInputs[j] > maxProxSensorEval )
				{
					//if (j == 0 || j == 1 || j == 6 || j == 7)
					maxProxSensorEval = pfThisSensorInputs[j];
				}
			}
		}
	}

	maxProxSensorEval = 1 - maxProxSensorEval;
	
	/* Max Speed * Same Direction * Min Prox * go forwards */
	double fitness = maxSpeedEval * sameDirectionEval * maxProxSensorEval * (leftSpeed * rightSpeed);

	m_unNumberOfSteps++;
	m_fComputedFitness += fitness;

	/* Get Collisions */
	int nContact = 0;
	CContactSensor *m_seContact = (CContactSensor*) m_pcEpuck->GetSensor(SENSOR_CONTACT);
	double* contact = m_seContact->GetSensorReading(m_pcEpuck);
	for ( int j = 0 ; j < m_seContact->GetNumberOfInputs() ; j++)
	{
		if(contact[j] > 0.0) 
			nContact=1;
	} 

	if ( nContact == 1 )
		m_unCollisionsNumber++;		

}

/******************************************************************************/
/******************************************************************************/
