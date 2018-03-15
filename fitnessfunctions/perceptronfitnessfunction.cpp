#include "perceptronfitnessfunction.h"
#include "collisionmanager.h"

/******************************************************************************/
/******************************************************************************/

CPerceptronFitnessFunction::CPerceptronFitnessFunction(const char* pch_name, 
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

	m_unNumberOfSteps = 0;
	m_fComputedFitness = 0.0;
}


/******************************************************************************/
/******************************************************************************/

double CPerceptronFitnessFunction::GetFitness()
{    

	//printf("COLL: %2d\n",CCollisionManager::GetInstance()->GetTotalNumberOfCollisions());
	//fflush(stdout);
	/* If the robot has collide */
	//if (CCollisionManager::GetInstance()->GetTotalNumberOfCollisions() != 0)
	//{
	//return 0;
	//}
	//
	int coll = (CCollisionManager::GetInstance()->GetTotalNumberOfCollisions());

	double fit = ((m_fComputedFitness / (double) m_unNumberOfSteps) - ((double) coll * 0.1));
	if ( fit < 0.0 ) fit = 0.0;

	return fit;
}

/******************************************************************************/
/******************************************************************************/
void CPerceptronFitnessFunction::SimulationStep(unsigned int n_simulation_step, double f_time, double f_step_interval)
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
	leftSpeed = 0.5 + ( leftSpeed / ( 2.0 *  m_pcEpuck->GetMaxWheelSpeed()) );
	rightSpeed = 0.5 + ( rightSpeed / ( 2.0 *  m_pcEpuck->GetMaxWheelSpeed()) );

	/* Eval maximum speed partial fitness */
	double maxSpeedEval = (leftSpeed + rightSpeed) / 2.0;

	/* Eval same directio partial fitness */
	double sameDirectionEval = 1 - sqrt(fabs(leftSpeed - rightSpeed));

	/* Eval minimum sensor reading partial fitness */
	double maxSensorEval = 0;
	TSensorVector vecSensors = m_pcEpuck->GetSensors();
	for (TSensorIterator i = vecSensors.begin(); i != vecSensors.end(); i++)
	{
		unsigned int unThisSensorsNumberOfInputs = (*i)->GetNumberOfInputs();
		double* pfThisSensorInputs = (*i)->GetComputedSensorReadings();

		for (int j = 0; j < unThisSensorsNumberOfInputs; j++)
		{
			if ( pfThisSensorInputs[j] > maxSensorEval )
			{	
				maxSensorEval = pfThisSensorInputs[j];
			}
		}
	}

	maxSensorEval = 1 - maxSensorEval;

	double fitness = maxSpeedEval * sameDirectionEval * maxSensorEval;

	m_unNumberOfSteps++;
	m_fComputedFitness += fitness;

	///* DEBUG */
	//printf("LeftSpeed %2f RightSpeed %2f MaxSensor %2f PartialFitness1 %2f PartialFitness2 %2f TimeStepFitness %2f Fitness %2f TimeStep %u\n",
					//leftSpeed, rightSpeed, maxSensorEval,
					//maxSpeedEval, sameDirectionEval,
					//fitness, m_fComputedFitness, m_unNumberOfSteps);
	//sleep(1);
	/* DEBUG */
}

/******************************************************************************/
/******************************************************************************/
