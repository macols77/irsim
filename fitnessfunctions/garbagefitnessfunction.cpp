#include "garbagefitnessfunction.h"
#include "collisionmanager.h"

#define SEARCH 	0
#define DEPOSIT 1
/******************************************************************************/
/******************************************************************************/

CGarbageFitnessFunction::CGarbageFitnessFunction(const char* pch_name, 
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
	m_unState = SEARCH;

	
	m_unCollisionsNumber= 0;		
	m_unGreyFlag = 0;
	m_unGreyCounter = 0;
}

/******************************************************************************/
/******************************************************************************/

CGarbageFitnessFunction::~CGarbageFitnessFunction(){
}
/******************************************************************************/
/******************************************************************************/

double CGarbageFitnessFunction::GetFitness()
{    
  /* Start Exp1 */
  //double fit = ( m_fComputedFitness / (double) m_unNumberOfSteps );
  /* End Exp1 */
  
  /* Start Exp2, Exp6-A, Exp6-B */
  //double fit = ( m_fComputedFitness / (double) m_unNumberOfSteps ) * (1 - ((double) (fmin(m_unCollisionsNumber,10.0)/10.0)));
  /* End Exp2 */

  /* Start Exp3-5, Exp6-C , Exp 7-8*/
  double fit = ( m_fComputedFitness / (double) m_unNumberOfSteps ) * (1 - ((double) (fmin(m_unCollisionsNumber,30.0)/30.0))) * ( (double) (fmin(m_unGreyCounter, 5.0)/ 5.0 ));
  if (m_unGreyFlag == 0 )
    fit /= 10.0;
  /* End Exp3-6 */

	
  if ( fit < 0.0 ) fit = 0.0;
  if ( fit > 1.0 ) fit = 1.0;

	return fit;
}

/******************************************************************************/
/******************************************************************************/
void CGarbageFitnessFunction::SimulationStep(unsigned int n_simulation_step, double f_time, double f_step_interval)
{

 /* Maximize movement as in avoidfitnessfunction */

	double leftSpeed = 0.0;
	double rightSpeed = 0.0;
	m_pcEpuck->GetWheelSpeed(&leftSpeed,&rightSpeed);
	leftSpeed = 0.5 + ( leftSpeed / ( 2.0 *  m_pcEpuck->GetMaxWheelSpeed()) );
	rightSpeed = 0.5 + ( rightSpeed / ( 2.0 *  m_pcEpuck->GetMaxWheelSpeed()) );

	/* Eval maximum speed partial fitness */
	double maxSpeedEval = (fabs(leftSpeed - 0.5) + fabs(rightSpeed - 0.5));

	/* Eval same directio partial fitness */
	double sameDirectionEval = 1 - sqrt(fabs(leftSpeed - rightSpeed));
	
	/* Eval minimum sensor reading partial fitness */
	double maxProxSensorEval = 0;
	double maxLightSensorEval = 0;
	double maxBlueLightSensorEval = 0;
	double lightRange = 0;
	TSensorVector vecSensors = m_pcEpuck->GetSensors();

	double* groundMemory;
	double* ground;
	
  double blueLightS0=0;
	double blueLightS7=0;
	double lightS0=0;
	double lightS7=0;

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
					maxProxSensorEval = pfThisSensorInputs[j];
				}
			}
		}

		else if ( (*i)->GetType() == SENSOR_GROUND_MEMORY)
		{
			groundMemory = (*i)->GetComputedSensorReadings();
		}
		
		else if ( (*i)->GetType() == SENSOR_GROUND)
		{
			ground = (*i)->GetComputedSensorReadings();
		}
		
		else if ( (*i)->GetType() == SENSOR_REAL_LIGHT)
		{
			unsigned int unThisSensorsNumberOfInputs = (*i)->GetNumberOfInputs();
			double* pfThisSensorInputs = (*i)->GetComputedSensorReadings();

			for (int j = 0; j < unThisSensorsNumberOfInputs; j++)
			{
				if ( pfThisSensorInputs[j] > maxLightSensorEval )
				{	
					maxLightSensorEval = pfThisSensorInputs[j];
				}
        if (j==0)
          lightS0 = pfThisSensorInputs[j];
        else if (j==7)
          lightS7 = pfThisSensorInputs[j];
			}
		}
		else if ( (*i)->GetType() == SENSOR_REAL_BLUE_LIGHT)
		{
			unsigned int unThisSensorsNumberOfInputs = (*i)->GetNumberOfInputs();
			double* pfThisSensorInputs = (*i)->GetComputedSensorReadings();

			for (int j = 0; j < unThisSensorsNumberOfInputs; j++)
			{
				if ( pfThisSensorInputs[j] > maxBlueLightSensorEval )
				{	
					maxBlueLightSensorEval = pfThisSensorInputs[j];
				}
        if (j==0)
          blueLightS0 = pfThisSensorInputs[j];
        else if (j==7)
          blueLightS7 = pfThisSensorInputs[j];
			}
		}
	}

	maxProxSensorEval = 1 - maxProxSensorEval;

	/* START Garbage Exp 1 */
  //double fitness = 0.0;

  //if (groundMemory[0] > 0.0)
  //{
    //fitness = maxLightSensorEval;
    //m_unGreyFlag = 1;
  //}
  //else
    //fitness = maxSpeedEval * sameDirectionEval * maxProxSensorEval * (leftSpeed * rightSpeed);
	/* END Garbage Exp 1 */
	
	/* START Garbage Exp 2 */
  //double fitness = 0.0; //maxSpeedEval * sameDirectionEval;

  //if (groundMemory[0] > 0.0)
  //{
    //fitness = ( lightS0 + lightS7);
  //}
  //else
  //{
    //fitness = ( blueLightS0 + blueLightS7);
  //}

	/* END Garbage Exp 2 */
	
  /* START Garbage Exp 3 */
  //double fitness = 1; //maxSpeedEval * sameDirectionEval;

  //if (groundMemory[0] > 0.0)
  //{
    //fitness *= ( lightS0 + lightS7);
    //if (m_unGreyFlag == 0)
    //{
      //m_unGreyFlag = 1;
      //m_unGreyCounter++;
    //}
  //}
  //else
  //{
    //fitness *= ( blueLightS0 + blueLightS7);
    //if (m_unGreyFlag == 1)
    //{
      //m_unGreyFlag = 0;
    //}
  //}

	/* END Garbage Exp 3 */
  
  /* START Garbage Exp 4-5 */
  //double fitness = 1; //maxSpeedEval * sameDirectionEval;

  //if (groundMemory[0] > 0.0)
  //{
    //fitness *= ( lightS0 + lightS7);
    //if (m_unGreyFlag == 0)
    //{
      //m_unGreyFlag = 1;
      //m_unGreyCounter++;
    //}
  //}
  //else
  //{
    //fitness *= ( blueLightS0 + blueLightS7);
    //if (m_unGreyFlag == 1)
    //{
      //m_unGreyFlag = 0;
    //}
  //}

  //fitness *= maxProxSensorEval * (leftSpeed * rightSpeed); // * (1-maxLightSensorEval);
	/* END Garbage Exp 4-5 */
  
  /* START Garbage Exp 6A */
  //double fitness = 1; //0.5 * (maxSpeedEval * sameDirectionEval);
  //fitness *= (( blueLightS0 + blueLightS7)/2);
  /* END Garbage Exp 6A */

  /* START Garbage Exp 6B */
  //double fitness = 1; //0.5 * (maxSpeedEval * sameDirectionEval);

  //if (groundMemory[0] > 0.0)
  //{
    //fitness *= ( lightS0 + lightS7)/2;
  //}
  //else
  //{
    //fitness *= (( blueLightS0 + blueLightS7)/2);
  //}
	/* END Garbage Exp 6B */

  /* START Garbage Exp 6C */
  //double fitness = 1; //0.5 * (maxSpeedEval * sameDirectionEval);

  //if (groundMemory[0] > 0.0)
  //{
    //fitness *= ( lightS0 + lightS7)/2;
    //if (m_unGreyFlag == 0)
    //{
      //m_unGreyFlag = 1;
      //m_unGreyCounter++;
    //}
  //}
  //else
  //{
    //fitness *= (( blueLightS0 + blueLightS7)/2);
    //if (m_unGreyFlag == 1)
    //{
      //m_unGreyFlag = 0;
    //}
  //}
	/* END Garbage Exp 6C */
  
  /* START Garbage Exp 7-8 */
  double fitness = 1; //0.5 * (maxSpeedEval * sameDirectionEval);

  if (groundMemory[0] > 0.0)
  {
    fitness *= ( lightS0 + lightS7)/2;
    if (m_unGreyFlag == 0)
    {
      m_unGreyFlag = 1;
      m_unGreyCounter++;
    }
  }
  else
  {
    fitness *= (( blueLightS0 + blueLightS7)/2);
    if (m_unGreyFlag == 1)
    {
      m_unGreyFlag = 0;
    }
  }
	/* END Garbage Exp 7-8 */

 
	m_unNumberOfSteps++;
	m_fComputedFitness += fitness;
	
	//printf("ComputedFitness: %2f\n", m_fComputedFitness);
	
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
