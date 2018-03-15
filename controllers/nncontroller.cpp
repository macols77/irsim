#include "nncontroller.h"


/******************************************************************************/
/******************************************************************************/

CNNController::CNNController(const char* pch_name, 
                             CEpuck* pc_epuck) :
    CController(pch_name, pc_epuck),
    m_pfInputs(NULL)
{
    m_unNumberOfSensorInputs = 0;
    TSensorVector vecSensors = pc_epuck->GetSensors();
    for (TSensorIterator i = vecSensors.begin(); i != vecSensors.end(); i++)
    {
        m_unNumberOfSensorInputs += (*i)->GetNumberOfInputs();
    }        
    m_unNumberOfActuatorOutputs = 0;
    TActuatorVector vecActuators = pc_epuck->GetActuators();
    for (TActuatorIterator j = vecActuators.begin(); j != vecActuators.end(); j++)
    {
        m_unNumberOfActuatorOutputs += (*j)->GetNumberOfOutputs();
    }        

    if (m_unNumberOfSensorInputs == 0)
    {
        printf("No sensor inputs specified for sbot %s. Detected when creating the controller\n"
               "%s. Remember to add all sensors to an sbot BEFORE creating its controller.", 
               pc_epuck->GetName(), pch_name);
		fflush(stdout);
    }

    if (m_unNumberOfActuatorOutputs == 0)
    {
        printf("No actuator outputs specified for sbot %s. Detected when creating the controller\n"
               "%s. Remember to add all actuators to an sbot BEFORE creating its controller.", 
               pc_epuck->GetName(), pch_name);
    }

    m_pfInputs  = (double*) malloc(m_unNumberOfSensorInputs  * sizeof(double));
}

/******************************************************************************/
/******************************************************************************/

CNNController::~CNNController()
{
    if (m_pfInputs)
        free(m_pfInputs);
}


/******************************************************************************/
/******************************************************************************/

void CNNController::SimulationStep(unsigned n_step_number, double f_time, double f_step_interval)
{

    unsigned int unCurrentInput = 0;

    // First we get all the sensor readings for inputs:
    TSensorVector vecSensors = GetEpuck()->GetSensors();
    for (TSensorIterator i = vecSensors.begin(); i != vecSensors.end(); i++)
    {
          unsigned int unThisSensorsNumberOfInputs = (*i)->GetNumberOfInputs();          
					double* pfThisSensorsInputs = (*i)->GetComputedSensorReadings();

					for (int j = 0; j < unThisSensorsNumberOfInputs; j++)
          {
              m_pfInputs[unCurrentInput++] = pfThisSensorsInputs[j];
							//printf("%2f\n",pfThisSensorsInputs[j]);
          }           
    }        


    // Then propagate the readings and get the output:
    const double* pfOutputs = ComputeOutputs(m_pfInputs);
    unsigned int unCurrentOutput = 0;

    // Apply the outputs to all the actuators:
    TActuatorVector vecActuators = GetEpuck()->GetActuators();
    for (TActuatorIterator k = vecActuators.begin(); k != vecActuators.end(); k++)
    {
          unsigned int unThisActuatorNumberOfOutputs = (*k)->GetNumberOfOutputs();
          
          for (int l = 0; l < unThisActuatorNumberOfOutputs; l++)
          {
              (*k)->SetOutput(l, pfOutputs[unCurrentOutput++]);
          }              
    }
}

/******************************************************************************/
/******************************************************************************/

unsigned int CNNController::GetNumberOfSensorInputs()
{
    return m_unNumberOfSensorInputs;
}

/******************************************************************************/
/******************************************************************************/

unsigned int CNNController::GetNumberOfActuatorOutputs()
{
    return m_unNumberOfActuatorOutputs;
}

/******************************************************************************/
/******************************************************************************/

double* CNNController::LoadWeights(const char* pch_filename)
{
	printf("Controller: %s, loading weights from: %s\n",GetName(),pch_filename);
    int nErrorCode;
    ifstream in;
    in.open( pch_filename, ios::in );
    if( !in ) {
        printf("Cannot open file containing neural network weights: %s", pch_filename);
		fflush(stdout);
    }

    int length = 0;
    if( !(in >> length) ) {
        printf("Cannot read file containing neural network weights: %s\n", pch_filename);
		fflush(stdout);
    }

    double* weights = new double[length];

    for( int i = 0; i < length; i++ ) {
        if( !(in >> weights[i] ) ) {
            printf("Cannot read weight %d from file: %s.\n", i, pch_filename);
			fflush(stdout);
        }
    }
	
	return weights;
    //delete weights;
}

/******************************************************************************/
/******************************************************************************/

void CNNController::SaveState(const char* pch_filename){

}

/******************************************************************************/
/******************************************************************************/

void CNNController::LoadState(const char* pch_filename){
	
}
/******************************************************************************/
/******************************************************************************/
