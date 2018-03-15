#include "perceptroncontroller.h"

/******************************************************************************/
/******************************************************************************/

CPerceptronController::CPerceptronController(const char* pch_name, CEpuck* pc_epuck) : 
    CNNController(pch_name, pc_epuck)
{

	unsigned int unRequiredNumberOfWeights = (GetNumberOfSensorInputs() + 1) * GetNumberOfActuatorOutputs();
	m_pfWeights = (double*) malloc(unRequiredNumberOfWeights * sizeof(double));
	m_pfOutputs = (double*) malloc(GetNumberOfActuatorOutputs() * sizeof(double));;
	m_fUpperBounds=1.0;
	m_fLowerBounds=0.0;

	//printf("Perceptron controller instantiated\n");
	//fflush(stdout);
	
}

/******************************************************************************/
/******************************************************************************/

CPerceptronController::~CPerceptronController()
{
    free(m_pfWeights);
    free(m_pfOutputs);
}

/******************************************************************************/
/******************************************************************************/

void CPerceptronController::SetWeights(unsigned int un_number_of_weights, 
                                       double* pf_weights) 
{
	//printf("CONTROLLER: SetWeights called!!\n");
	//fflush(stdout);
    unsigned int unRequiredNumberOfWeights = (GetNumberOfSensorInputs() + 1) * GetNumberOfActuatorOutputs();
    if (un_number_of_weights != unRequiredNumberOfWeights)
    {
			//printf("Trying to set an incorrect number of weights (%d should be %d), the sbot\n"
					//"has %d sensor inputs and %d actuator outputs\n", un_number_of_weights,
					//unRequiredNumberOfWeights, GetNumberOfSensorInputs(), GetNumberOfActuatorOutputs());
        return;
    }      

	
    // Set the weights:
    memcpy(m_pfWeights, pf_weights, unRequiredNumberOfWeights * sizeof(double));    
	
	//Scale the weights in given range
	if(!((m_fUpperBounds==1.0)&&(m_fLowerBounds==0.0))){
		for(int i=0;i<un_number_of_weights;i++){
			m_pfWeights[i]=m_pfWeights[i]*(m_fUpperBounds-m_fLowerBounds)+m_fLowerBounds;
		}
	}	
}

/******************************************************************************/
/******************************************************************************/


//WARNING: MODIFIED VERSION FOR THE CORRIDOR EXPERIMENT 

/*
const double* CPerceptronController::ComputeOutputs(double* pf_inputs)
{
    for( int i = 0; i < m_unNumberOfActuatorOutputs; i++ ) {
        // Add the bias (weighted by the first weight to the i'th output node)
        m_pfOutputs[i] = m_pfWeights[i * (m_unNumberOfSensorInputs + 1)];			    
        
        //for( int j = 0; j < m_unNumberOfSensorInputs; j++ ) {
		for( int j = 0; j < 4; j++ ) {
            // Compute the weight number
            //int ji = i * (m_unNumberOfSensorInputs + 1) + (j + 1);
			int ji = i * (5) + (j + 1);
            // Add the weighted input
            m_pfOutputs[i] += m_pfWeights[ji] * (pf_inputs[j]+pf_inputs[j+1])/2.0;			 
        }
        
        // Apply the transfer function (sigmoid with output in [0,1])
        m_pfOutputs[i] = 1.0/( 1 + exp( -m_pfOutputs[i]) );	  
    }

    return m_pfOutputs;
}*/

const double* CPerceptronController::ComputeOutputs(double* pf_inputs)
{

	/* DEBUG */
	//printf("Number of inputs: %d\n",m_unNumberOfSensorInputs);
	//for(int k=0;k<m_unNumberOfSensorInputs;k++){
	//printf("Current sensor: %d, value: %f\n",k,pf_inputs[k]);
	//fflush(stdout);
	//}
	/* DEBUG */
	
    for( int i = 0; i < m_unNumberOfActuatorOutputs; i++ ) {
        // Add the bias (weighted by the first weight to the i'th output node)
        m_pfOutputs[i] = m_pfWeights[i * (m_unNumberOfSensorInputs + 1)];			    
        
        for( int j = 0; j < m_unNumberOfSensorInputs; j++ ) {
            // Compute the weight number
            int ji = i * (m_unNumberOfSensorInputs + 1) + (j + 1);
            // Add the weighted input
            m_pfOutputs[i] += m_pfWeights[ji] * pf_inputs[j];			
						/* DEBUG */
						//printf(" %2f ", m_pfWeights[ji]);
						/* DEBUG */
        }
        
        // Apply the transfer function (sigmoid with output in [0,1])
        m_pfOutputs[i] = 1.0/( 1 + exp( -m_pfOutputs[i]) );	 

				/* DEBUG */
				//printf("\n%2f\n",m_pfOutputs[i]);
				/* DEBUG */
    }
		//sleep(1);
    return m_pfOutputs;
}
/******************************************************************************/
/******************************************************************************/

void CPerceptronController::Reset(){
}

/******************************************************************************/
/******************************************************************************/

void CPerceptronController::SetUpperBounds(float fUB){
	m_fUpperBounds=fUB;	
}

/******************************************************************************/
/******************************************************************************/

void CPerceptronController::SetLowerBounds(float fLB){
	m_fLowerBounds=fLB;
}

/******************************************************************************/
/******************************************************************************/
/*
CPerceptronController::CPerceptronController(const char* pch_name, 
                                             CSbot* pc_sbot,
                                             CArguments* pc_controller_arguments) : 
    CNNController(pch_name, pc_sbot, pc_controller_arguments)
{

    unsigned int unRequiredNumberOfWeights = (GetNumberOfSensorInputs() + 1) * GetNumberOfActuatorOutputs();

    m_pfWeights = (double*) malloc(unRequiredNumberOfWeights * sizeof(double));
    m_pfOutputs = (double*) malloc(GetNumberOfActuatorOutputs() * sizeof(double));;
}

/******************************************************************************/
/******************************************************************************/
