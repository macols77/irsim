#include "layercontroller.h"

/******************************************************************************/
/******************************************************************************/

CLayerController::CLayerController(const char* pch_name, CEpuck* pc_epuck, 
																	 unsigned int un_number_of_sensory_inputs,
																	 unsigned int un_number_of_layer_inputs, 
																	 unsigned int un_number_of_outputs, 
																	 unsigned int un_activation_function,
																	 unsigned int un_label, 
																	 double f_lower_bounds, 
																	 double f_upper_bounds) 
{

	m_unNumberOfSensoryInputs = un_number_of_sensory_inputs;
	m_unNumberOfLayerInputs = un_number_of_layer_inputs;
	m_unNumberOfOutputs = un_number_of_outputs;
	m_unActivationFunction = un_activation_function;
	m_unLabel = un_label;
	m_fLowerBounds = f_lower_bounds;
	m_fUpperBounds = f_upper_bounds;


	m_unRequiredNumberOfWeights = (GetNumberOfLayerInputs() + 1) * GetNumberOfOutputs();
	//m_pfWeights = (double*) malloc(m_unRequiredNumberOfWeights * sizeof(double));
	m_pfOutputs = (double*) malloc(GetNumberOfOutputs() * sizeof(double));;
	//m_fUpperBounds=1.0;
	//m_fLowerBounds=0.0;
}

/******************************************************************************/
/******************************************************************************/

CLayerController::~CLayerController()
{
    free(m_pfWeights);
    free(m_pfOutputs);

}

/******************************************************************************/
/******************************************************************************/

//void CLayerController::SetWeights(unsigned int un_number_of_weights, 
                                       //double* pf_weights) 
//{
	//if (un_number_of_weights != m_unRequiredNumberOfWeights)
	//{
		//printf("CLayerController::SetWeights ERROR. Numero incorrecto de pesos\n");
		//exit(1);
		////return;
	//}      

	
	//// Set the weights:
	//memcpy(m_pfWeights, pf_weights, m_unRequiredNumberOfWeights * sizeof(double));    

	////Scale the weights in given range
	//if(!((m_fUpperBounds==1.0)&&(m_fLowerBounds==0.0))){
		//for(int i=0;i<un_number_of_weights;i++){
			//m_pfWeights[i]=m_pfWeights[i]*(m_fUpperBounds-m_fLowerBounds)+m_fLowerBounds;
		//}
	//}	
//}

/******************************************************************************/
/******************************************************************************/

double* CLayerController::ComputeOutputs(double* pf_layer_inputs, double* pf_sensory_inputs, double* pf_weights)
{

	/* For every output */
	for( int i = 0; i < m_unNumberOfOutputs; i++ ) {
		
		/* ABOUT THE SENSOR INPUTS*/
		
		/* If no sensory inputs init m_pfOutputs to 0 */
		if ( pf_sensory_inputs == NULL)
		{
			m_pfOutputs[i] = 0.0;
		}
		/* if sensory inputs */
		else
		{

			/* If IDENTITY function, do no scale sensor inputs */
      //if ( m_unActivationFunction == IDENTITY_ACTIVATION)
				// Add the sensor input
				m_pfOutputs[i] = pf_sensory_inputs[i];
			
			/* if not, scale sensor inputs */
        //else
				// Add the scaled sensor input 
				// wight is 1.0, there is not gain
        //m_pfOutputs[i] = pf_sensory_inputs[i] * ( 1.0 * (m_fUpperBounds - m_fLowerBounds) + m_fLowerBounds);
		
		}
	
		/* ABOUT THE WEIGHT LAYER INPUTS */

		/* If there is inputs comming from other layers */
		if ( pf_layer_inputs != NULL )
		{
			/* For the number of inputs */
			for( int j = 0; j < m_unNumberOfLayerInputs; j++ ) {
				/* Compute the number weight number */
				int ji = i * (m_unNumberOfLayerInputs + 1) + (j + 1);
				
				/* If LINEAR function, do not scale weights */
				if ( m_unActivationFunction == LINEAR_ACTIVATION )
					// Add the weighted input
					m_pfOutputs[i] += pf_weights[ji] * pf_layer_inputs[j];		
				/* If not, scale weights */
				else
					// Add the scaled weighted input
					m_pfOutputs[i] += ( pf_weights[ji] * (m_fUpperBounds - m_fLowerBounds) + m_fLowerBounds) * pf_layer_inputs[j];		
			}
			//cout << endl;
		}

		/* ABOUT THE BIAS */
		
		// Add the bias (weighted by the first weight to the i'th output node)
		
		/* Apply the bias and transfer function */
		double beta = 1.0;
		switch ( m_unActivationFunction)
		{
			/* If IDENTITY, no bias nothing to do */
			case IDENTITY_ACTIVATION:
				break;
			
			/* If SIGMOID, use bias and calc sigmoid */
			case SIGMOID_ACTIVATION:
				
				//printf("wieght:  %2f\n",pf_weights[i * (m_unNumberOfLayerInputs + 1)]);
				//printf("m_pfOutputs[%d]: %2f\n", i,  m_pfOutputs[i] );
				/* Add scaled BIAS */
				m_pfOutputs[i] -= pf_weights[i * (m_unNumberOfLayerInputs + 1)] * (m_fUpperBounds - m_fLowerBounds) + m_fLowerBounds;			   
				//printf("m_pfOutputs[%d]: %2f\n", i,  m_pfOutputs[i] );
				/* Calc Sigmoid */
				m_pfOutputs[i] = 1.0/( 1 + exp ( - (beta * m_pfOutputs[i]) ) );	
				//printf("m_pfOutputs[%d]-sigmoid: %2f\n", i,  m_pfOutputs[i] );

				break;
		
			/* IF STEP, use not scaled BIAS as a THRESHOLD */
			case STEP_ACTIVATION:
				/* If output bigger than THRESHOLD output 1 */
				if ( m_pfOutputs[i] > pf_weights[i * (m_unNumberOfLayerInputs + 1)] )
					m_pfOutputs[i] = 1.0;
				/* If not, output 0 */
				else
					m_pfOutputs[i] = 0.0;
				break;

			/* If LINEAR, do not use BIAS and create y=1-x, function */
			case LINEAR_ACTIVATION: 
				m_pfOutputs[i] = 1 - m_pfOutputs[i]; 
				break;

			/* If PROGRAM, create your own equation */
			case PROGRAM_ACTIVATION:
				/* YOU NEED TO PROGRAM HERE YOUR EQUATION */
				break;
		}

		/* DEBUG */
		//printf("\n%2f\n",m_pfOutputs[i]);
		/* DEBUG */
	}
	//sleep(1);
	return m_pfOutputs;
}
/******************************************************************************/
/******************************************************************************/

void CLayerController::Reset(){
}

///******************************************************************************/
///******************************************************************************/

//void CLayerController::SetUpperBounds(float fUB){
	//m_fUpperBounds=fUB;	
//}

///******************************************************************************/
///******************************************************************************/

//void CLayerController::SetLowerBounds(float fLB){
	//m_fLowerBounds=fLB;
//}

///******************************************************************************/
///******************************************************************************/

unsigned int CLayerController::GetNumberOfSensoryInputs ( void )
{
	return m_unNumberOfSensoryInputs;
}

/******************************************************************************/
/******************************************************************************/
unsigned int CLayerController::GetNumberOfLayerInputs ( void )
{
	return m_unNumberOfLayerInputs;
}

/******************************************************************************/
/******************************************************************************/

unsigned int CLayerController::GetNumberOfOutputs ( void )
{
	return m_unNumberOfOutputs;
}

/******************************************************************************/
/******************************************************************************/

unsigned int CLayerController::GetLabel ( void )
{
	return m_unLabel;
}

/******************************************************************************/
/******************************************************************************/
