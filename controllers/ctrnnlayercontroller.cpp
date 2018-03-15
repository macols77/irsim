#include "ctrnnlayercontroller.h"

/******************************************************************************/
/******************************************************************************/

CCTRNNLayerController::CCTRNNLayerController(const char* pch_name, CEpuck* pc_epuck, 
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
	m_pfOutputs = (double*) malloc(GetNumberOfOutputs() * sizeof(double));
	m_pfStates   = (double*) malloc(GetNumberOfOutputs() * sizeof(double));

  /* Reset layer */
  Reset();
}

/******************************************************************************/
/******************************************************************************/

CCTRNNLayerController::~CCTRNNLayerController()
{
    free(m_pfOutputs);
    free(m_pfStates);

}

/******************************************************************************/
/******************************************************************************/

double* CCTRNNLayerController::ComputeOutputs(double* pf_layer_inputs, double* pf_sensory_inputs, double* pf_weights)
{
 /* Info about chromosome 
  * \tetha \beta \tau \gain \weights_layers \weights_self
  *
  *
  * */

	/* For every output */
	for ( int i = 0; i < m_unNumberOfOutputs; i++ ) {
		
	
    /* We are going to build: State: s[k+1] = s[k] + \DeltaT/tau (-s[k] + \sum \omega x + gI) 
     * We cal \delta = -s[k] + \sum \omega x + gI 
     * If IDENTITY no state, just a stupid neuron */
    double f_delta = 0.0;
    

    if ( m_unActivationFunction != IDENTITY_ACTIVATION )
      f_delta = -m_pfStates[i];
		
		/* ABOUT THE SENSOR INPUTS*/
    /* ADD: gI
     * if sensory inputs */
		if ( pf_sensory_inputs != NULL)
		{

			/* If IDENTITY function, do no scale sensor inputs */
			if ( m_unActivationFunction == IDENTITY_ACTIVATION)
				// Add the sensor input
        //m_pfOutputs[i] = pf_sensory_inputs[i];
        f_delta += pf_sensory_inputs[i];
			
			/* if not, scale sensor inputs */
			else
      {
        // Add the scaled sensor input 
        int ji = i * ( m_unNumberOfLayerInputs + SIGMOID_CONSTANTS + GAIN_CONSTANTS ) + SIGMOID_CONSTANTS;

        f_delta += pf_sensory_inputs[i] * ( pf_weights[ji] * (m_fUpperBounds - m_fLowerBounds) + m_fLowerBounds);
      }
		}
	
		/* ABOUT THE WEIGHTs LAYER INPUTS */
    /* ADD: \sum W * y */
		/* If there is inputs comming from other layers */
		if ( pf_layer_inputs != NULL )
		{
			/* For the number of inputs */
			for( int j = 0; j < m_unNumberOfLayerInputs; j++ ) 
      {
        /* If IDENTITY function, input to delta */ 
				if ( m_unActivationFunction == IDENTITY_ACTIVATION )
					f_delta += pf_layer_inputs[j];	
        
        /* If no IDENTITY */
        else
        {
          /* Create index */
          int ji = 0;
          
          /* Compute the number weight number */
          /* If Hidden or Motor Layer*/
          if ( m_unNumberOfSensoryInputs == 0)
            ji = i * (m_unNumberOfLayerInputs + SIGMOID_CONSTANTS ) + (j + SIGMOID_CONSTANTS);
          
          /* If Associative Layer */
          else
            ji = i * (m_unNumberOfLayerInputs + SIGMOID_CONSTANTS + GAIN_CONSTANTS ) + (j + SIGMOID_CONSTANTS + GAIN_CONSTANTS);
          
          f_delta += ( pf_weights[ji] * (m_fUpperBounds - m_fLowerBounds) + m_fLowerBounds) * pf_layer_inputs[j];		
        }//end no identity
			}//end for layer input
		}//end weights layer

    /* ABOUT THE SELF connection 
     * Only for NO Identity and NO Sensory Layers*/
    //if ( ( m_unActivationFunction != IDENTITY_ACTIVATION ) && ( m_unNumberOfLayerInputs != 0 ) )
    //{
      //for ( int j = 0 ; j < m_unNumberOfOutputs ; j++ )
      //{
        ///* Create index */
        //int ji = 0;
        
        ///* If Hidden or Motor Layer*/
        //if ( m_unNumberOfSensoryInputs == 0)
          //ji = i * (m_unNumberOfLayerInputs + 3 + m_unNumberOfOutputs) + (j + 3 + m_unNumberOfLayerInputs);

        ///* If Associative Layer */
        //else
          //ji = i * (m_unNumberOfLayerInputs + 4 + m_unNumberOfOutputs) + (j + 4 + m_unNumberOfLayerInputs);

        //f_delta += ( pf_weights[ji] * (m_fUpperBounds - m_fLowerBounds) + m_fLowerBounds) * m_pfOutputs[j];		
      //}//end for layer output
    //}//end self weights 
    
    /* COMPUTE STATE */
    if ( m_unActivationFunction == IDENTITY_ACTIVATION)
      m_pfStates[i] = f_delta;
    
    /* Get and scale tau from chromosome */
    else
    {
      /* Create index */
      int ji = 0;
      
      //if hidden or motor
      if (m_unNumberOfSensoryInputs == 0)
        ji = i * (m_unNumberOfLayerInputs + SIGMOID_CONSTANTS ) + TAU_POSITION_CONSTANT;
      //if associative or sensory
      else
        ji = i * (m_unNumberOfLayerInputs + SIGMOID_CONSTANTS + GAIN_CONSTANTS ) + TAU_POSITION_CONSTANT;


      double tau = pow(10, (m_fUpperBounds * pf_weights[ji]) );

      /* Compute STATE */
      m_pfStates[i] += ((double) TIMESTEP / tau) * f_delta; 
    }

		/* ABOUT THE ACTIVATION */
		
		// Add the bias (weighted by the first weight to the i'th output node)

    double beta   = 0.0;
    double theta  = 0.0;
    /* Create index */
    int ji        = 0;
		
    /* Apply the bias and transfer function */
		switch ( m_unActivationFunction)
		{
			/* If IDENTITY, no bias nothing to do */
			case IDENTITY_ACTIVATION:
        m_pfOutputs[i] = m_pfStates[i];
        if ( m_pfOutputs[i] > 1.0 )
          m_pfOutputs[i] = 1.0;
        if ( m_pfOutputs[i] < 0.0 )
          m_pfOutputs[i] = 0.0;
				break;
			
			/* If SIGMOID, use bias and calc sigmoid */
			case SIGMOID_ACTIVATION:
        /* Get beta from chromosome */
        //if hidden or motor
        if (m_unNumberOfSensoryInputs == 0)
          ji = i * (m_unNumberOfLayerInputs + SIGMOID_CONSTANTS ) + BETA_POSITION_CONSTANT;
        //if associative or sensory
        else
          ji = i * (m_unNumberOfLayerInputs + SIGMOID_CONSTANTS + GAIN_CONSTANTS ) + BETA_POSITION_CONSTANT;

        beta = pf_weights[ji] * (m_fUpperBounds - m_fLowerBounds) + m_fLowerBounds;			   
				
        
        /* Get theta from chromosome */
        //if sensory input 
        //if hidden or motor
        if (m_unNumberOfSensoryInputs == 0)
          ji = i * (m_unNumberOfLayerInputs + SIGMOID_CONSTANTS );
        //if associative or sensory
        else
          ji = i * (m_unNumberOfLayerInputs + SIGMOID_CONSTANTS + GAIN_CONSTANTS );
        
        theta = pf_weights[ji] * (m_fUpperBounds - m_fLowerBounds) + m_fLowerBounds;			   
				
        /* Calc Sigmoid */
				m_pfOutputs[i] = 1.0/( 1 + exp ( -(beta * ( m_pfStates[i] - theta ) ) ) );	
				//printf("m_pfOutputs[%d]-sigmoid: %2f\n", i,  m_pfOutputs[i] );
				break;
		
			/* IF STEP, use not scaled BIAS as a THRESHOLD */
			case STEP_ACTIVATION:
				/* Get theta from chromosome */
        theta = pf_weights[i * (m_unNumberOfLayerInputs + 3 )] * (m_fUpperBounds - m_fLowerBounds) + m_fLowerBounds;			   
				/* If output bigger than THRESHOLD output 1 */
				if ( m_pfStates[i] > theta )
					m_pfOutputs[i] = 1.0;
				else
					m_pfOutputs[i] = 0.0;
				break;

			/* If LINEAR, do not use BIAS and create y=1-x, function */
			case LINEAR_ACTIVATION: 
				m_pfOutputs[i] = 1 - m_pfStates[i];
				break;

			/* If PROGRAM, create your own equation */
			case PROGRAM_ACTIVATION:
				/* YOU NEED TO PROGRAM HERE YOUR EQUATION */
				break;
		}
	}
	
  return m_pfOutputs;
}
/******************************************************************************/
/******************************************************************************/

void CCTRNNLayerController::Reset()
{
  /* DEBUG */
  //printf("RESET on CCTRNNLayerController:Reset\n");
  /* DEBUG */
  
  for ( int i = 0; i < GetNumberOfOutputs() ; i++)
  {
    m_pfOutputs[i] = 0.0;
    m_pfStates[i] = 0.0;
  }
}

///******************************************************************************/
///******************************************************************************/

unsigned int CCTRNNLayerController::GetNumberOfSensoryInputs ( void )
{
	return m_unNumberOfSensoryInputs;
}

/******************************************************************************/
/******************************************************************************/
unsigned int CCTRNNLayerController::GetNumberOfLayerInputs ( void )
{
	return m_unNumberOfLayerInputs;
}

/******************************************************************************/
/******************************************************************************/

unsigned int CCTRNNLayerController::GetNumberOfOutputs ( void )
{
	return m_unNumberOfOutputs;
}

/******************************************************************************/
/******************************************************************************/

unsigned int CCTRNNLayerController::GetLabel ( void )
{
	return m_unLabel;
}

/******************************************************************************/
/******************************************************************************/
