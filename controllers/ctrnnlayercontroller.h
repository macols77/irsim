#ifndef CTRNNLAYERCONTROLLER_H_
#define CTRNNLAYERCONTROLLER_H_

/******************************************************************************/
/******************************************************************************/

#include <math.h>
#include <vector>
#include <list>

using namespace std;

class CCTRNNLayerController;


/******************************************************************************/
/******************************************************************************/

/*#include "controller.h"*/
#include "epuck.h"

/******************************************************************************/
/******************************************************************************/
#define IDENTITY_ACTIVATION	0
#define SIGMOID_ACTIVATION 	1
#define STEP_ACTIVATION 		2
#define LINEAR_ACTIVATION 	3
#define PROGRAM_ACTIVATION 	4

#define TIMESTEP            1

#define SIGMOID_CONSTANTS       3 /* Beta, Theta, Tau */
#define GAIN_CONSTANTS          1 /* One g per neuron */
#define THETA_POSITION_CONSTANT 0 /* Position of BETA on chromosome */
#define BETA_POSITION_CONSTANT  1 /* Position of BETA on chromosome */
#define TAU_POSITION_CONSTANT   2 /* Position of TAU on chromosome */
/******************************************************************************/
/******************************************************************************/
class CCTRNNLayerController 
{
public:
	CCTRNNLayerController(const char* pch_name, CEpuck* pc_epuck, 
									 unsigned int number_of_sensory_inputs, 
									 unsigned int number_of_layer_inputs, 
									 unsigned int number_of_outputs,
									 unsigned int un_activation_function,
									 unsigned int un_label,
									 double f_lower_bounds,
									 double f_upper_bounds);
	virtual ~CCTRNNLayerController();

	/*virtual void SetWeights(unsigned int un_number_of_weights, */
	/*double* pf_weights); */
	virtual double* ComputeOutputs(double* pf_layer_inputs, double* pf_sensory_inputs, double* pf_weight_matrix);

	virtual void Reset();

	unsigned int GetNumberOfSensoryInputs ( void );
	unsigned int GetNumberOfLayerInputs ( void );
	unsigned int GetNumberOfOutputs ( void );
	unsigned int GetLabel ( void );

protected:
	unsigned int m_unNumberOfSensoryInputs;
	unsigned int m_unNumberOfLayerInputs;
	unsigned int m_unNumberOfOutputs;
	unsigned int m_unActivationFunction;
	unsigned int m_unLabel;
	
	double*  m_pfOutputs;
	double*  m_pfStates;
	
	double   m_fLowerBounds;
	double	 m_fUpperBounds;

	unsigned int m_unRequiredNumberOfWeights;
};


/******************************************************************************/
/******************************************************************************/


#endif
