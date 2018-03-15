#ifndef NNCONTROLLER_H_
#define NNCONTROLLER_H_

/******************************************************************************/
/******************************************************************************/

#include <math.h>
#include <vector>
#include <list>

#include <iostream>
#include <sys/types.h>
#include <fstream>

using namespace std;

class CNNController;

/******************************************************************************/
/******************************************************************************/

#include "controller.h"

/******************************************************************************/
/******************************************************************************/

class CNNController : public CController
{
public:
    CNNController(const char* pch_name, CEpuck* pc_epuck);
    virtual ~CNNController();
    virtual double* LoadWeights(const char* pch_filename);
	//virtual void LoadWeights(const char* pch_filename);
    virtual void SimulationStep(unsigned n_step_number, double f_time, double f_step_interval);


    // A neural network controller needs only to implement the 
    // following two methods:
    virtual void SetWeights(unsigned int un_number_of_weights, 
                            double* pf_genes) = 0; 

    virtual const double* ComputeOutputs(double* pf_inputs) = 0;
	
	//Added mainly for the recurrent neural networks, to reset neuron's states
	virtual void Reset()=0;

	virtual void SaveState(const char* pch_filename);
	virtual void LoadState(const char* pch_filename);

protected:
    unsigned int GetNumberOfSensorInputs();
    unsigned int GetNumberOfActuatorOutputs();

protected:
    unsigned int m_unNumberOfSensorInputs;
    unsigned int m_unNumberOfActuatorOutputs;

    double*      m_pfInputs;
};

/******************************************************************************/
/******************************************************************************/

#endif
