#ifndef PERCEPTRONCONTROLLER_H_
#define PERCEPTRONCONTROLLER_H_

/******************************************************************************/
/******************************************************************************/

#include <math.h>
#include <vector>
#include <list>

using namespace std;

class CPerceptronController;


/******************************************************************************/
/******************************************************************************/

#include "controller.h"
#include "nncontroller.h"

/******************************************************************************/
/******************************************************************************/

class CPerceptronController : public CNNController
{
public:
    //CPerceptronController(const char* pch_name, CSbot* pc_sbot, CArguments* pc_controller_arguments);
    CPerceptronController(const char* pch_name, CEpuck* pc_epuck);
    virtual ~CPerceptronController();

    virtual void SetWeights(unsigned int un_number_of_weights, 
                            double* pf_weights); 
    virtual const double* ComputeOutputs(double* pf_inputs);

	virtual void Reset();

	virtual void SetUpperBounds(float);
	virtual void SetLowerBounds(float);

protected:
    double*  m_pfWeights;
    double*  m_pfOutputs;
	double   m_fLowerBounds;
	double	 m_fUpperBounds;
};


/******************************************************************************/
/******************************************************************************/


#endif
