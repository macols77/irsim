#ifndef LIGHTFITNESSFUNCTION_H_
#define LIGHTFITNESSFUNCTION_H_

/******************************************************************************/
/******************************************************************************/

#include <math.h>
#include <vector>
#include <list>
#include "general.h"
using namespace std;

class CLightFitnessFunction;

#include "fitnessfunction.h"
#include "simulator.h"

/******************************************************************************/
/******************************************************************************/

class CLightFitnessFunction : public CFitnessFunction
{
public:
    CLightFitnessFunction(const char* pch_name, CSimulator* pc_simulator,
                                    unsigned int un_collisions_allowed_per_epuck);
    virtual double GetFitness();
		virtual void SimulationStep(unsigned int n_simulation_step, double f_time, double f_step_interval);

protected:
		unsigned int m_unNumberOfSteps;
		double 			m_fComputedFitness;
		CEpuck* m_pcEpuck;
    unsigned int m_unBlueLightFlag;
    unsigned int m_unVirtualCounter;

};

/******************************************************************************/
/******************************************************************************/

#endif
