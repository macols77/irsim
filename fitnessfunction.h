#ifndef FITNESSFUNCTION_H_
#define FITNESSFUNCTION_H_


/******************************************************************************/
/******************************************************************************/

class CFitnessFunction;

#include <math.h>
#include <vector>
#include <list>

using namespace std;

#include "simobject.h"
#include "epuck.h"
#include "simulator.h"

/******************************************************************************/
/******************************************************************************/

class CFitnessFunction : public CSimObject
{
public:
//    CFitnessFunction(const char* pch_name, CSimulator* pc_simulator, CArguments* pc_fitness_function_arguments);
    CFitnessFunction(const char* pch_name, CSimulator* pc_simulator);
    virtual ~CFitnessFunction();
    
    virtual double GetFitness() = 0;   
		virtual void SimulationStep(unsigned int n_simulation_step, double f_time, double f_step_interval);

protected:
    CSimulator*       m_pcSimulator;
//    CArguments*       m_pcFitnessFunctionArguments;
};

/******************************************************************************/
/******************************************************************************/

#endif
