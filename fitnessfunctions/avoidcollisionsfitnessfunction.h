#ifndef AVOIDCOLLISIONSFITNESSFUNCTION_H_
#define AVOIDCOLLISIONSFITNESSFUNCTION_H_

/******************************************************************************/
/******************************************************************************/

#include <math.h>
#include <vector>
#include <list>
#include "general.h"
using namespace std;

class CAvoidCollisionsFitnessFunction;

#include "fitnessfunction.h"
#include "simulator.h"
#include "sensor.h"
#include "batterysensor.h"
#include "contactsensor.h"

/******************************************************************************/
/******************************************************************************/

class CAvoidCollisionsFitnessFunction : public CFitnessFunction
{
public:
    CAvoidCollisionsFitnessFunction(const char* pch_name, CSimulator* pc_simulator,
                                    unsigned int un_collisions_allowed_per_epuck);
    virtual double GetFitness();
		virtual void SimulationStep(unsigned int n_simulation_step, double f_time, double f_step_interval);

protected:
		unsigned int m_unNumberOfSteps;
		double 			m_fComputedFitness;
		CEpuck* m_pcEpuck;
		CBatterySensor* m_seBattery;

		unsigned int m_unCollisionsNumber;

};

/******************************************************************************/
/******************************************************************************/

#endif
