#ifndef LOADFITNESSFUNCTION_H_
#define LOADFITNESSFUNCTION_H_

/******************************************************************************/
/******************************************************************************/

#include <math.h>
#include <vector>
#include <list>
#include "general.h"
using namespace std;

class CLoadFitnessFunction;

#include "fitnessfunction.h"
#include "simulator.h"
#include "sensor.h"
#include "batterysensor.h"
#include "bluebatterysensor.h"
#include "redbatterysensor.h"
#include "groundsensor.h"
#include "groundmemorysensor.h"
#include "lightsensor.h"
#include "bluelightsensor.h"
#include "redlightsensor.h"
#include "contactsensor.h"

/******************************************************************************/
/******************************************************************************/

class CLoadFitnessFunction : public CFitnessFunction
{
public:
    CLoadFitnessFunction(const char* pch_name, CSimulator* pc_simulator,
                                    unsigned int un_collisions_allowed_per_epuck);
		~CLoadFitnessFunction();
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
