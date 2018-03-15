#ifndef TESTSWITCHLIGHTCONTROLLER_H_
#define TESTSWITCHLIGHTCONTROLLER_H_


/******************************************************************************/
/******************************************************************************/

#include "controller.h"

/******************************************************************************/
/******************************************************************************/

class CTestSwitchLightController : public CController
{
public:

		/* Constructor */
    CTestSwitchLightController (const char* pch_name, CEpuck* pc_epuck);

		/* Destructor */
    ~CTestSwitchLightController();

		/*Simulation step */
    void SimulationStep(unsigned n_step_number, double f_time, double f_step_interval);

private:
    CWheelsActuator* m_acWheels;
		CRealLightSensor* m_seLight;
    CEpuck* m_pcEpuck;
};

#endif
