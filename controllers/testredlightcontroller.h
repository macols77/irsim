#ifndef TESTREDLIGHTCONTROLLER_H_
#define TESTREDLIGHTCONTROLLER_H_


/******************************************************************************/
/******************************************************************************/

#include "controller.h"

/******************************************************************************/
/******************************************************************************/

class CTestRedLightController : public CController
{
public:

		/* Constructor */
    CTestRedLightController (const char* pch_name, CEpuck* pc_epuck);

		/* Destructor */
    ~CTestRedLightController();

		/*Simulation step */
    void SimulationStep(unsigned n_step_number, double f_time, double f_step_interval);

private:
    CWheelsActuator* m_acWheels;
		CRealRedLightSensor* m_seLight;
    CEpuck* m_pcEpuck;
};

#endif
