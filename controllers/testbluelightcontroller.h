#ifndef TESTBLUELIGHTCONTROLLER_H_
#define TESTBLUELIGHTCONTROLLER_H_


/******************************************************************************/
/******************************************************************************/

#include "controller.h"

/******************************************************************************/
/******************************************************************************/

class CTestBlueLightController : public CController
{
public:

		/* Constructor */
    CTestBlueLightController (const char* pch_name, CEpuck* pc_epuck);

		/* Destructor */
    ~CTestBlueLightController();

		/*Simulation step */
    void SimulationStep(unsigned n_step_number, double f_time, double f_step_interval);

private:
    CWheelsActuator* m_acWheels;
		CRealBlueLightSensor* m_seLight;
    CEpuck* m_pcEpuck;
};

#endif
