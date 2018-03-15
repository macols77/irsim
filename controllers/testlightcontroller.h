#ifndef TESTLIGHTCONTROLLER_H_
#define TESTLIGHTCONTROLLER_H_


/******************************************************************************/
/******************************************************************************/

#include "controller.h"

/******************************************************************************/
/******************************************************************************/

class CTestLightController : public CController
{
public:

		/* Constructor */
    CTestLightController (const char* pch_name, CEpuck* pc_epuck);

		/* Destructor */
    ~CTestLightController();

		/*Simulation step */
    void SimulationStep(unsigned n_step_number, double f_time, double f_step_interval);

private:
    CWheelsActuator* m_acWheels;
		CRealLightSensor* m_seLight;
    CEpuck* m_pcEpuck;
};

#endif
