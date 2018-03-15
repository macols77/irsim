#ifndef TESTWHEELSCONTROLLER_H_
#define TESTWHEELSCONTROLLER_H_


/******************************************************************************/
/******************************************************************************/

#include "controller.h"

/******************************************************************************/
/******************************************************************************/

class CTestWheelsController : public CController
{
public:

		/* Constructor */
    CTestWheelsController (const char* pch_name, CEpuck* pc_epuck);

		/* Destructor */
    ~CTestWheelsController();

		/*Simulation step */
    void SimulationStep(unsigned n_step_number, double f_time, double f_step_interval);

private:
    CWheelsActuator* m_acWheels;
    CEpuck* m_pcEpuck;
};

#endif
