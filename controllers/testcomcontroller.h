#ifndef TESTCOMCONTROLLER_H_
#define TESTCOMCONTROLLER_H_


/******************************************************************************/
/******************************************************************************/

#include "controller.h"

/******************************************************************************/
/******************************************************************************/

class CTestComController : public CController
{
public:

		/* Constructor */
    CTestComController (const char* pch_name, CEpuck* pc_epuck);

		/* Destructor */
    ~CTestComController();

		/*Simulation step */
    void SimulationStep(unsigned n_step_number, double f_time, double f_step_interval);

private:
    CWheelsActuator* m_acWheels;
		CComSensor* m_seCom;
    CEpuck* m_pcEpuck;

		int m_nComData;
};

#endif
