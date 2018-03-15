#ifndef TESTPROXCONTROLLER_H_
#define TESTPROXCONTROLLER_H_

/******************************************************************************/
/******************************************************************************/

#include "controller.h"

/******************************************************************************/
/******************************************************************************/

class CTestProxController : public CController
{
public:

    CTestProxController (const char* pch_name, CEpuck* pc_epuck);
    ~CTestProxController();
    void SimulationStep(unsigned n_step_number, double f_time, double f_step_interval);

private:
    CWheelsActuator* m_acWheels;
    CEpuckProximitySensor* m_seProx;
    CEpuck* m_pcEpuck;
    
};

#endif
