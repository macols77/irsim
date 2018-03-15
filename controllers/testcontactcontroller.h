#ifndef TESTCONTACTCONTROLLER_H_
#define TESTCONTACTCONTROLLER_H_

/******************************************************************************/
/******************************************************************************/

#include "controller.h"

/******************************************************************************/
/******************************************************************************/

class CTestContactController : public CController
{
public:

    CTestContactController (const char* pch_name, CEpuck* pc_epuck);
    ~CTestContactController();
    void SimulationStep(unsigned n_step_number, double f_time, double f_step_interval);

private:
    CWheelsActuator* m_acWheels;
    CContactSensor* m_seContact;
    CEpuck* m_pcEpuck;
    
};

#endif
