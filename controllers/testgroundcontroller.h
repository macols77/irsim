#ifndef TESTGROUNDCONTROLLER_H_
#define TESTGROUNDCONTROLLER_H_


/******************************************************************************/
/******************************************************************************/

#include "controller.h"

/******************************************************************************/
/******************************************************************************/

class CTestGroundController : public CController
{
public:

    CTestGroundController (const char* pch_name, CEpuck* pc_epuck);
    ~CTestGroundController();
    void SimulationStep(unsigned n_step_number, double f_time, double f_step_interval);

private:
    CEpuck* m_pcEpuck;
    
		CWheelsActuator* m_acWheels;
    CEpuckProximitySensor* m_seProx;
		CLightSensor* m_seLight;
		CContactSensor* m_seContact;
		CGroundSensor* m_seGround;
		CGroundMemorySensor* m_seGroundMemory;
    
};

#endif
