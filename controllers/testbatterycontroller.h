#ifndef TESTBATTERYCONTROLLER_H_
#define TESTBATTERYCONTROLLER_H_


/******************************************************************************/
/******************************************************************************/

#include "controller.h"

/******************************************************************************/
/******************************************************************************/

class CTestBatteryController : public CController
{
public:

    CTestBatteryController (const char* pch_name, CEpuck* pc_epuck);
    ~CTestBatteryController();
    void SimulationStep(unsigned n_step_number, double f_time, double f_step_interval);

private:
    CEpuck* m_pcEpuck;
    
		CWheelsActuator* m_acWheels;
    CEpuckProximitySensor* m_seProx;
		CRealLightSensor* m_seLight;
		CContactSensor* m_seContact;
		CGroundSensor* m_seGround;
		CGroundMemorySensor* m_seGroundMemory;
		CBatterySensor* m_seBattery;   

		int m_dStatus;
};

#endif
