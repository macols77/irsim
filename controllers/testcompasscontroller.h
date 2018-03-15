#ifndef TESTCOMPASSCONTROLLER_H_
#define TESTCOMPASSCONTROLLER_H_


/******************************************************************************/
/******************************************************************************/

#include "controller.h"

/******************************************************************************/
/******************************************************************************/

class CTestCompassController : public CController
{
public:

		/* Constructor */
    CTestCompassController (const char* pch_name, CEpuck* pc_epuck);

		/* Destructor */
    ~CTestCompassController();

		/*Simulation step */
    void SimulationStep(unsigned n_step_number, double f_time, double f_step_interval);

private:
    CWheelsActuator* m_acWheels;
		CEncoderSensor* m_seEncoder;
		CCompassSensor* m_seCompass;
    CEpuck* m_pcEpuck;

    double    m_fOrientation;
    dVector2  m_vPosition;
    dVector2  maxErrorA;
};

#endif
