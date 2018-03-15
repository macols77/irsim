#ifndef TESTENCODERCONTROLLER_H_
#define TESTENCODERCONTROLLER_H_


/******************************************************************************/
/******************************************************************************/

#include "controller.h"

/******************************************************************************/
/******************************************************************************/

class CTestEncoderController : public CController
{
public:

		/* Constructor */
    CTestEncoderController (const char* pch_name, CEpuck* pc_epuck);

		/* Destructor */
    ~CTestEncoderController();

		/*Simulation step */
    void SimulationStep(unsigned n_step_number, double f_time, double f_step_interval);

private:
    CWheelsActuator* m_acWheels;
		CEncoderSensor* m_seEncoder;
    CEpuck* m_pcEpuck;

    double m_fOrientation;
    dVector2  m_vPosition;
};

#endif
