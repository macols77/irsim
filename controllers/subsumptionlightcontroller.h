#ifndef SUBSUMPTIONLIGHTCONTROLLER_H_
#define SUBSUMPTIONLIGHTCONTROLLER_H_

/******************************************************************************/
/******************************************************************************/

#include "controller.h"

/******************************************************************************/
/******************************************************************************/

class CSubsumptionLightController : public CController
{
public:

    CSubsumptionLightController (const char* pch_name, CEpuck* pc_epuck, int n_write_to_file);
    ~CSubsumptionLightController();
    void SimulationStep(unsigned n_step_number, double f_time, double f_step_interval);

private:
		/* ROBOT */
    CEpuck* m_pcEpuck;
   
	 	/* SENSORS */
		CWheelsActuator* m_acWheels;
    CEpuckProximitySensor* m_seProx;
		CLightSensor* m_seLight;
		CContactSensor* m_seContact;
		CGroundSensor* m_seGround;
		CGroundMemorySensor* m_seGroundMemory;
		CBatterySensor* m_seBattery;   

		/* Global Variables */
		double 		m_fLeftSpeed;
		double 		m_fRightSpeed;
		double**	m_fActivationTable;
		double 		m_fTime;

		int m_nWriteToFile;
		/* Functions */

		void ExecuteBehaviors ( void );
		void Coordinator ( void );

		void ObstacleAvoidance ( unsigned int un_priority );
		void Navigate ( unsigned int un_priority );
		void GoLoad ( unsigned int un_priority );
};

#endif
