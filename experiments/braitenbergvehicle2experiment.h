#ifndef BRAITENBERGVEHICLE2_H
#define BRAITENBERGVEHICLE2_H

/******************************************************************************/
/******************************************************************************/

#include "experiment.h"

/******************************************************************************/
/******************************************************************************/

class CBraitenbergVehicle2Experiment : public CExperiment
{
public:
    CBraitenbergVehicle2Experiment ( const char* pch_name , const char* paramsFile );
		~CBraitenbergVehicle2Experiment ( void );
protected:
    // Overwritten from the superclasses:
    CArena* CreateArena();
		void AddActuators(CEpuck* pc_epuck);
    void AddSensors(CEpuck* pc_epuck);
    void SetController(CEpuck* pc_epuck);
    void CreateAndAddEpucks(CSimulator* pc_simulator);
   	void Reset(); 
private:

		/* VARIABLES*/
		/* Extra */
		int m_nRobotsNumber;
		int m_nWriteToFile;
		dVector2* m_pcvRobotPositions;
		double* m_fRobotOrientations;
		int m_nRunTime;

		/* Environment */
		int m_nLightObjectNumber;
		dVector2 *m_pcvLightObjects;
		
		int m_nBlueLightObjectNumber;
		dVector2 *m_pcvBlueLightObjects;
		
		int m_nRedLightObjectNumber;
		dVector2 *m_pcvRedLightObjects;
		
		int m_nNumberOfGroundArea;
		dVector2* m_vGroundAreaCenter;
		double* m_fGroundAreaExternalRadius;
		double * m_fGroundAreaInternalRadius;
		double * m_fGroundAreaColor;

		/* Morphology */

		/* Sensors */
		float m_fLightSensorRange;
		float m_fBlueLightSensorRange;
		float m_fRedLightSensorRange;
		
		double m_fBatterySensorRange;
		double m_fBatteryChargeCoef;
		double m_fBatteryDischargeCoef;
		
		double m_fBlueBatterySensorRange;
		double m_fBlueBatteryChargeCoef;
		double m_fBlueBatteryDischargeCoef;
		
		double m_fRedBatterySensorRange;
		double m_fRedBatteryChargeCoef;
		double m_fRedBatteryDischargeCoef;
};

/******************************************************************************/
/******************************************************************************/

#endif
