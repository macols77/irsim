#ifndef TESTBATTERYEXP_H
#define TESTBATTERYEXP_H

/******************************************************************************/
/******************************************************************************/

#include "experiment.h"

/******************************************************************************/
/******************************************************************************/

class CTestBatteryExp : public CExperiment
{
public:
    CTestBatteryExp ( const char* pch_name , const char* paramsFile );
		~CTestBatteryExp ( void );
protected:
    // Overwritten from the superclasses:
    CArena* CreateArena();
		void AddActuators(CEpuck* pc_epuck);
    void AddSensors(CEpuck* pc_epuck);
    void SetController(CEpuck* pc_epuck);
    void CreateAndAddEpucks(CSimulator* pc_simulator);
   	void Reset(); 
private:
    int m_nRobotsNumber;
		int m_nLightObjectNumber;
		float m_fLightSensorRange;
		dVector2 *m_pcvLightObjects;
};

/******************************************************************************/
/******************************************************************************/

#endif
