#ifndef TESTGROUNDEXP_H
#define TESTGROUNDEXP_H

/******************************************************************************/
/******************************************************************************/

#include "experiment.h"

/******************************************************************************/
/******************************************************************************/

class CTestGroundExp : public CExperiment
{
public:
    CTestGroundExp ( const char* pch_name , const char* paramsFile );
		~CTestGroundExp ( void );
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

		/* Preys */
		dVector2* preyCenter;
		double* preyExternalRadius;
		double * preyInternalRadius;
		double * preyColor;

};

/******************************************************************************/
/******************************************************************************/

#endif
