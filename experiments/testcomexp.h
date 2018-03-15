#ifndef TESTCOMEXP_H
#define TESTCOMEXP_H

/******************************************************************************/
/******************************************************************************/

#include "experiment.h"

/******************************************************************************/
/******************************************************************************/

class CTestComExp : public CExperiment
{
public:
    CTestComExp(const char* pch_name, const char* paramsFile);

protected:
    // Overwritten from the superclasses:
    CArena* CreateArena();
		void AddActuators(CEpuck* pc_epuck);
    void AddSensors(CEpuck* pc_epuck);
    void SetController(CEpuck* pc_epuck);
    void CreateAndAddEpucks(CSimulator* pc_simulator);
    
private:
    int m_nRobotsNumber;
};

/******************************************************************************/
/******************************************************************************/

#endif
