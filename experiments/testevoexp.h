#ifndef TESTEVOEXP_H
#define TESTEVOEXP_H

/******************************************************************************/
/******************************************************************************/

#include "experiment.h"

/******************************************************************************/
/******************************************************************************/

class CTestEvoExp : public CExperiment
{
public:
    CTestEvoExp(const char* pch_name, const char* paramsFile);

protected:
    // Overwritten from the superclasses:
    virtual CArena* CreateArena();
		virtual void AddActuators(CEpuck* pc_epuck);
    virtual void AddSensors(CEpuck* pc_epuck);
    virtual void SetController(CEpuck* pc_epuck);
    virtual void CreateAndAddEpucks(CSimulator* pc_simulator);
   	void Reset(); 
private:
    int m_nRobotsNumber;
		float m_fUpperBounds,m_fLowerBounds;

};

/******************************************************************************/
/******************************************************************************/

#endif
