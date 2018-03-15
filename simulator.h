#ifndef SIMULATOR_H
#define SIMULATOR_H

/******************************************************************************/
/******************************************************************************/

class CSimulator;

/******************************************************************************/
/******************************************************************************/

#include "simobject.h"
#include "arena.h"
#include "epuck.h"
#include "puck.h"

/******************************************************************************/
/******************************************************************************/

class CSimulator : public CSimObject
{
public:
    CSimulator(const char* pch_name);
    ~CSimulator();

    static CSimulator* GetInstance();


    // Set the arena:
    virtual void SetArena(CArena* pc_arena);

    // Get the arena:
    virtual CArena* GetArena();

    // Set the time interval between consecutive steps:
    virtual void SetStepInterval(double f_step_interval);
    virtual double GetStepInterval() const;

    // Take one step of the simulation:
    virtual void TakeSimulationStep();
    
    // End the simulation:
    virtual void EndSimulation();
    
    // Check if the simulation has ended:
    virtual bool HasEnded();

    // Returns the time in the virtual world in seconds:
    virtual double GetTime();

    // Returns the number of the current controlstep:
    virtual unsigned int GetControlStepNumber();

    // Set the time limit in simulation time (time in the
    // virtual world):
    virtual void   SetTimeLimit(double f_time_limit);
    virtual double GetTimeLimit();


    // Add an epuck :
    virtual void AddEpuck(CEpuck* pc_epuck);    
	virtual void RemoveEpuck(CEpuck* pc_epuck); //Removes given epuck from the simulation (robot identified by name)
    virtual TEpuckVector* GetEpucks();
	// Add a puck :
    virtual void AddPuck(CPuck* pc_puck);    
    virtual TPuckVector* GetPucks();
	
	virtual void SetCollisionModelPresent(int i_collision_model_present);
    virtual int GetCollisionModelPresent();

	virtual void SetCollisionHandler(int i_collision_handler);
	virtual int GetCollisionHandler();

	virtual void ResetSimulation();

	
protected:
    static double DEFAULT_STEP_INTERVAL;
    static CSimulator*   m_cInstance;

    bool                 m_bSimulationEnded;

    double               m_fTimeLimit;
    double               m_fTime;
    double               m_fStepInterval;
    unsigned int         m_unStep;

    TEpuckVector      m_vecEpucks;
    TPuckVector       m_vecPucks;
//    TSwarmBotVector      m_vecSwarmBots;
 //   TStoyVector		     m_vecStoys;

//    TSbotVector          m_vecSbotsAttemptingGrip;
//    TSbotVector          m_vecSbotsReleasingGrip;

    CArena*              m_pcArena;
    
    int                   m_iCollisionModelPresent;
	int					  m_iCollisionHandler;
};

/******************************************************************************/
/******************************************************************************/

#endif


//    virtual void RemoveSwarmBot(CSwarmBot* pc_swarmbot);    
//    virtual TSwarmBotVector* GetSwarmBots();

    // Add a swarmbot:
  //  virtual void AddSwarmBot(CSwarmBot* pc_swarmbot);    
  //  virtual void RemoveSwarmBot(CSwarmBot* pc_swarmbot);    
  //  virtual TSwarmBotVector* GetSwarmBots();

    // Add an stoy:
  //  virtual void AddStoy(CStoy* pc_stoy);
 //   virtual void RemoveStoy(CStoy* pc_stoy);
  //  virtual TStoyVector* GetStoys();

    // When an s-bot attempts to grip or when it releases something,
    // it should call these functions:
 //   virtual void AttemptGrip(CSbot* pc_sbot);
 //   virtual void ReleaseGrip(CSbot* pc_sbot);

//protected:
//    virtual void HandleGrippingAttempts();
 //   virtual void HandleGripReleases();
//    virtual void SplitSwarmBot(CSbot* pc_releasing_sbot);
//    virtual void JoinSwarmBots(CSwarmBot* pc_swarmbot1, CSwarmBot* pc_swarmbot2);
