#include "simulator.h"
#include "collisionhandler.h"
#include "experiment.h"


/******************************************************************************/
/******************************************************************************/

CSimulator* CSimulator::m_cInstance = NULL;
double       CSimulator::DEFAULT_STEP_INTERVAL = 0.1;

/******************************************************************************/
/******************************************************************************/



CSimulator::CSimulator(const char* pch_name) :
    CSimObject(pch_name), 
    m_bSimulationEnded(false), 
    m_fTimeLimit(1e10)

{
	
    m_cInstance = this;

    SetStepInterval(DEFAULT_STEP_INTERVAL);
	
    m_fTime = 0;
    m_unStep = 0;
}

/******************************************************************************/
/******************************************************************************/

CSimulator::~CSimulator()
{
    m_cInstance = NULL;
}

/******************************************************************************/
/******************************************************************************/

void CSimulator::TakeSimulationStep()
{
	//printf("Simulator, step no: %d\n",m_unStep);
    vector<dVector2> vOldPositions;
    // We first let the bots sense the world:
    TEpuckIterator j;
    for (j = m_vecEpucks.begin(); j != m_vecEpucks.end(); j++)
    {
		//printf("Calling sense on epuck: %s\n",(*j)->GetName());
        (*j)->Sense(this);
		//printf("After sense\n");
		//printf("Epuck: %s, position: %f--%f\n",(*j)->GetName(),((*j)->GetPosition()).x,((*j)->GetPosition()).y);
		//sleep(1);
		//Save old positions if we have a reset collision handler
		if(GetCollisionModelPresent()!=COLLISION_MODEL_NONE){
			if(GetCollisionHandler()==COLLISION_HANDLER_RESET){
				vOldPositions.push_back((*j)->GetPosition());		
			}
		}
    } 
    
	
    // We then take an action and a simulation step:
		//printf("1\n");
    SimulationStep(m_unStep, m_fTime, m_fStepInterval);
		//printf("2\n");

	//If we have a collision model we handle the collisions
	if(GetCollisionModelPresent()!=COLLISION_MODEL_NONE){
		
		if(GetCollisionHandler()==COLLISION_HANDLER_POSITION){
		    BounceCollision(CCollisionManager::GetInstance(),&m_vecEpucks,m_unStep);
		    BounceCollision(CCollisionManager::GetInstance(),&m_vecPucks,m_unStep);
		}else if(GetCollisionHandler()==COLLISION_HANDLER_RESET){
			
			TEpuckVector* pc_EpuckArr = GetEpucks();
			TEpuckIterator it = (*pc_EpuckArr).begin();
			
			vector<dVector2>::iterator posIt=vOldPositions.begin();
			
			while(it!=(*pc_EpuckArr).end()){
//				printf("Old pos: %f -- %f\n" ,(*posIt).x,(*posIt).y);
				ResetToPreviousPosition(CCollisionManager::GetInstance(),(CCollisionEpuck*)(*it),(*posIt));
//				printf("Cur pos: %f -- %f\n" ,((*it)->GetPosition()).x,((*it)->GetPosition()).y);
				it++;
				posIt++;
			}
		}
	}

	//printf("3\n");

    m_unStep++;
    m_fTime += m_fStepInterval;    
//printf("Simulator time: %f\n",m_fTime);
    if (m_fTime > m_fTimeLimit){
	//	printf("Simulator time expired\n");
        EndSimulation();
	}
		//printf("4\n");
}

/******************************************************************************/
/******************************************************************************/


CSimulator* CSimulator::GetInstance() 
{
    return m_cInstance;
}

/******************************************************************************/
/******************************************************************************/

void CSimulator::SetStepInterval(double f_step_interval)
{
    m_fStepInterval = f_step_interval;
}

/******************************************************************************/
/******************************************************************************/

void CSimulator::SetArena(CArena* pc_arena)
{
    m_pcArena = pc_arena;
    AddChild(pc_arena);
}

/******************************************************************************/
/******************************************************************************/

void CSimulator::AddEpuck(CEpuck* pc_epuck)
{
    m_vecEpucks.push_back(pc_epuck);
    AddChild(pc_epuck);
}

/******************************************************************************/
/******************************************************************************/

void CSimulator::AddPuck(CPuck* pc_puck)
{
    m_vecPucks.push_back(pc_puck);
    AddChild(pc_puck);

}

/******************************************************************************/
/******************************************************************************/

CArena* CSimulator::GetArena()
{ 
    return m_pcArena;
}

/******************************************************************************/
/******************************************************************************/

void CSimulator::EndSimulation()
{
    m_bSimulationEnded = true;       
}

/******************************************************************************/
/******************************************************************************/

bool CSimulator::HasEnded()
{
    return m_bSimulationEnded;
}

/******************************************************************************/
/******************************************************************************/

TEpuckVector* CSimulator::GetEpucks()
{
    return &m_vecEpucks;
}

/******************************************************************************/
/******************************************************************************/

TPuckVector* CSimulator::GetPucks()
{
    return &m_vecPucks;
}

/******************************************************************************/
/******************************************************************************/

double CSimulator::GetTime()
{
    return m_fTime;
}

/******************************************************************************/
/******************************************************************************/

void CSimulator::SetTimeLimit(double f_time_limit)
{
    m_fTimeLimit = f_time_limit;
}

/******************************************************************************/
/******************************************************************************/

double CSimulator::GetTimeLimit()
{
    return m_fTimeLimit;
}

/******************************************************************************/
/******************************************************************************/

double CSimulator::GetStepInterval() const 
{
    return m_fStepInterval;
}


/******************************************************************************/
/******************************************************************************/

unsigned int CSimulator::GetControlStepNumber()
{
    return m_unStep;
}

/******************************************************************************/
/******************************************************************************/

void CSimulator::SetCollisionModelPresent(int i_collision_model_present)
{
    m_iCollisionModelPresent = i_collision_model_present;
}

/******************************************************************************/
/******************************************************************************/

int CSimulator::GetCollisionModelPresent()
{
    return m_iCollisionModelPresent;
}

/******************************************************************************/
/******************************************************************************/

void CSimulator::SetCollisionHandler(int i_collision_handler)
{
    m_iCollisionHandler = i_collision_handler;
}

/******************************************************************************/
/******************************************************************************/

int CSimulator::GetCollisionHandler()
{
    return m_iCollisionHandler;
}

/******************************************************************************/
/******************************************************************************/

void CSimulator::ResetSimulation(){
	//printf("Resetting simualtion., number of robots: %d\n",m_vecEpucks.size());
	//sleep(1);
	m_fTime=0.0;
	m_unStep=0;
	m_bSimulationEnded=false;
}

/******************************************************************************/
/******************************************************************************/

void CSimulator::RemoveEpuck(CEpuck* pc_epuck)
{
    //Actually the epuck itself is not destoyed but just removed from the "list"
    bool removed=false;
    vector<CEpuck*>::iterator it=m_vecEpucks.begin();
    while(!removed && it!=m_vecEpucks.end())
    {
	if(strcmp((*it)->GetName(),pc_epuck->GetName())==0)
	{
	    //printf("Found the epuck %s, collision object name: %s",(*it)->GetName(),(((CCollisionEpuck*)(*it))->GetCollisionObject())->GetName());
	    if((dynamic_cast<CCollisionEpuck*>(*it)))
	    {
		//Also removing collision object from collision manager list
		CCollisionManager::GetInstance()->RemoveCollisionObject(((CCollisionEpuck*)(*it))->GetCollisionObject());
	    }
	    m_vecEpucks.erase(it);
	    RemoveChild(pc_epuck);
	    removed=true;
	}
	it++;	
    }
}

/******************************************************************************/
/******************************************************************************/
