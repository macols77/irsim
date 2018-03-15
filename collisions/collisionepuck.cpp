#include "collisionepuck.h"
#include "collisionmanager.h"
#include "circlecollisionobject.h"

/******************************************************************************/
/******************************************************************************/
/**

This is the abstract epuck collision object which just contains a compound collision
object without any children
   
**/
/******************************************************************************/
/******************************************************************************/


CCollisionEpuck::CCollisionEpuck(const char* pch_name, double f_xpos, double f_ypos, double f_rotation) :
    CEpuck(pch_name, f_xpos, f_ypos, f_rotation)
{
    char pchTemp[1024];
    sprintf(pchTemp, "%s_compound_co", pch_name);
    m_pcCompoundCollisionObject = new CCompoundCollisionObject(pchTemp, this, 0, 0);
    GetCollisionManager()->AddCollisionObject(m_pcCompoundCollisionObject);
    
    SetGripperPresence(false);
}

/******************************************************************************/
/******************************************************************************/

CCollisionEpuck::~CCollisionEpuck()
{
    delete m_pcCompoundCollisionObject;
}

/******************************************************************************/
/******************************************************************************/

void CCollisionEpuck::SimulationStep(unsigned n_step_number, double f_time, double f_step_interval)
{
	//executing the step
    CEpuck::SimulationStep(n_step_number, f_time, f_step_interval);	
}

/******************************************************************************/
/******************************************************************************/

CCollisionObject* CCollisionEpuck::GetCollisionObject()
{
    return m_pcCompoundCollisionObject;
}

/******************************************************************************/
/******************************************************************************/

void CCollisionEpuck::UpdateCollisionPosition(){
	m_pcCompoundCollisionObject->ComputeNewPositionAndRotationFromParent();
}
