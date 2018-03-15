#include "collisionpuck.h"
#include "collisionmanager.h"
#include "circlecollisionobject.h"

/******************************************************************************/
/******************************************************************************/

CCollisionPuck::CCollisionPuck(const char* pch_name, double f_xpos, double f_ypos, double f_rotation) :
    CPuck(pch_name, f_xpos, f_ypos, f_rotation)
{
    char pchTemp[1024];
    sprintf(pchTemp, "%s_body_co", pch_name);
    m_pcBodyCollisionObject = new CCircleCollisionObject(pchTemp, this, 0, 0, 1.02*CPuck::CHASSIS_RADIUS);
    m_pcCompoundCollisionObject = new CCompoundCollisionObject(pchTemp, this, 0, 0);
    GetCollisionManager()->AddCollisionObject(m_pcCompoundCollisionObject);
    m_pcCompoundCollisionObject->AddCollisionChild(m_pcBodyCollisionObject);
}

/******************************************************************************/
/******************************************************************************/

CCollisionPuck::~CCollisionPuck()
{
    delete m_pcCompoundCollisionObject;
}

/******************************************************************************/
/******************************************************************************/

void CCollisionPuck::SimulationStep(unsigned n_step_number, double f_time, double f_step_interval)
{
	//executing the step
    CPuck::SimulationStep(n_step_number, f_time, f_step_interval);	
}

/******************************************************************************/
/******************************************************************************/

CCollisionObject* CCollisionPuck::GetCollisionObject()
{
    return m_pcCompoundCollisionObject;
}

/******************************************************************************/
/******************************************************************************/

void CCollisionPuck::UpdateCollisionPosition(){
	m_pcCompoundCollisionObject->ComputeNewPositionAndRotationFromParent();
}
