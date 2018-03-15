#include "collisionepucksimple.h"
#include "collisionhandler.h"
#include "collisionmanager.h"

#include<stdio.h> 
/******************************************************************************/
/******************************************************************************/
/**

The simple e-puck collision object is a compound collision object consisting of:

- A circle representing the e-puck's body.
   
**/
/******************************************************************************/
/******************************************************************************/


CCollisionEpuckSimple::CCollisionEpuckSimple(const char* pch_name, double f_xpos, double f_ypos, double f_rotation) :
    CCollisionEpuck(pch_name, f_xpos, f_ypos, f_rotation)
{
    char pchTemp[1024];
    sprintf(pchTemp, "%s_body_co", pch_name);
    m_pcBodyCollisionObject = new CCircleCollisionObject(pchTemp, this, 0, 0, 1.02*CEpuck::CHASSIS_RADIUS);
	//printf("Epuck collision obj radius: %f\n",1.02*CEpuck::CHASSIS_RADIUS);
	//m_pcBodyCollisionObject = new CCircleCollisionObject(pchTemp, this, 0, 0, CEpuck::CHASSIS_RADIUS);
    m_pcCompoundCollisionObject->AddCollisionChild(m_pcBodyCollisionObject);
    
    SetGripperPresence(false);
}



/******************************************************************************/
/******************************************************************************/

CCollisionEpuckSimple::~CCollisionEpuckSimple()
{
    delete m_pcBodyCollisionObject;
}

/******************************************************************************/
/******************************************************************************/

void CCollisionEpuckSimple::SimulationStep(unsigned n_step_number, double f_time, double f_step_interval)
{
	
    CCollisionEpuck::SimulationStep(n_step_number, f_time, f_step_interval);
    m_pcCompoundCollisionObject->ComputeNewPositionAndRotationFromParent();

}

/******************************************************************************/
/******************************************************************************/
