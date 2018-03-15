#include "collisionepuckgripper.h"
#include "collisionmanager.h"
#include "circlecollisionobject.h"

/******************************************************************************/
/******************************************************************************/

CCollisionEpuckGripper::CCollisionEpuckGripper(const char* pch_name, double f_xpos, double f_ypos, double f_rotation) :
    CCollisionEpuck(pch_name, f_xpos, f_ypos, f_rotation)
{
	SetGripperAngle(45/180*M_PI);
    // Body of the epuck :
    char pchTemp[1024]; sprintf(pchTemp, "%s_body_co", pch_name);
    m_pcBodyCollisionObject = new CCircleCollisionObject(pchTemp, this, 0, 0, 1.02*CEpuck::CHASSIS_RADIUS);
	m_pcCompoundCollisionObject->AddCollisionChild(m_pcBodyCollisionObject);
    // Right gripper of the epuck :
    char pchTemp2[1024]; sprintf(pchTemp2, "%s_right_co", pch_name);
    dVector2 radius, half_gripper, center_to_half_gripper_right;
    radius.x = 1.02*CEpuck::CHASSIS_RADIUS * cos(GRIPPER_ANGLE);
    radius.y = - 1.02*CEpuck::CHASSIS_RADIUS * sin(GRIPPER_ANGLE);
    half_gripper.x = GRIPPER_LENGTH/2;
    half_gripper.y = 0.002;
    dVec2Add(center_to_half_gripper_right, radius, half_gripper);
    m_pcGripRightCollisionObject = new CRectangleCollisionObject(pchTemp2, this, center_to_half_gripper_right.x, center_to_half_gripper_right.y, 0, GRIPPER_LENGTH/2, 0.001);
	m_pcCompoundCollisionObject->AddCollisionChild(m_pcGripRightCollisionObject);
    // Left gripper of the epuck :
    char pchTemp3[1024]; sprintf(pchTemp3, "%s_left_co", pch_name);
    dVector2 center_to_half_gripper_left;
    radius.y = 1.02*CEpuck::CHASSIS_RADIUS * sin(GRIPPER_ANGLE);
    dVec2Add(center_to_half_gripper_left, radius, half_gripper);
    m_pcGripLeftCollisionObject = new CRectangleCollisionObject(pchTemp2, this, center_to_half_gripper_left.x, center_to_half_gripper_left.y, 0, GRIPPER_LENGTH/2, 0.001);
	m_pcCompoundCollisionObject->AddCollisionChild(m_pcGripLeftCollisionObject);
    
    /*
    double length = 2*CEpuck::CHASSIS_RADIUS + CEpuck::GRIPPER_LENGTH;
    double width = 2*CEpuck::CHASSIS_RADIUS;
    m_pcBodyCollisionObject = new CRectangleCollisionObject(pchTemp, this, 0, 0, 0, length, width);
    m_pcCompoundCollisionObject->AddCollisionChild(m_pcBodyCollisionObject);
    */
    
    SetGripperPresence(true);
}

/******************************************************************************/
/******************************************************************************/

CCollisionEpuckGripper::~CCollisionEpuckGripper()
{
    delete m_pcBodyCollisionObject;
}

/******************************************************************************/
/******************************************************************************/

void CCollisionEpuckGripper::SimulationStep(unsigned n_step_number, double f_time, double f_step_interval)
{
	//executing the step
    CEpuck::SimulationStep(n_step_number, f_time, f_step_interval);	
}

/******************************************************************************/
/******************************************************************************/
