#include "circlecollisionobject.h"

/******************************************************************************/
/******************************************************************************/

CCircleCollisionObject::CCircleCollisionObject(const char* pch_name, CGeometry* pc_parent, 
                                               double f_relative_pos_x, double f_relative_pos_y, 
                                               double f_radius) : 
    CCollisionObject(pch_name, pc_parent, f_relative_pos_x, f_relative_pos_y), m_fRadius(f_radius)
{
    ComputeNewPositionAndRotationFromParent();
}



/******************************************************************************/
/******************************************************************************/

CCircleCollisionObject::~CCircleCollisionObject(){
}

/******************************************************************************/
/******************************************************************************/
void CCircleCollisionObject::ComputeNewPositionAndRotationFromParent()
{
    SetPosition(m_pcParent->GetPosition().x + m_vRelativePosition.x,
                m_pcParent->GetPosition().y + m_vRelativePosition.y);

    m_cAABB.Reset(m_pcParent->GetPosition().x + m_vRelativePosition.x, 
                  m_pcParent->GetPosition().y + m_vRelativePosition.y, 
                  m_fRadius * 2, m_fRadius * 2);
}

/******************************************************************************/
/******************************************************************************/

int CCircleCollisionObject::GetCollisionObjectType()
{
    return COLLISION_OBJECT_TYPE_CIRCLE;
}

/******************************************************************************/
/******************************************************************************/

double CCircleCollisionObject::GetRadius()
{
    return m_fRadius;
}

/******************************************************************************/
/******************************************************************************/

void CCircleCollisionObject::SetRadius (double r)
{
    m_fRadius = r;
}

/******************************************************************************/
/******************************************************************************/
