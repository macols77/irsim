#include "ringcollisionobject.h"
/******************************************************************************/
/******************************************************************************/

// COLLISION OBJECT FOR THE ROUND ARENA

/******************************************************************************/
/******************************************************************************/

CRingCollisionObject::CRingCollisionObject(const char* pch_name, CGeometry* pc_parent, 
                                               double f_relative_pos_x, double f_relative_pos_y, 
                                               double f_radius) : 
    CCollisionObject(pch_name, pc_parent, f_relative_pos_x, f_relative_pos_y), m_fRadius(f_radius)
{
    ComputeNewPositionAndRotationFromParent();
}

/******************************************************************************/
/******************************************************************************/

void CRingCollisionObject::ComputeNewPositionAndRotationFromParent()
{
    SetPosition(m_pcParent->GetPosition().x + m_vRelativePosition.x,
                m_pcParent->GetPosition().y + m_vRelativePosition.y);

    m_cAABB.Reset(m_pcParent->GetPosition().x + m_vRelativePosition.x, 
                  m_pcParent->GetPosition().y + m_vRelativePosition.y, 
                  m_fRadius * 2, m_fRadius * 2);
}

/******************************************************************************/
/******************************************************************************/

int CRingCollisionObject::GetCollisionObjectType()
{
    return COLLISION_OBJECT_TYPE_RING;
}

/******************************************************************************/
/******************************************************************************/

double CRingCollisionObject::GetRadius()
{
    return m_fRadius;
}

/******************************************************************************/
/******************************************************************************/

void CRingCollisionObject::SetRadius(double r)
{
    m_fRadius = r;
}

/******************************************************************************/
/******************************************************************************/
