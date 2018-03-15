#include "compoundcollisionobject.h"
#include "axisalignedboundingbox.h"

/******************************************************************************/
/******************************************************************************/

CCompoundCollisionObject::CCompoundCollisionObject(const char* pch_name, CGeometry* pc_parent, 
                                                   double f_relative_pos_x, double f_relative_pos_y) :
    CCollisionObject(pch_name, pc_parent, f_relative_pos_x, f_relative_pos_y)
{
}

/******************************************************************************/
/******************************************************************************/

CCompoundCollisionObject::~CCompoundCollisionObject()
{
}

/******************************************************************************/
/******************************************************************************/

//Updates object position and children's one

void CCompoundCollisionObject::ComputeNewPositionAndRotationFromParent()
{
    if (m_pcParent != NULL)
    {
        dVector2 vNewPos = m_vRelativePosition;
        dVec2Rotate(m_pcParent->GetRotation(), vNewPos);
        dVec2Add(vNewPos, m_pcParent->GetPosition(), vNewPos);
        SetPosition(vNewPos);
        SetRotation(m_pcParent->GetRotation());
    
        m_cAABB.Reset(vNewPos.x, vNewPos.y, 0, 0); 
        for (int i = 0; i < m_vecChildren.size(); i++)
        {
            if (m_vecChildren[i]->IsEnabled())
            {
                m_vecChildren[i]->ComputeNewPositionAndRotationFromParent();
                m_cAABB.Add(m_vecChildren[i]->GetAABB());
            }
        }
    } else {
        dVector2 vPosition = m_vecChildren[0]->GetPosition();
        
        SetPosition(vPosition);
        SetRotation(0);

        m_cAABB.Reset(vPosition.x, vPosition.y, 0, 0);


        for (int i = 0; i < m_vecChildren.size(); i++)
        {
            if (m_vecChildren[i]->IsEnabled())
            {
                m_cAABB.Add(m_vecChildren[i]->GetAABB());
            }
        }
    }
}


/******************************************************************************/
/******************************************************************************/

void CCompoundCollisionObject::AddCollisionChild(CCollisionObject* pc_new_child)
{
    m_vecChildren.push_back(pc_new_child);
}

/******************************************************************************/
/******************************************************************************/

int CCompoundCollisionObject::GetCollisionObjectType()
{
    return COLLISION_OBJECT_TYPE_COMPOUND;
}

/******************************************************************************/
/******************************************************************************/

TCollisionObjectVector* CCompoundCollisionObject::GetCollisionChildren()
{
    return &m_vecChildren;
}

/******************************************************************************/
/******************************************************************************/

void CCompoundCollisionObject::DeleteChildren()
{
    for (int i = 0; i < m_vecChildren.size(); i++)
    {
        delete m_vecChildren[i];
    }
}

/******************************************************************************/
/******************************************************************************/
