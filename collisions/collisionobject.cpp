#include "collisionobject.h"

/******************************************************************************/
/******************************************************************************/

CCollisionObject::CCollisionObject(const char* pch_name, CGeometry* pc_parent, 
                                   double f_relative_pos_x, double f_relative_pos_y) : 
    CGeometry(pch_name, 0, 0, 0, 0), 
    m_bEnabled(true), 
    m_pcParent(pc_parent),
    m_cAABB(pc_parent == NULL ? f_relative_pos_x : pc_parent->GetPosition().x,  pc_parent == NULL ? f_relative_pos_y : pc_parent->GetPosition().y),
    m_pcCollisionCallback(NULL),
    m_pvCollisionCallbackAux(NULL)
{
    m_vRelativePosition.x = f_relative_pos_x;
    m_vRelativePosition.y = f_relative_pos_y;

    if (pc_parent != NULL)
    {
        SetPosition(m_vRelativePosition.x + m_pcParent->GetPosition().x, m_vRelativePosition.y + m_pcParent->GetPosition().y);
        SetRotation(m_pcParent->GetRotation());
    } 
}

/******************************************************************************/
/******************************************************************************/

CCollisionObject::~CCollisionObject(){
}
/******************************************************************************/
/******************************************************************************/

void CCollisionObject::Enable()
{
    m_bEnabled = true;
}

/******************************************************************************/
/******************************************************************************/
void CCollisionObject::Disable()
{
    m_bEnabled = false;
}

/******************************************************************************/
/******************************************************************************/

bool CCollisionObject::IsEnabled()
{
    return m_bEnabled;
}

/******************************************************************************/
/******************************************************************************/

void CCollisionObject::SetRelativePosition(double f_x, double f_y)
{
    m_vRelativePosition.x = f_x;
    m_vRelativePosition.y = f_y;
}

/******************************************************************************/
/******************************************************************************/

void CCollisionObject::SetRelativePosition(dVector2 v_position)
{
    m_vRelativePosition = v_position;
}

/******************************************************************************/
/******************************************************************************/

void CCollisionObject::GetRelativePosition(double* pf_x, double* pf_y)  
{
    (*pf_x) = m_vRelativePosition.x;
    (*pf_y) = m_vRelativePosition.y;
}

/******************************************************************************/
/******************************************************************************/

dVector2 CCollisionObject::GetRelativePosition()
{
    return m_vRelativePosition;
}
    
/******************************************************************************/
/******************************************************************************/

CAxisAlignedBoundingBox* CCollisionObject::GetAABB()
{
    return &m_cAABB;
}

/******************************************************************************/
/******************************************************************************/

void CCollisionObject::RegisterCollisionCallback(CCollisionCallback* pc_callback, 
                                                 void* pv_aux)
{
    m_pcCollisionCallback     = pc_callback;
    m_pvCollisionCallbackAux  = pv_aux;
}

/******************************************************************************/
/******************************************************************************/

void CCollisionObject::CollisionOccurred(CCollisionObject* pc_with_object)
{
    if (m_pcCollisionCallback != 0)
        m_pcCollisionCallback->CollisionOccurred(this, 
                                                 pc_with_object,
                                                 m_pvCollisionCallbackAux);
}


/******************************************************************************/
/******************************************************************************/
CGeometry* CCollisionObject::GetParent()
{
  return m_pcParent;
}

/******************************************************************************/
/******************************************************************************/

CGeometry* CCollisionObject::GetFounderParent()
{
  CGeometry* pcFounderParent = m_pcParent;
  CCollisionObject* pcCollisionObject = dynamic_cast<CCollisionObject*>(pcFounderParent);
  while( pcCollisionObject != NULL )
    {
      pcFounderParent = pcCollisionObject->GetParent();
      pcCollisionObject = dynamic_cast<CCollisionObject*>(pcFounderParent);
    }
  return pcFounderParent; 
}

/******************************************************************************/
/******************************************************************************/

void CCollisionObject::SetParent(CGeometry* pc_parent)
{
    m_pcParent = pc_parent;
}

/******************************************************************************/
/******************************************************************************/
