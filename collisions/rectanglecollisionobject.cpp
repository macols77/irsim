#include "rectanglecollisionobject.h"

/*******************************************************************************

The corners of the collision object are organized in the following way 
when rotation is 0:
                          ^
                          |
          X1,Y1...........|..........X2,Y1
          .               |              .
          .               |              .
         --------------------------------->
           .              |              .
           .              |              .  
          X1,Y2...........|..........X2,Y2
                          |


*******************************************************************************/


/******************************************************************************/
/******************************************************************************/

CRectangleCollisionObject::CRectangleCollisionObject(const char* pch_name, CGeometry* pc_parent, 
                                                     double f_relative_pos_x, double f_relative_pos_y, 
                                                     double f_relative_rotation, double f_size_x,
                                                     double f_size_y) : 
    CCollisionObject(pch_name, pc_parent, f_relative_pos_x, f_relative_pos_y), 
    m_fRelativeRotation(f_relative_rotation), 
    m_fHalfSizeX(f_size_x / 2), 
    m_fHalfSizeY(f_size_y / 2)
{
    if (pc_parent != NULL)
    {
        ComputeNewPositionAndRotationFromParent();
    }
    else 
    {        
        m_vCornerX1Y1.x = f_relative_pos_x - m_fHalfSizeX;
        m_vCornerX1Y1.y = f_relative_pos_y + m_fHalfSizeY;

        m_vCornerX2Y1.x = f_relative_pos_x + m_fHalfSizeX;
        m_vCornerX2Y1.y = f_relative_pos_y + m_fHalfSizeY;

        m_vCornerX2Y2.x = f_relative_pos_x + m_fHalfSizeX;
        m_vCornerX2Y2.y = f_relative_pos_y - m_fHalfSizeY;

        m_vCornerX1Y2.x = f_relative_pos_x - m_fHalfSizeX;
        m_vCornerX1Y2.y = f_relative_pos_y - m_fHalfSizeY;

        m_cAABB.Reset(f_relative_pos_x, f_relative_pos_y, f_size_x, f_size_y);
        SetPosition(f_relative_pos_x, f_relative_pos_y);
        SetRotation(f_relative_rotation);
    }
}

/******************************************************************************/
/******************************************************************************/

CRectangleCollisionObject::~CRectangleCollisionObject(){
//	printf("Destroying rectangle collision obj: %s\n",GetName());
}

/******************************************************************************/
/******************************************************************************/


void CRectangleCollisionObject::ComputeNewPositionAndRotationFromParent()
{                               
    // First we rotate the rectangle and afterwards we move it:
    double fRotation = m_fRelativeRotation + m_pcParent->GetRotation();    
    fRotation = NormalizeAngle(fRotation);
    SetRotation(fRotation);
    
    // Based on the rotation of two points we can find the other two: 
    m_vCornerX1Y1.x = -m_fHalfSizeX;
    m_vCornerX1Y1.y = -m_fHalfSizeY;
    
    m_vCornerX2Y1.x = m_fHalfSizeX;
    m_vCornerX2Y1.y = -m_fHalfSizeY;

    dVec2Rotate(fRotation, m_vCornerX1Y1);
    dVec2Rotate(fRotation, m_vCornerX2Y1);


    m_vCornerX2Y2.x = -m_vCornerX1Y1.x;
    m_vCornerX2Y2.y = -m_vCornerX1Y1.y;

    m_vCornerX1Y2.x = -m_vCornerX2Y1.x;
    m_vCornerX1Y2.y = -m_vCornerX2Y1.y;

    // Calcuate the size of the AABB:       
    double fAABBSizeX = fabs(m_vCornerX2Y2.x) > fabs(m_vCornerX2Y1.x) ? fabs(m_vCornerX2Y2.x) * 2 : fabs(m_vCornerX2Y1.x) * 2;
    double fAABBSizeY = fabs(m_vCornerX2Y2.y) > fabs(m_vCornerX2Y1.y) ? fabs(m_vCornerX2Y2.y) * 2 : fabs(m_vCornerX2Y1.y) * 2;
        
    // Now move the rectangle:
    dVector2 vNewPos = m_vRelativePosition;
    dVec2Rotate(m_pcParent->GetRotation(), vNewPos);
   
    dVec2Add(vNewPos, vNewPos, m_pcParent->GetPosition());
    SetPosition(vNewPos);

    dVec2Add(m_vCornerX1Y1, m_vCornerX1Y1, vNewPos);
    dVec2Add(m_vCornerX2Y1, m_vCornerX2Y1, vNewPos);
    dVec2Add(m_vCornerX2Y2, m_vCornerX2Y2, vNewPos);
    dVec2Add(m_vCornerX1Y2, m_vCornerX1Y2, vNewPos);
    
    m_cAABB.Reset(vNewPos.x, vNewPos.y, fAABBSizeX, fAABBSizeY);
}

/******************************************************************************/
/******************************************************************************/

int CRectangleCollisionObject::GetCollisionObjectType()
{
    return COLLISION_OBJECT_TYPE_RECTANGLE;
}

/******************************************************************************/
/******************************************************************************/

double CRectangleCollisionObject::GetHalfSizeX()
{
    return m_fHalfSizeX;
}

/******************************************************************************/
/******************************************************************************/

double CRectangleCollisionObject::GetHalfSizeY()
{
    return m_fHalfSizeY;
}

/******************************************************************************/
/******************************************************************************/

bool CRectangleCollisionObject::CheckCollisionWithRectangle(CRectangleCollisionObject* pc_rectangle)
{
    if (CheckHalfCollisionWithRectangle(pc_rectangle))
        return true;
    else
        return pc_rectangle->CheckHalfCollisionWithRectangle(this);
}


/******************************************************************************/
/******************************************************************************/

#define CHECK_CORNER(v) (v.x <= m_fHalfSizeX && v.x >= -m_fHalfSizeX && v.y <= m_fHalfSizeY && v.y >= -m_fHalfSizeY)    

// We check if pc_rectangle has a corner within "this" rectangle:
bool CRectangleCollisionObject::CheckHalfCollisionWithRectangle(CRectangleCollisionObject* pc_rectangle)
{
    dVector2 vCornerX1Y1 = pc_rectangle->m_vCornerX1Y1;
    dVector2 vCornerX2Y1 = pc_rectangle->m_vCornerX2Y1;
    dVector2 vCornerX2Y2 = pc_rectangle->m_vCornerX2Y2;
    dVector2 vCornerX1Y2 = pc_rectangle->m_vCornerX1Y2;      

    dVector2 vMyPos = GetPosition();

    dVec2Sub(vCornerX1Y1, vMyPos, vCornerX1Y1);
    dVec2Sub(vCornerX1Y2, vMyPos, vCornerX1Y2);
    dVec2Sub(vCornerX2Y2, vMyPos, vCornerX2Y2);
    dVec2Sub(vCornerX2Y1, vMyPos, vCornerX2Y1);

    double fMyRot = GetRotation();

    dVec2Rotate(-fMyRot, vCornerX1Y1);
    dVec2Rotate(-fMyRot, vCornerX1Y2);
    dVec2Rotate(-fMyRot, vCornerX2Y2);
    dVec2Rotate(-fMyRot, vCornerX2Y1);

    return CHECK_CORNER(vCornerX1Y1) ||
        CHECK_CORNER(vCornerX1Y2) || 
        CHECK_CORNER(vCornerX2Y2) || 
        CHECK_CORNER(vCornerX2Y1);
}

/******************************************************************************/
/******************************************************************************/

void CRectangleCollisionObject::SetSize(double f_size_x, double f_size_y)
{
    m_fHalfSizeX = f_size_x / 2;
    m_fHalfSizeY = f_size_y / 2;

    ComputeNewPositionAndRotationFromParent();

}

/******************************************************************************/
/******************************************************************************/
