#ifndef RECTANGLECOLLISIONOBJECT_H
#define RECTANGLECOLLISIONOBJECT_H

/******************************************************************************/
/******************************************************************************/

#include "collisionobject.h"

#include <math.h>
#include <vector>
#include <list>

using namespace std;

/******************************************************************************/
/******************************************************************************/

class CRectangleCollisionObject : public CCollisionObject
{
public:
    CRectangleCollisionObject(const char* pch_name, CGeometry* pc_parent, 
                              double f_relative_pos_x, double f_relative_pos_y, 
                              double f_relative_rotation, double f_size_x,
                              double f_size_y);
	~CRectangleCollisionObject();
    virtual void ComputeNewPositionAndRotationFromParent();
    virtual int  GetCollisionObjectType();
    
    virtual void SetSize(double f_size_x, double f_size_y);

    double GetHalfSizeX();
    double GetHalfSizeY();

    // This checks collision:
    bool CheckCollisionWithRectangle(CRectangleCollisionObject* pc_rectangle);


protected:
    // This checks collision in one direction only:
    bool CheckHalfCollisionWithRectangle(CRectangleCollisionObject* pc_rectangle);

protected:
    dVector2 m_vCornerX1Y1;
    dVector2 m_vCornerX2Y1;
    dVector2 m_vCornerX2Y2;
    dVector2 m_vCornerX1Y2;      

    double m_fHalfSizeX;
    double m_fHalfSizeY;

    double m_fRelativeRotation;
};

/******************************************************************************/
/******************************************************************************/

#endif
