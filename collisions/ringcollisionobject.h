#ifndef RINGCOLLISIONOBJECT_H
#define RINGCOLLISIONOBJECT_H

/******************************************************************************/
/******************************************************************************/

#include "collisionobject.h"

#include <math.h>
#include <vector>
#include <list>

using namespace std;

/******************************************************************************/
/******************************************************************************/

class CRingCollisionObject : public CCollisionObject
{
public:
    CRingCollisionObject(const char* pch_name, CGeometry* pc_parent, 
                           double f_relative_pos_x, double f_relative_pos_y,
                           double f_radius);

    virtual void ComputeNewPositionAndRotationFromParent();
    virtual int  GetCollisionObjectType();
    double GetRadius();
    void SetRadius(double r);

protected:
    double m_fRadius;

};

/******************************************************************************/
/******************************************************************************/

#endif
