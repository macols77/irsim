#ifndef CIRCLECOLLISIONOBJECT_H
#define CIRCLECOLLISIONOBJECT_H

/******************************************************************************/
/******************************************************************************/

#include "collisionobject.h"

#include <math.h>
#include <vector>
#include <list>

using namespace std;

/******************************************************************************/
/******************************************************************************/

//A round collision object

class CCircleCollisionObject : public CCollisionObject
{
public:
    CCircleCollisionObject(const char* pch_name, CGeometry* pc_parent, 
                           double f_relative_pos_x, double f_relative_pos_y,
                           double f_radius);
	~CCircleCollisionObject();
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
