#ifndef COMPOUNDCOLLISIONOBJECT_H
#define COMPOUNDCOLLISIONOBJECT_H
/******************************************************************************/
/******************************************************************************/

//OBJECT WHICH CAN BE COMPOSED OF OTHER COLLISION OBJECTS, TO CREATE COMPLEX
//COLLISION STRUCTURES

/******************************************************************************/
/******************************************************************************/

#include "collisionobject.h"

#include <math.h>
#include <vector>
#include <list>

using namespace std;

/******************************************************************************/
/******************************************************************************/

class CAxisAlignedBoundingBox;

/******************************************************************************/
/******************************************************************************/

class CCompoundCollisionObject : public CCollisionObject
{
public:
    CCompoundCollisionObject(const char* pch_name, CGeometry* pc_parent, 
                             double f_relative_pos_x, double f_relative_pos_y);

    virtual ~CCompoundCollisionObject();

    virtual void AddCollisionChild(CCollisionObject* pc_new_child);
    virtual TCollisionObjectVector* GetCollisionChildren();

    virtual void ComputeNewPositionAndRotationFromParent();
    virtual int  GetCollisionObjectType();
    virtual void DeleteChildren();
    
protected:
    TCollisionObjectVector      m_vecChildren;
};

/******************************************************************************/
/******************************************************************************/


#endif
