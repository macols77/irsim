#ifndef COLLISIONCALLBACK_H_
#define COLLISIONCALLBACK_H_

/******************************************************************************/
/******************************************************************************/

#include <math.h>
#include <vector>
#include <list>

using namespace std;

#include "collisionobject.h"

/******************************************************************************/
/******************************************************************************/


class CCollisionCallback 
{
public:
    virtual void CollisionOccurred(CCollisionObject* pc_object1, 
                                   CCollisionObject* pc_object2,
                                   void* pv_aux_info) {};
};

/******************************************************************************/
/******************************************************************************/

#endif
