#ifndef COLLISIONOBJECT_H_
#define COLLISIONOBJECT_H_

/******************************************************************************/
/******************************************************************************/

class CCollisionObject;

#include <math.h>
#include <vector>
#include <list>

using namespace std;

typedef vector<CCollisionObject*>              TCollisionObjectVector;
typedef vector<CCollisionObject*>::iterator    TCollisionObjectIterator;

/******************************************************************************/
/******************************************************************************/

#include "geometry.h"
#include "axisalignedboundingbox.h"
#include "collisioncallback.h"


/******************************************************************************/
/******************************************************************************/

#define COLLISION_OBJECT_TYPE_COMPOUND   0
#define COLLISION_OBJECT_TYPE_RECTANGLE  1
#define COLLISION_OBJECT_TYPE_CIRCLE     2
#define COLLISION_OBJECT_TYPE_RING       3

/******************************************************************************/
/******************************************************************************/

class CCollisionObject : public CGeometry
{
public:
    CCollisionObject(const char* pch_name, CGeometry* pc_parent, 
                     double f_relative_pos_x, double f_relative_pos_y);
	~CCollisionObject();
    virtual void Enable();
    virtual void Disable();
    virtual bool IsEnabled();

    virtual void SetRelativePosition(double f_x, double f_y);   
    virtual void SetRelativePosition(dVector2 v_position);   

    virtual void GetRelativePosition(double* pf_x, double* pf_y);   
    virtual dVector2 GetRelativePosition();
    
    virtual void ComputeNewPositionAndRotationFromParent() = 0;
    virtual int GetCollisionObjectType()                   = 0;

    virtual void RegisterCollisionCallback(CCollisionCallback* pc_callback, void* pv_aux); 
    virtual void CollisionOccurred(CCollisionObject* pc_with_object);
	
    virtual CAxisAlignedBoundingBox* GetAABB();

    CGeometry*	     GetParent();
    virtual void     SetParent(CGeometry* pc_parent);

    CGeometry*	     GetFounderParent();

protected:
    bool                     m_bEnabled;
    CGeometry*               m_pcParent;

    dVector2                 m_vRelativePosition;
    CAxisAlignedBoundingBox  m_cAABB;

    CCollisionCallback*      m_pcCollisionCallback;
    void*                    m_pvCollisionCallbackAux;
};

/******************************************************************************/
/******************************************************************************/

#endif
