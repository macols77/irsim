#ifndef COLLISIONMANAGER_H_
#define COLLISIONMANAGER_H_


/******************************************************************************/
/******************************************************************************/

//This class knows the geometry of objects and deals with compenetrations
//which define collisions

/******************************************************************************/
/******************************************************************************/

class CCollisionManager;
class CArena;

#include "collisionobject.h"
#include "axisalignedboundingbox.h"

#include <math.h>
#include <vector>
#include <list>

using namespace std;

/******************************************************************************/
/******************************************************************************/

class CCollisionManager : public CSimObject
{
public:
    CCollisionManager(char* pch_name, CArena* pc_arena);
    virtual ~CCollisionManager();

    virtual void AddCollisionObject(CCollisionObject* pc_collision_object);
    virtual void RemoveCollisionObject(CCollisionObject* pc_collision_object);

    virtual bool CheckCollision(CCollisionObject* pc_collision_object, bool b_count_collisions = true);
    TCollisionObjectVector CalculateAndGetCollisionVector(CCollisionObject* pc_collision_object, bool b_count_collisions = true);

	//"Overloaded" method which returns a vector of collision objects instead of a vector representing the collision point
    TCollisionObjectVector CalculateAndGetCollisionsWithWalls2(CCollisionObject* pc_collision_object);
    vector<dVector2> CalculateAndGetCollisionsWithWalls(CCollisionObject* pc_collision_object);

    virtual unsigned int GetTotalNumberOfCollisions();
    vector<dVector2> CalculateAndGetCollisionsWithWallsDetailed(CCollisionObject* pc_collision_object);

	virtual void Reset();
	//Added because the bounce collision model generates "fake" collisions

    static CCollisionManager* GetInstance();

protected:
    bool CheckCollisionBetweenTwoObjects(CCollisionObject* pc_collision_object1,
                                         CCollisionObject* pc_collision_object2);

    static CCollisionManager* m_pcInstance;
    TCollisionObjectVector m_vecCollisionObjects;

    CArena* m_pcArena;

    unsigned int m_unNumberOfCollisionsDetected;
};

/******************************************************************************/
/******************************************************************************/

#endif
