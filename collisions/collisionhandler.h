#ifndef COLLISIONHANDLER_H
#define COLLISIONHANDLER_H

#include "collisionepuck.h"
#include "collisionpuck.h"
#include "collisionepucksimple.h"
#include "collisionmanager.h"
#include "rectanglecollisionobject.h"
#include "circlecollisionobject.h"
#include "ringcollisionobject.h"

#define RESET_COLLISION_HANLDER 0
#define BOUNCE_COLLISION_HANDLER 1

using namespace std;

/******************************************************************************/
// Functions to handle collisions (repositioning algorithms)
/******************************************************************************/

void ResetToPreviousPosition(CCollisionManager*,CCollisionEpuck*,dVector2);

void BounceCollision(CCollisionManager*,TEpuckVector*,int);
void BounceCollision(CCollisionManager*,TPuckVector*,int);

#endif
