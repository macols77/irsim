#ifndef COLLISIONEPUCKGRIPPER_H_
#define COLLISIONEPUCKGRIPPER_H_

/******************************************************************************/
/******************************************************************************/

class CCollisionEpuckGripper;

/******************************************************************************/
/******************************************************************************/

#include "epuck.h"
#include "compoundcollisionobject.h"
#include "circlecollisionobject.h"
#include "rectanglecollisionobject.h"
#include "collisionepuck.h"

#include <math.h>
#include <vector>
#include <list>

using namespace std;

/******************************************************************************/
/******************************************************************************/

class CCollisionEpuckGripper : public CCollisionEpuck
{
public:
    CCollisionEpuckGripper(const char* pch_name, double f_xpos, double f_ypos, double f_rotation);
    virtual ~CCollisionEpuckGripper();

    virtual void SimulationStep(unsigned n_step_number, double f_time, double f_step_interval);
    
protected:
    CCircleCollisionObject*  m_pcBodyCollisionObject;
    CRectangleCollisionObject* m_pcGripRightCollisionObject;
    CRectangleCollisionObject* m_pcGripLeftCollisionObject;
};

/******************************************************************************/
/******************************************************************************/

#endif
