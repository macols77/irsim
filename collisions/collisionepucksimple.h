#ifndef COLLISIONEPUCKSIMPLE_H_
#define COLLISIONEPUCKSIMPLE_H_

/******************************************************************************/
/******************************************************************************/

class CCollisionEpuckSimple;

/******************************************************************************/
/******************************************************************************/

#include "collisionepuck.h"
#include "circlecollisionobject.h"

/******************************************************************************/
/******************************************************************************/

class CCollisionEpuckSimple : public CCollisionEpuck 
{
public:
    CCollisionEpuckSimple(const char* pch_name, double f_xpos, double f_ypos, double f_rotation);
    virtual ~CCollisionEpuckSimple();

    virtual void SimulationStep(unsigned n_step_number, double f_time, double f_step_interval);
protected:
    CCircleCollisionObject*        m_pcBodyCollisionObject;
};

/******************************************************************************/
/******************************************************************************/

#endif
