#ifndef COLLISIONPUCK_H_
#define COLLISIONPUCK_H_

/******************************************************************************/
/******************************************************************************/

class CCollisionPuck;

/******************************************************************************/
/******************************************************************************/

#include "puck.h"
#include "compoundcollisionobject.h"
#include "circlecollisionobject.h"
#include "rectanglecollisionobject.h"

#include <math.h>
#include <vector>
#include <list>

using namespace std;

/******************************************************************************/
/******************************************************************************/

class CCollisionPuck : public CPuck
{
public:
    CCollisionPuck(const char* pch_name, double f_xpos, double f_ypos, double f_rotation);
    virtual ~CCollisionPuck();

    virtual void SimulationStep(unsigned n_step_number, double f_time, double f_step_interval);
    virtual CCollisionObject* GetCollisionObject();

	//Method added to force the repositioning of the collision object (otherwise sometimes the epuck
	//could be in a different position with respect to its collision object)
	virtual void UpdateCollisionPosition();

protected:
    CCompoundCollisionObject*   m_pcCompoundCollisionObject;
    CCircleCollisionObject*     m_pcBodyCollisionObject;
};

/******************************************************************************/
/******************************************************************************/

#endif
