#ifndef PROJECTIONACTUATOR_H_
#define PROJECTIONACTUATOR_H_

/******************************************************************************/
/******************************************************************************/

//#include "common.h"
class CProjectionActuator;

#include "actuator.h"

/******************************************************************************/
/******************************************************************************/

class CProjectionActuator : public CActuator
{
public:
    CProjectionActuator(const char* pch_name, CEpuck* pc_epuck);
    
    // 0 = left, 1 = right
		virtual void SetOutput(unsigned int un_input_index, double f_value);
    
		/*virtual void SetSpeed(double left, double right);*/
    
    virtual unsigned int GetType();
		virtual void SetNestProjection ( int state );
		virtual void SetPreyProjection ( int state );
    
protected:
		/*double m_fSpeedVector[MAX_REAL_SPEED - MIN_REAL_SPEED + 1];*/
};


/******************************************************************************/
/******************************************************************************/

#endif

