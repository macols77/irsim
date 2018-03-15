#ifndef WHEELSACTUATOR_H_
#define WHEELSACTUATOR_H_

/******************************************************************************/
/******************************************************************************/

//#include "common.h"
class CWheelsActuator;

#include "actuator.h"

#define MAX_REAL_SPEED 1000
#define MIN_REAL_SPEED -1000

/******************************************************************************/
/******************************************************************************/

class CWheelsActuator : public CActuator
{
public:
    CWheelsActuator(const char* pch_name, CEpuck* pc_epuck);
    
    // 0 = left, 1 = right
    virtual void SetOutput(unsigned int un_input_index, double f_value);
    
    virtual void SetSpeed(double left, double right);
    
    virtual unsigned int GetType();
    
protected:
    double m_fSpeedVector[MAX_REAL_SPEED - MIN_REAL_SPEED + 1];
};


/******************************************************************************/
/******************************************************************************/

#endif

