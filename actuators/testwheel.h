#ifndef TESTWHEEL_H_
#define TESTWHEEL_H_

/******************************************************************************/
/******************************************************************************/

class CTestWheel;

#include "actuator.h"
#include "random.h"

#define MAX_REAL_SPEED 1000
#define MIN_REAL_SPEED -1000

/******************************************************************************/
/******************************************************************************/

class CTestWheel : public CActuator
{
public:
    CTestWheel(const char* pch_name, CEpuck* pc_epuck);
    
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
