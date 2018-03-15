#ifndef TEMPORARYMESSAGEACTUATOR_H_
#define TEMPORARYMESSAGEACTUATOR_H_

/******************************************************************************/
/******************************************************************************/

//#include "common.h"
class CTemporaryMessageActuator;

#include "actuator.h"

#define MESSAGE_TIME_LIFE 10*100

/******************************************************************************/
/******************************************************************************/

class CTemporaryMessageActuator : public CActuator
{
public:
    CTemporaryMessageActuator(const char* pch_name, CEpuck* pc_epuck);
    
    // 0 = left, 1 = right
    virtual void SetOutput(unsigned int un_input_index, double f_value);
    
	virtual void SetMessage(CEpuck* pc_epuck_recip, char* message, double f_time);
    virtual double GetCreationTime();
    virtual unsigned int GetType();
    
protected:
	char c_message[128];
	CEpuck* pc_epuck_Recipient;
	double f_Ctime;
    
};


/******************************************************************************/
/******************************************************************************/

#endif

