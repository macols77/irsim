#include "temporarymessageactuator.h"

/******************************************************************************/
/******************************************************************************/

CTemporaryMessageActuator::CTemporaryMessageActuator(const char* pch_name, CEpuck* pc_epuck) :
    CActuator(pch_name, pc_epuck, 1)
{

}

/******************************************************************************/
/******************************************************************************/

unsigned int CTemporaryMessageActuator::GetType()
{
    return ACTUATOR_TEMPORARY_MESSAGE;
}

/******************************************************************************/
/******************************************************************************/

void CTemporaryMessageActuator::SetMessage(CEpuck* pc_epuck_recip,char* message, double f_time)
{
	sprintf(c_message, message);
	f_Ctime=f_time;
	pc_epuck_Recipient=pc_epuck_recip;
}

/******************************************************************************/
/******************************************************************************/

void CTemporaryMessageActuator::SetOutput(unsigned int un_input_index, double f_value)
{
  
  
}

double CTemporaryMessageActuator::GetCreationTime(void)
{
	return f_Ctime;
}

/******************************************************************************/
/******************************************************************************/
