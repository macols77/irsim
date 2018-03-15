#include "projectionactuator.h"

/******************************************************************************/
/******************************************************************************/

CProjectionActuator::CProjectionActuator(const char* pch_name, CEpuck* pc_epuck) :
    CActuator(pch_name, pc_epuck, 1)
{

}

/******************************************************************************/
/******************************************************************************/

unsigned int CProjectionActuator::GetType()
{
    return ACTUATOR_PROJECTION;
}

/******************************************************************************/
/******************************************************************************/

void CProjectionActuator::SetOutput(unsigned int un_input_index, double f_value)
{
}

/******************************************************************************/
/******************************************************************************/

void CProjectionActuator::SetNestProjection ( int state )
{
	CEpuck* m_pcEpuck = GetEpuck(); 
	m_pcEpuck->SetNestProjection(state);
}

/******************************************************************************/
/******************************************************************************/

void CProjectionActuator::SetPreyProjection ( int state )
{
	CEpuck* m_pcEpuck = GetEpuck(); 
	m_pcEpuck->SetPreyProjection(state);
}

