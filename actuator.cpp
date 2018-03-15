#include "actuator.h"

/******************************************************************************/
/******************************************************************************/

//CActuator::CActuator(const char* pch_name, CSBot* pc_sbot, unsigned int un_number_of_inputs) : 
//    CSimObject(pch_name), m_unNumberOfOutputs(un_number_of_inputs), m_pcSBot(pc_sbot)

CActuator::CActuator(const char* pch_name, CEpuck* pc_epuck, unsigned int un_number_of_inputs) : 
    CSimObject(pch_name), m_unNumberOfOutputs(un_number_of_inputs), m_pcEpuck(pc_epuck)
{
}

/******************************************************************************/
/******************************************************************************/

unsigned int CActuator::GetNumberOfOutputs()
{
    return m_unNumberOfOutputs;
}

/******************************************************************************/
/******************************************************************************/

CEpuck* CActuator::GetEpuck()
{
    return m_pcEpuck;
}

/******************************************************************************/
/******************************************************************************/

