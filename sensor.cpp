#include "sensor.h"

/******************************************************************************/
/******************************************************************************/

char  CSensor::m_pchSensorReadingsString[10240];

/******************************************************************************/
/******************************************************************************/

CSensor::CSensor(const char* pch_name, unsigned int un_no_outputs) : CSimObject(pch_name), m_unNumberOfInputs(un_no_outputs)
{
    if (m_unNumberOfInputs > 0)
        m_pfInputs =(double*)  malloc(sizeof(double) * m_unNumberOfInputs);
    else
        m_pfInputs = NULL;
}

/******************************************************************************/
/******************************************************************************/

CSensor::~CSensor()
{
    if (m_pfInputs)
        free(m_pfInputs);
}

/******************************************************************************/
/******************************************************************************/

unsigned int CSensor::GetNumberOfInputs()
{
    return m_unNumberOfInputs;
}


/******************************************************************************/
/******************************************************************************/

void CSensor::SetInput(unsigned int un_index, double f_value)
{
    if (un_index < m_unNumberOfInputs)
        m_pfInputs[un_index] = f_value;
    else{
            printf("Input out of %d >= %d", un_index, m_unNumberOfInputs);
            fflush(stdout);
        }
}

/******************************************************************************/
/******************************************************************************/

double* CSensor::GetInputs()  
{
    return m_pfInputs;
}

/******************************************************************************/
/******************************************************************************/

double CSensor::GetInput(unsigned int un_index)  
{
    return m_pfInputs[un_index];
}

/******************************************************************************/
/******************************************************************************/

char* CSensor::SensorReadingsToString(unsigned int un_number_of_readings, 
                                      double* pf_readings)
{
    unsigned int unStrLen = 0;

    for (int i = 0; i < un_number_of_readings; i++)
    {
        sprintf(&m_pchSensorReadingsString[unStrLen], "[%2d: %2.6f] ", i, pf_readings[i]);
        unStrLen += strlen(&m_pchSensorReadingsString[unStrLen]);
    }    

    return &m_pchSensorReadingsString[0];
}


/******************************************************************************/
/******************************************************************************/

double* CSensor::GetComputedSensorReadings()
{
	return m_pfInputs;
    //return GetInputs();
}

/******************************************************************************/
/******************************************************************************/


//float CSensor::NormalizeAngle( float angle)
//{
	//if (angle < 0.0 )
		//return fmod(angle, 2 * M_PI) + 2 * M_PI;
	//else 
		//return fmod(angle, 2*M_PI);
//}
