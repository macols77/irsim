#include "arguments.h"
#include <stdio.h>
#include <string>

/******************************************************************************/
/******************************************************************************/

CArguments::CArguments(const char* pch_argumentlist)
{
    if (pch_argumentlist == NULL)
    {
        m_pchArgumentList = NULL;
    } else {        
        m_pchArgumentList = new char[strlen(pch_argumentlist) + 2];
        strcpy(m_pchArgumentList, pch_argumentlist);
    }
}

/******************************************************************************/
/******************************************************************************/

CArguments::~CArguments()
{
    delete[] m_pchArgumentList;
}

/******************************************************************************/
/******************************************************************************/

const char* CArguments::GetArgument(const char* pch_indicator)
{
    if (m_pchArgumentList == NULL)
    {
        return NULL;
    }

    char pchTempTag[1024];
    strcpy(pchTempTag, pch_indicator);
    //    strcat(pchTempTag, "=");

    int nParameterStringLength = strlen(m_pchArgumentList);
    int nCurrentIndex          = 0;
    int nTagLength             = strlen(pchTempTag);
   
    bool bTagFound = false;

    while (nCurrentIndex + nTagLength <= nParameterStringLength && !bTagFound)
    {
        if (strncmp(&m_pchArgumentList[nCurrentIndex], pchTempTag, nTagLength) == 0)
        {
            if (nCurrentIndex + nTagLength + 1 >= nParameterStringLength  ||
                m_pchArgumentList[nCurrentIndex + nTagLength] == '=' || 
                m_pchArgumentList[nCurrentIndex + nTagLength] == ',')
            {

                if (nCurrentIndex == 0)
                    bTagFound = true;
                else if (m_pchArgumentList[nCurrentIndex-1] == ',')
                    bTagFound = true;
                else
                    nCurrentIndex++;                
            } else {
                nCurrentIndex++;
            }
        } else {
            nCurrentIndex++;
        }
    }

    if (!bTagFound)
    {
        return NULL;
    }
    else 
    {        
        nCurrentIndex += nTagLength;
        if (m_pchArgumentList[nCurrentIndex] == '=')
        {
            nCurrentIndex++;
        }

        int nValueIndex = 0;
        while (m_pchArgumentList[nCurrentIndex] != ',' && nCurrentIndex < nParameterStringLength)
        {
            m_pchArgumentValue[nValueIndex++] = m_pchArgumentList[nCurrentIndex++];
        }

        m_pchArgumentValue[nValueIndex] = '\0';
    }
       
    return m_pchArgumentValue;
}

/******************************************************************************/
/******************************************************************************/

bool CArguments::GetArgumentIsDefined(const char* pch_indicator)
{
    if (GetArgument(pch_indicator) == NULL)
    {
        return false;
    } else {
        return true;
    }   
}

/******************************************************************************/
/******************************************************************************/

int  CArguments::GetArgumentAsInt(const char* pch_indicator)
{
    const char* pchArg = GetArgument(pch_indicator);
    return atoi(pchArg == NULL ? "0" : pchArg);
}

/******************************************************************************/
/******************************************************************************/

double CArguments::GetArgumentAsDouble(const char* pch_indicator)
{
    const char* pchArg = GetArgument(pch_indicator);
    if (pchArg != NULL)
        return atof(GetArgument(pch_indicator));
    else
        return 0.0;

}

/******************************************************************************/
/******************************************************************************/

const char* CArguments::GetArgumentAsString(const char* pch_indicator)
{
    return GetArgument(pch_indicator);
}

/******************************************************************************/
/******************************************************************************/

const char* CArguments::GetCompleteString() const
{
    return (const char*) m_pchArgumentList;
}

/******************************************************************************/
/******************************************************************************/
