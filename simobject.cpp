#include "simobject.h"

/******************************************************************************/
/******************************************************************************/

CSimObject::CSimObject(const char* pch_name) 
{
    if (pch_name != NULL)
    {
        m_pchName = (char*) malloc(strlen(pch_name) + 1);
        strcpy(m_pchName, pch_name);
    } else {
        m_pchName = NULL;
    }
}

/******************************************************************************/
/******************************************************************************/


CSimObject::~CSimObject() 
{
    TSimObjectsIterator i = m_vecSimObjectChildren.begin();
    for (i = m_vecSimObjectChildren.begin(); i != m_vecSimObjectChildren.end(); i++)
    {        
        delete (*i);
    }


   if (m_pchName) 
        free(m_pchName);
}

/******************************************************************************/
/******************************************************************************/

const char* CSimObject::GetName() const
{
    return m_pchName;
}

/******************************************************************************/
/******************************************************************************/

void CSimObject::Draw(CRender* pc_render)
{
    TSimObjectsIterator i = m_vecSimObjectChildren.begin();

    for (i = m_vecSimObjectChildren.begin(); i != m_vecSimObjectChildren.end(); i++)
    {
        (*i)->Draw(pc_render);
    }
}

/******************************************************************************/
/******************************************************************************/

void CSimObject::SimulationStep(unsigned int n_step_number, 
                               double f_time, 
                               double f_step_interval)
{

    for (int i = 0; i < m_vecSimObjectChildren.size(); i++)
    {
        m_vecSimObjectChildren[i]->SimulationStep(n_step_number, f_time, f_step_interval);
    }
}

/******************************************************************************/
/******************************************************************************/

void CSimObject::Keypressed(int keycode)
{
    TSimObjectsIterator i = m_vecSimObjectChildren.begin();

    for (i = m_vecSimObjectChildren.begin(); i != m_vecSimObjectChildren.end(); i++)
    {
        (*i)->Keypressed(keycode);
    }
}

/******************************************************************************/
/******************************************************************************/

void CSimObject::AddChild(CSimObject* pc_child)
{
    m_vecSimObjectChildren.push_back(pc_child);
}

/******************************************************************************/
/******************************************************************************/

void CSimObject::RemoveChild(CSimObject* pc_child)
{
    TSimObjectsIterator i = m_vecSimObjectChildren.begin();

    while (i != m_vecSimObjectChildren.end() && (*i) != pc_child)
        i++;

    if (i == m_vecSimObjectChildren.end())
    {
        printf("%s tried to remove a non-existing child %s", GetName(), pc_child->GetName());
        fflush(stdout);
    } else {
        m_vecSimObjectChildren.erase(i);
    }
}

/******************************************************************************/
/******************************************************************************/

void CSimObject::PrintfChildren(unsigned indent)
{
    for (int j = 0; j < indent; j++)
        printf(" ");

    if (m_pchName)
        printf("%s\n", GetName());
    else
        printf("NULL\n", GetName());
        
    TSimObjectsIterator i = m_vecSimObjectChildren.begin();
    for (i = m_vecSimObjectChildren.begin(); i != m_vecSimObjectChildren.end(); i++)
    {
        (*i)->PrintfChildren(indent + 2);
    }
}

/******************************************************************************/
/******************************************************************************/

TSimObjectsVector* CSimObject::GetChildren()
{
    return &m_vecSimObjectChildren;
}

/******************************************************************************/
/******************************************************************************/
