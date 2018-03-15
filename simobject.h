/******************************************************************************

This is the super-class for almost all twodee classes. It is quite
simple. It simply contains a couple of public accessible methods. Its
main feature is that it uses recursive parent-child
relationships. This allows for events such as keypresses, a new
simulation cycle, and rendering to be done recursively. 

It also allows all objects to have distinct names - and invaluable
tool for debugging.

*******************************************************************************/

#ifndef SIMOBJECT_H_
#define SIMOBJECT_H_

/******************************************************************************/
/******************************************************************************/

#include <math.h>
#include <vector>
#include <list>

#include "general.h"
using namespace std;
/******************************************************************************/
/******************************************************************************/

class CRender;
class CSimObject;

typedef vector<CSimObject*>           TSimObjectsVector;
typedef vector<CSimObject*>::iterator TSimObjectsIterator;

/******************************************************************************/
/******************************************************************************/

class CSimObject 
{
public:
    CSimObject(const char* pch_name);
    virtual ~CSimObject();
    const char* GetName() const;

    virtual void Draw(CRender* pc_render);
    virtual void SimulationStep(unsigned int n_step_number, double f_time, double f_step_interval);
    virtual void Keypressed(int keycode);

    virtual void AddChild(CSimObject* pc_child);
    virtual void RemoveChild(CSimObject* pc_child);

    virtual void PrintfChildren(unsigned indent);

    virtual TSimObjectsVector* GetChildren();

protected:
    char*             m_pchName;
    
    TSimObjectsVector m_vecSimObjectChildren;

};

/******************************************************************************/
/******************************************************************************/


#endif
