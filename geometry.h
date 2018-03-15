#ifndef GEOMETRY_H
#define GEOMETRY_H

/******************************************************************************/
/******************************************************************************/

class CGeometry;

class CCollisionManager;

/******************************************************************************/
/******************************************************************************/

#include "general.h"
#include "simobject.h"
#include "simmath.h"
/******************************************************************************/
/******************************************************************************/

#define LED_COLOR_BLACK	 0
#define LED_COLOR_RED	 1
#define LED_COLOR_GREEN	 2
#define LED_COLOR_BLUE	 3
#define LED_COLOR_YELLOW 4
// White light coming from a light source
#define LED_COLOR_WHITE  5   

#define NUM_COLORS 6

/******************************************************************************/
/******************************************************************************/

class CGeometry : public CSimObject
{
public:
    CGeometry(const char* pch_name, double f_pos_x, double f_pos_y, double f_rotation, double f_mass);
    CGeometry(const char* pch_name);
    ~CGeometry();

    virtual void SetPosition(double x, double y);
    virtual void SetPosition(dVector2 v);
    virtual void GetPosition(double* x, double* y);   
    virtual dVector2 GetPosition();   
    
    virtual double SetRotation(double f_rotation);
    virtual double GetRotation();

    virtual void  SetMass(double f_mass);
    virtual double GetMass();

    static void SetCollisionManager(CCollisionManager* pc_collision_manager);

protected:
    static bool AreCollisionsOn();
    static CCollisionManager* GetCollisionManager();

    static CCollisionManager* m_pcCollisionManager;


protected:
    dVector2            m_vPosition;
    double              m_fRotation;                  
    double              m_fMass;
};

/******************************************************************************/
/******************************************************************************/


#endif
