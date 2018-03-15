#ifndef PUCK_H
#define PUCK_H

/******************************************************************************/
/******************************************************************************/

#include <math.h>
#include <vector>
#include <list>

using namespace std;

/******************************************************************************/
/******************************************************************************/

class CPuck;

typedef vector<CPuck*>              TPuckVector;
typedef vector<CPuck*>::iterator    TPuckIterator;

/******************************************************************************/
/******************************************************************************/

#include "epuck.h"
#include "geometry.h"
#include "simulator.h"

/******************************************************************************/
/******************************************************************************/

class CPuck : public CGeometry
{
public:
    CPuck(const char* pch_name, double f_posx, double f_posy, double f_rotation);
	~CPuck();
    virtual void Draw(CRender* pc_render);

	//Collision stuff
	void SetCollisions(unsigned int);
	void AddCollision();
	unsigned int GetCollisions();
	
    void SimulationStep(unsigned int n_simulation_step, 
                        double f_time, 
                        double f_step_interval);


    virtual void    SetGrippingEpuck(CEpuck* pc_epuck);    
    virtual CEpuck* GetGrippingEpuck();    
        
    // Get the initial rotation of the puck:
    virtual double  GetInitialRotation();
    virtual void    SetInitialRotation(double f_initial_rotation);

    // Get/Set color of LEDs
    virtual void         SetColor(float f_red, float f_green, float f_blue);
    virtual unsigned int GetAllColoredLeds();
    virtual void         SetAllColoredLeds(unsigned int un_color);
    virtual bool         SomeColoredLedIsOn();
    virtual unsigned int GetColoredLed(unsigned int un_index);
    virtual void         SetColoredLed(unsigned int un_index, unsigned int un_color);


    // Some constants for the geometry of the pucks:
    static double CHASSIS_RADIUS;

    // Default color of a puck
    static float DEFAULT_COLOR_RED;
    static float DEFAULT_COLOR_GREEN;
    static float DEFAULT_COLOR_BLUE;
	
protected:
    static double   PUCK_MASS;

protected:
    float           m_fColorRed;
    float           m_fColorGreen;
    float           m_fColorBlue;

    double          m_fInitialRotation;
    
    bool            m_bIsGripped;
    CEpuck*         m_pcGrippingEpuck;

    int		        m_punColoredLeds[8];

	unsigned int    m_unCollisionCount;
	
};

/******************************************************************************/
/******************************************************************************/

#endif
