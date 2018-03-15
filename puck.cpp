#include "puck.h"
#include "render.h"

/******************************************************************************/
/******************************************************************************/

#define INFINITE 100000

double CPuck::CHASSIS_RADIUS     = 0.02;

double CPuck::PUCK_MASS               = 0.3;

float CPuck::DEFAULT_COLOR_RED       = 0;
float CPuck::DEFAULT_COLOR_GREEN     = 1;
float CPuck::DEFAULT_COLOR_BLUE      = 1;

/******************************************************************************/
/******************************************************************************/

CPuck::CPuck(const char* pch_name, double f_xpos, double f_ypos, double f_rotation) :
    CGeometry(pch_name, f_xpos, f_ypos, f_rotation, PUCK_MASS), m_fColorRed(DEFAULT_COLOR_RED), m_fColorGreen(DEFAULT_COLOR_GREEN), m_fColorBlue(DEFAULT_COLOR_BLUE), m_fInitialRotation(f_rotation), m_pcGrippingEpuck(NULL)
{
    SetAllColoredLeds(LED_COLOR_BLACK);
	m_unCollisionCount=0;
}

/******************************************************************************/
/******************************************************************************/

CPuck::~CPuck(){
}

/******************************************************************************/
/******************************************************************************/

void CPuck::Draw(CRender* pc_render)
{
    pc_render->DrawPuck(this, m_fColorRed, m_fColorGreen, m_fColorBlue);
    CSimObject::Draw(pc_render);
}

/******************************************************************************/
/******************************************************************************/

void CPuck::SetCollisions(unsigned int num){
		m_unCollisionCount=num;
}

/******************************************************************************/
/******************************************************************************/

void CPuck::AddCollision(){
	m_unCollisionCount++;
}

/******************************************************************************/
/******************************************************************************/

unsigned int CPuck::GetCollisions(){
	return m_unCollisionCount;
}

/******************************************************************************/
/******************************************************************************/

// The rotation of the puck is not taken into account
void CPuck::SimulationStep(unsigned int n_simulation_step,
                           double f_time,
                           double f_step_interval)
{
    if(m_pcGrippingEpuck != NULL)
    {
        double fNewXPuck, fNewYPuck;
        double fXEpuck, fYEpuck, fRotEpuck;
        m_pcGrippingEpuck->GetPosition(&fXEpuck, &fYEpuck);
        fRotEpuck = m_pcGrippingEpuck->GetRotation();

        fNewXPuck = fXEpuck + (CEpuck::CHASSIS_RADIUS + CPuck::CHASSIS_RADIUS)*cos(fRotEpuck);
        fNewYPuck = fYEpuck + (CEpuck::CHASSIS_RADIUS + CPuck::CHASSIS_RADIUS)*sin(fRotEpuck);

        SetPosition(fNewXPuck, fNewYPuck);
    }
}

/******************************************************************************/
/******************************************************************************/

void CPuck::SetGrippingEpuck(CEpuck* pc_epuck)
{
    m_pcGrippingEpuck = pc_epuck;
}

/******************************************************************************/
/******************************************************************************/

CEpuck* CPuck::GetGrippingEpuck()
{
    return m_pcGrippingEpuck;
}

/******************************************************************************/
/******************************************************************************/

double CPuck::GetInitialRotation()
{
    return m_fInitialRotation;
}

/******************************************************************************/
/******************************************************************************/

void CPuck::SetInitialRotation(double f_initial_rotation)
{
    m_fInitialRotation = f_initial_rotation;
}

/******************************************************************************/
/******************************************************************************/

void CPuck::SetColor(float f_red, float f_green, float f_blue)
{
    m_fColorRed     = f_red;
    m_fColorGreen   = f_green;
    m_fColorBlue    = f_blue;
}

/******************************************************************************/
/******************************************************************************/

unsigned int CPuck::GetAllColoredLeds(){
    return m_punColoredLeds[0];
}

/******************************************************************************/
/******************************************************************************/

void CPuck::SetAllColoredLeds(unsigned int un_color)
{
    for (int i = 0; i < 8; i++)
        m_punColoredLeds[i] = un_color ;

    if(un_color == LED_COLOR_BLACK) 		  SetColor(0,0,0);
    else if(un_color == LED_COLOR_RED) 	  SetColor(1,0,0);
    else if(un_color == LED_COLOR_GREEN)   SetColor(0,1,0);
    else if(un_color == LED_COLOR_BLUE) 	  SetColor(0,0,1);
    else if(un_color == LED_COLOR_YELLOW ) SetColor(1,1,0);
    else if(un_color == LED_COLOR_WHITE ) SetColor(1,1,1);
}

/******************************************************************************/
/******************************************************************************/

void CPuck::SetColoredLed(unsigned int un_index, unsigned int un_color)
{
    m_punColoredLeds[un_index] = un_color;
}

/******************************************************************************/
/******************************************************************************/

unsigned int CPuck::GetColoredLed(unsigned int un_index)
{
    return m_punColoredLeds[un_index];
}

/******************************************************************************/
/******************************************************************************/

bool CPuck::SomeColoredLedIsOn()
{
    unsigned int unIndex = 0;
    while (unIndex < 8)
    {
        if (m_punColoredLeds[unIndex] != LED_COLOR_BLACK)
            return true;
        else
            unIndex++;
    }
    return false;
}

/******************************************************************************/
/******************************************************************************/
