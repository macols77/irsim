
/******************************************************************************

The NullRender simply renders nothing, but drives the simulation. Useful for
doing evolutions.

*******************************************************************************/


#ifndef NULLRENDER_H_
#define NULLRENDER_H_


/******************************************************************************/
/******************************************************************************/

#include "render.h"

/******************************************************************************/
/******************************************************************************/

class CNullRender : public CRender
{
public:
    CNullRender(CSimulator* pc_simulator);
    
    virtual void DrawEpuck(CEpuck* pc_epuck, float f_color_red, float f_color_green, float f_color_blue);
    virtual void DrawPuck(CPuck* pc_puck, float f_color_red, float f_color_green, float f_color_blue);
//	virtual void DrawColoredWall(CColoredWall* pc_wall);
//	virtual void DrawLight(CLightObject* pc_light);
    virtual void DrawArena(CArena* pc_arena);
		virtual void Draw2DLine(dVector2 v_orig, dVector2 v_dest, float height, float color_red = 1.0, float color_green = 0.0, float color_blue = 0.0);
    virtual void Start();
};

/******************************************************************************/
/******************************************************************************/

    //virtual void DrawSbot(CSbot* pc_sbot, float f_color_red, float f_color_green, float f_color_blue);
   // virtual void DrawStoy(CStoy* pc_stoy, float f_color_red, float f_color_green, float f_color_blue);
   // virtual void DrawPalet(CPalet* pc_palet, float f_color_red, float f_color_green, float f_color_blue);
   // virtual void DrawLight(CLightEmitter* pc_light);
   // virtual void DrawSound(CSoundEmitter* pc_sound);
   // virtual void DrawIntensitySound(CSoundEmitter* pc_sound, double f_intensity);


#endif
