#ifndef RENDER_H_
#define RENDER_H_

/******************************************************************************/
/******************************************************************************/
/*
class CSbot;
class CPalet;
class CStoy;
class CGeometry;
class CSimulator;
class CLightEmitter;
class CSoundEmitter;
*/

class CGeometry;
class CEpuck;
class CPuck;
class CSimulator;
/*class CLightObject;	*/
/*class CColoredWall;*/
	

class CArena;	
/******************************************************************************/
/******************************************************************************/
using namespace std;

#include "epuck.h"
#include "puck.h"
#include "simulator.h"
/*#include "objects/coloredwall.h"*/
//#include "light.h"
#include "arena.h"


/******************************************************************************/
/******************************************************************************/

class CRender 
{
public:
//    CRender(CSimulator* pc_simulator, CArguments* pc_renderer_arguments = 0);
    CRender(CSimulator* pc_simulator);
    virtual ~CRender();


	virtual void DrawEpuck(CEpuck* pc_epuck, float f_color_red, float f_color_green, float f_color_blue) = 0;
	virtual void DrawPuck(CPuck* pc_puck, float f_color_red, float f_color_green, float f_color_blue) = 0;
//	virtual void DrawColoredWall(CColoredWall* pc_wall)=0;
//	virtual void DrawLight(CLightObject* pc_light)=0;
//    virtual void DrawSbot(CSbot* pc_sbot, float f_color_red, float f_color_green, float f_color_blue) = 0;
//    virtual void DrawStoy(CStoy* pc_stoy, float f_color_red, float f_color_green, float f_color_blue) = 0;
//    virtual void DrawPalet(CPalet* pc_palet, float f_color_red, float f_color_green, float f_color_blue) = 0;
    virtual void DrawArena(CArena* pc_arena) = 0;
//    virtual void DrawSound(CSoundEmitter* pc_arena) = 0;
//    virtual void DrawIntensitySound(CSoundEmitter* pc_arena, double f_intensity) = 0;
    virtual void DrawIndicator(float f_x, float f_y, float f_color_red, float f_color_green, float f_color_blue);
    virtual void Draw2DLine(dVector2 v_orig, dVector2 v_dest, float height, float color_red = 1.0, float color_green = 0.0, float color_blue = 0.0) = 0;

    virtual void Start() = 0; 
    
    virtual void SetSimulator(CSimulator* pc_simulator);



    // Frame rate = 1, a frame is drawn for each simulation step
    // 2 = a frame is drawn for each other simulation step.
    // All values >= 1 are valid (e.g. 1.5 is ok).
    virtual void SetFrameRate(double f_frame_rate);

    static CRender* GetInstance();
    
    virtual void dumpCurrentFrame();
    void SetProduceFrames(bool b);

            
protected:
    static CRender* m_pcRender;

    CSimulator* m_pcSimulator;

    double m_fFrameRate;
    
    int m_iFrame;
    bool m_bProduceFrames;
    
//    CArguments* m_pcRendererArguments;

};

/******************************************************************************/
/******************************************************************************/

#endif
