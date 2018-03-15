#ifndef SIMPLEDRAWSTUFFRENDER_H_
#define SIMPLEDRAWSTUFFRENDER_H_

/******************************************************************************/
/******************************************************************************/

#include "render.h"
//#include "collisionmanager.h"
#include "rectanglecollisionobject.h"

//Sensors
#include "lightsensor.h"
#include "bluelightsensor.h"
#include "redlightsensor.h"
#include "reallightsensor.h"
#include "realbluelightsensor.h"
#include "realredlightsensor.h"
#include "groundsensor.h"
#include "contactsensor.h"
#include "epuckproximitysensor.h"

#include "drawstuff/drawstuff.h"
#include <GL/gl.h>
#include <GL/glu.h>
/******************************************************************************/
/******************************************************************************/

typedef double dsRotationMatrix[12];

// Create a drawstuff rotation matrix corresponding to a given angle:
/*#define dRotation2dsRotationMatrix(angle, matrix)                                                               \*/
/*{                                                                                                             \*/
/*matrix[0 * 4 + 2] = matrix[1 * 4 + 2] = matrix[2 * 4 + 0] = matrix[2 * 4 + 1] = 0;                         \*/
/*matrix[0 * 4 + 3] = matrix[1 * 4 + 3] = matrix[2 * 4 + 3] = 0;                                             \*/
/*matrix[0 * 4 + 0] = matrix[1 * 4 + 1] = cos(angle);                                                        \*/
/*matrix[0 * 4 + 1] = -sin(angle);                                                                           \*/
/*matrix[1 * 4 + 0] = sin(angle);                                                                            \*/
/*matrix[2 * 4 + 2] = 1;					\*/
/*}*/


/*#define dTwoAngleRotation2dsRotationMatrix(z, y, matrix)                                                                                           \*/
/*{                                                                                                                                                \*/
/*matrix[0 * 4 + 0] = cos(y)*cos(z);           matrix[0 * 4 + 1] = -sin(z)        ;  matrix[0 * 4 + 2] = sin(y)*cos(z);         matrix[0 * 4 + 3] = 0;   \*/
/*matrix[1 * 4 + 0] = cos(y)*sin(z);           matrix[1 * 4 + 1] = cos(z)         ;  matrix[1 * 4 + 2] = sin(y)*sin(z);         matrix[1 * 4 + 3] = 0;   \*/
/*matrix[2 * 4 + 0] = -sin(y);                 matrix[2 * 4 + 1] = 0              ;  matrix[2 * 4 + 2] = cos(y);                matrix[2 * 4 + 3] = 0;   \*/
/*}*/


/******************************************************************************/
/******************************************************************************/

class CSimpleDrawStuffRender : public CRender 
{
public:
    CSimpleDrawStuffRender(CSimulator* pc_simulator, int argc, char** argv);
	
		virtual void dRotation2dsRotationMatrix ( double angle, double* matrix);
		virtual void dTwoAngleRotation2dsRotationMatrix(double z , double y, double* matrix );
    virtual void DrawEpuck(CEpuck* pc_epuck, float f_color_red, float f_color_green, float f_color_blue);
    virtual void DrawGripper(CEpuck* pc_epuck, float f_color_red, float f_color_green, float f_color_blue);
    virtual void DrawPuck(CPuck* pc_puck, float f_color_red, float f_color_green, float f_color_blue);
    virtual void DrawArena(CArena* pc_arena);
    virtual void DrawIndicator(float f_x, float f_y, float f_color_red, float f_color_green, float f_color_blue);
	virtual void Draw2DLine(dVector2 v_orig, dVector2 v_dest, float height, float color_red = 1.0, float color_green = 0.0, float color_blue = 0.0);
//	virtual void DrawColoredWall(CColoredWall* pc_wall);
//	virtual void DrawLight(CLightObject* pc_light);
    virtual void Start(); 
	virtual void SetFrameRate(float fFrameRate);
	
	void dumpCurrentFrame();
				
protected:
    static void DSCommandCallback(int key_code);
    static void DSSimLoopCallback(int pause);
    static void DSStartCallback();

    virtual void CommandCallback(int key_code);
    virtual void SimLoopCallback(int pause);
    virtual void StartCallback();

protected:
    unsigned int m_unWindowSizeX;
    unsigned int m_unWindowSizeY;

    int          m_argc;
    char**       m_argv;

    double       m_fSimulationFrames;
    double       m_fIndicatorSinusAngle;
 
    bool         m_bEnableFollowSbot;
    bool         m_bDisplayPrompt;
		bool		 	 	 m_bDrawIRSensors;	
		bool         m_bDrawLightSensors;
		bool         m_bDrawBlueLightSensors;
		bool         m_bDrawRedLightSensors;
		bool         m_bDrawContactSensors;
		bool		     m_bDrawCamera;	
		bool		     m_bDrawSound;	

    bool         m_bUseTextures;
    bool         m_bUseCustomTextureForGround;
    bool         m_bPauseAtStart;
     
};

/******************************************************************************/
/******************************************************************************/
//    virtual void DrawSbot(CSbot* pc_sbot, float f_color_red, float f_color_green, float f_color_blue);
//    virtual void DrawStoy(CStoy* pc_stoy, float f_color_red, float f_color_green, float f_color_blue);
//    virtual void DrawPalet(CPalet* pc_palet, float f_color_red, float f_color_green, float f_color_blue);    
//    virtual void DrawLight(CLightEmitter* pc_light);
//    virtual void DrawSound(CSoundEmitter* pc_sound);
//    virtual void DrawIntensitySound(CSoundEmitter* pc_sound, double f_intensity);   

#endif
