/******************************************************************************

An instance of the arena class keeps keeps track of the layout of the current 
world. Arenas are two dimensional and each point of the arena space can take
three values, namely normal (where the sbots can move), hole - into which the 
sbots can fall, and finally obstancles into which the sbots can bump.

An arena has a size and a resolution (which can be infinite). The size
corresponds to its physical size within the virtual world, while the resolution
denotes how many points there are in case of a discrete arena. 

This may sound complicated, but really there is nothing to it. Assume that we 
have a virtual world with the resolution 3x3:

                                      1 1 1
                                      0 1 0
                                      1 1 1

The physical size can be 1 meter, 5 m or whatever gets you off. The arena 
contains two holes (0s) and 7 "pixels" where the robots can move. Naturally,
we could have represented that in a different way, namely analytically in which 
case the resolution would have been infinite (for all practical purposes).

But why do we need a resolution? Because we like drawing stuff. And for each
pixel we can draw a small box. It looks good.

An arena can be queried about the state of any point defined by arena. This
is used by the sbots to figure out their sensor readings and possibly by 
some more advanced dymanics engine in the future (for obstacles etc.).

Note: In a discrete arena, the arena "pixels" run from 0..res-1 on both axis,
whereas querying for a point height runs from -x/2 .. x/2 and -y/2 .. -y/2. 

******************************************************************************/

#ifndef ARENA_H_
#define ARENA_H_

/******************************************************************************/
/******************************************************************************/

using namespace std;

#include "render.h"
#include "geometry.h"
#include "circlecollisionobject.h" //for the shelters
#include "objects/groundarea.h"
#include "objects/lightobject.h"
#include "objects/bluelightobject.h"
#include "objects/redlightobject.h"

class CCompoundCollisionObject;
//class CArena;
/******************************************************************************/
/******************************************************************************/

#define ARENA_RESOLUTION_INFINITE 0

#define ARENA_TYPE_ROUND     0
#define ARENA_TYPE_SQUARE     1
#define ARENA_TYPE_BOUNDLESS 2

/******************************************************************************/
/******************************************************************************/

enum eArenaHeight {

	HEIGHT_HOLE,
	HEIGHT_NORMAL,
	HEIGHT_OBSTACLE
};

/******************************************************************************/
/******************************************************************************/

/* struct TSound { */
/*     dVector2      vPosition;     */
/*     double        fDuration; */
/*     unsigned int  unType; */
/*     bool          bJustAdded; */
/* }; */



/******************************************************************************/
/******************************************************************************/

enum EAmbientLightLevel {
	AMBIENT_LIGHT_DARKNESS,
	AMBIENT_LIGHT_ARENA_LIGHT,   
	AMBIENT_LIGHT_FULL_LIGHT
} ;



/******************************************************************************/
/******************************************************************************/

class CArena : public CGeometry
{
	public:

		CArena(const char* pch_name);
		virtual ~CArena();

		virtual void GetSize(double* f_size_x, double* f_size_y);
		virtual void GetResolution(unsigned int* un_res_x, unsigned int* un_res_y);

		virtual eArenaHeight GetHeight(double x, double y) = 0;
		eArenaHeight GetHeight(dVector2 v_position);


		virtual void Draw(CRender* pc_render);

		virtual void SimulationStep(unsigned int n_step_number, double f_time, double f_step_interval);

		virtual double GetGroundColor(dVector2 v_position);
		virtual char* GetGroundAreaName(dVector2 Pos, double f_orientation);

		virtual CCompoundCollisionObject* GetHorizontalCollisionObject(double y, double x1, double x2) = 0;
		virtual CCompoundCollisionObject* GetVerticalCollisionObject(double x, double y1, double y2) = 0;

		virtual int GetArenaType() = 0;

		//SHELTER's STUFF

		virtual void AddShelter(CCircleCollisionObject* pc_Shelter);
		virtual bool IsUnderShelter(float f_X,float f_Y);
		dVector2 GetClosestShelterPosition(float f_X, float f_Y);
		virtual vector<CCircleCollisionObject*> GetShelters();


		//GROUND SENSOR's STUFF
		/* Add a ground round area that will be detected by the ground sensors*/
		virtual void AddGroundArea(CGroundArea* pc_ground_area);
		/* Get the color of the area 
		 * 1.0 is white
		 * 0.0 is black
		 * There is a whole greay scale in between
		 * */
		double *GetGroundAreaColor(dVector2 Pos, double f_orientation);
		/* Get all the ground areas */
		virtual vector<CGroundArea*> GetGroundAreas();



		/* LIGHT STUFF*/
		/* Add a light obhect */
		virtual void AddLightObject(CLightObject* pc_light_object);
		/* Get all the light objects */
		virtual vector<CLightObject*> GetLightObject();
		/* Detect light object */
		virtual bool LightDistance ( dVector2 Pos, double range, double *distance);
		/* Get Nearest Light Object */
		virtual bool GetNearestLight ( dVector2 Pos, double range, dVector2 *light_position, double *nearestDistance);
		virtual bool GetNearestLightInSector ( dVector2 Pos, double f_orientation, double f_aperture, double range, dVector2 *light_position, double *nearestDistance, double *relativeAngle);
		virtual bool GetNearestBlueLightInSector ( dVector2 Pos, double f_orientation, double f_aperture, double range, dVector2 *light_position, double *nearestDistance, double *relativeAngle);
		virtual bool GetNearestRedLightInSector ( dVector2 Pos, double f_orientation, double f_aperture, double range, dVector2 *light_position, double *nearestDistance, double *relativeAngle);
		
		
		/* BLUE LIGHT STUFF */
		/* Add a light obhect */
		virtual void AddBlueLightObject(CBlueLightObject* pc_blue_light_object);
		/* Get all the light objects */
		virtual vector<CBlueLightObject*> GetBlueLightObject();
		/* Detect light object */
		virtual bool BlueLightDistance ( dVector2 Pos, double range, double *distance);
		/* Get Nearest Blue Light Object */
		virtual bool GetNearestBlueLight ( dVector2 Pos, double range, dVector2 *light_position, double *nearestDistance);
		
		/* RED LIGHT STUFF */
		/* Add a light obhect */
		virtual void AddRedLightObject(CRedLightObject* pc_red_light_object);
		/* Get all the light objects */
		virtual vector<CRedLightObject*> GetRedLightObject();
		/* Detect light object */
		virtual bool RedLightDistance ( dVector2 Pos, double range, double *distance);
		/* Get Nearest Red Light Object */
		virtual bool GetNearestRedLight ( dVector2 Pos, double range, dVector2 *light_position, double *nearestDistance);


		/* Switch on/off light functions */
		virtual void SwitchNearestLight(dVector2 Pos, int n_value);
		virtual void SwitchNearestBlueLight(dVector2 Pos, int n_value);
		virtual void SwitchNearestRedLight(dVector2 Pos, int n_value);

	protected:
		virtual void SetSize(double f_size_x, double f_size_y);
		virtual void SetResolution(unsigned int un_res_x, unsigned int un_res_y);

		double       m_fSizeX;
		double       m_fSizeY;

		vector<CCircleCollisionObject*> m_vecShelters;
		/*vector<CLightObject*> m_vecLights;*/
		/*vector<CColoredWall*> m_vecWalls;*/
		vector<CGroundArea*> m_vecGroundArea;
		vector<CLightObject*> m_vecLightObject;
		vector<CBlueLightObject*> m_vecBlueLightObject;
		vector<CRedLightObject*> m_vecRedLightObject;

		unsigned int m_unResX;
		unsigned int m_unResY;

		double (*m_pColorFunction)(double,double);


};


/******************************************************************************/
/******************************************************************************/

#endif
