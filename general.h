#ifndef _GENERAL_
#define _GENERAL_

#include <ode/ode.h>
#include <math.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <stdio.h>

using namespace std;

//#define dDOUBLE

#define CATI    2 // 2^1  - 001 robot body and motorised wheel
#define CATII   4 // 2^2  - 010 ground
#define CATIII  8 // 2^3  - 100 no friction object

#define COLI    CATI + CATII + CATIII // a robot or an object
#define COLII   CATI                  //ground
#define COLIII  CATI + CATIII         //no friction geometries

#define TIME_STEP		  0.1
#define WORLD_GRAVITY    -9.81

//#define _GRAPHICS_
//#define _STATS_

#ifdef _GRAPHICS_
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>

//#include <gl.h>
//#include <glu.h>
//include <glut.h>
#include <drawstuff/drawstuff.h>
#endif


/* ------------------------------------------------------------------------------------------*/
/*                     Distance functions - Geometries                                       */
/* ------------------------------------------------------------------------------------------*/
/* Return the distance between two point of which we know the x and y coordinates.           */
inline dReal dist(dReal x1, dReal y1, dReal x2, dReal y2) {
  return sqrt( (x2-x1) * (x2-x1) + (y2-y1) * (y2-y1) );
}
/* ------------------------------------------------------------------------------------------*/
/* ------------------------------------------------------------------------------------------*/

double        getDouble(char sep, istream& I);
int           getInt(char sep, istream& I);
unsigned long getLongInt(char sep, istream& I);
void          getStr(char sep, istream& I, char* s);
char          getChar(char sep, istream& I);
bool          getYesNo(char sep, istream& I);
int           string2Int(char* s);
bool          isIntString(char* s);
bool          fileExists(char* F);
void          fatal     (bool condition, char* location, char* message);
#endif
