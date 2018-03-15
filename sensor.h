/******************************************************************************

This is the super-class for all sensors. It is quite simple. It simply contains
two public accessible methods, one for getting the number of output that a 
sensor produces (e.g. for an artificial neural network), and another for 
getting the actual outputs.

What goes inside a specialization of this class is all up to you. But 
notice that each robot with a given sensor has its own, private instance 
of the sensor.

*******************************************************************************/

#ifndef SENSOR_H_
#define SENSOR_H_


/******************************************************************************/
/******************************************************************************/

//#include "common.h"

// Returns the normalized angle in the range [0,2*M_PI)
/*#define NormalizeAngle(ang) \*/
/*(ang < 0 ? fmod(ang, 2*M_PI)+2*M_PI : fmod(ang, 2*M_PI))*/

#include <math.h>
#include <vector>
#include <list>
#include "general.h"
using namespace std;

/******************************************************************************/
/******************************************************************************/

class CSensor;

typedef vector<CSensor*>            TSensorVector;
typedef vector<CSensor*>::iterator  TSensorIterator;

/******************************************************************************/
/******************************************************************************/

#define SENSOR_CONTACT								1
#define SENSOR_PROXIMITY             	2
#define SENSOR_LIGHT									3
#define SENSOR_BATTERY								4
#define SENSOR_GROUND_MEMORY					5
#define SENSOR_GROUND									6
#define SENSOR_BLUE_LIGHT							7
#define SENSOR_RED_LIGHT							8
#define SENSOR_BLUE_BATTERY						9
#define SENSOR_RED_BATTERY						10
#define SENSOR_COM										11
#define SENSOR_REAL_LIGHT							12
#define SENSOR_REAL_BLUE_LIGHT				13
#define SENSOR_REAL_RED_LIGHT	  			14

#define SENSOR_COMPASS								0x0301
#define SENSOR_ENCODER								0x0302
#define SENSOR_RANDB									0x0303
#define SENSOR_PROX										0x0306
#define SENSOR_PROXIMITY_POLYNOMIAL   0x1000//Proximity sensor with polynomial model

//#define SENSOR_TRACTION                     0x0001
/*#define SENSOR_SIMPLE_MODULAR_CAMERA		0x0001*/
//#define SENSOR_GROUND                       0x0002
/*#define SENSOR_MODULAR_CAMERA               0x0002*/
/*#define SENSOR_GROUNDPLAYSOUND              0x0004*/
/*#define SENSOR_LIGHT                        0x0008*/
//#define SENSOR_SAMPLEDLIGHT                 0x0022
/*#define SENSOR_AMBIENTLIGHT                 0x0022*/
//#define SENSOR_SIMPLESOUND                  0x000A
/*#define SENSOR_SOUND_SIMPLE					0x000A*/
//#define SENSOR_INTENSITYSOUND               0x001C
//#define SENSOR_LIGHT			            0x001C
/*#define SENSOR_SAMPLED_IR                   0x0028*/
/*#define SENSOR_CAMERA                       0x000E*/
/*#define SENSOR_LIGHT_SAMPLED                0x0010*/
/*//#define SENSOR_NN_LR_CAMERA                 0x0010*/
/*#define SENSOR_PIE_CAMERA                   0x0012*/
/*#define SENSOR_SIMPLE_COLORED_OBJECT_CAMERA 0x0014*/
/*#define SENSOR_OBJECT_GRIPPED               0x0016*/
/*//#define SENSOR_GRIPPER_APERTURE             0x0018*/
/*#define SENSOR_PROXIMITY_LOOKUP             0x0018*/
/*#define SENSOR_HEADING_CAMERA               0x0020*/
/*//#define SENSOR_OPTICAL_BARRIER              0x001A*/
/*#define SENSOR_CLASS              			0x001A //The simple camera sensor*/
/*#define SENSOR_UDP_MESSAGE                  0x0024*/

/*#define SENSOR_WHEELS                       0x0100*/
/*//#define SENSOR_TURRETROTATION               0x0200*/
/*#define SENSOR_LIGHT_THRESHOLD               0x0200*/

/*#define SENSOR_CHAR_TRACTION                      'T'*/
/*#define SENSOR_CHAR_GROUND                        'G'*/
/*#define SENSOR_CHAR_GROUNDPLAYSOUND               'P'*/
/*#define SENSOR_CHAR_LIGHT                         'L'*/
/*#define SENSOR_CHAR_SAMPLEDLIGHT                  'M'*/
/*#define SENSOR_CHAR_TURRETROTATION                'R'*/
/*#define SENSOR_CHAR_SIMPLESOUND                   'S'*/
/*#define SENSOR_CHAR_INTENSITYSOUND                'X'*/
/*#define SENSOR_CHAR_CAMERA                        'C'*/
#define SENSOR_CHAR_PROXIMITY                     'I'
/*#define SENSOR_CHAR_SAMPLED_IR                    'Y'*/
/*#define SENSOR_CHAR_NN_LR_CAMERA                  'Z'  // Possibly followed by the number of objects to track*/
/*#define SENSOR_CHAR_PIE_CAMERA                    'E'*/
/*#define SENSOR_CHAR_OBJECT_GRIPPED                'O'*/
/*#define SENSOR_CHAR_GRIPPER_APERTURE              'A'*/
/*#define SENSOR_CHAR_SIMPLE_COLORED_OBJECT_CAMERA  'D'*/
/*#define SENSOR_CHAR_OPTICAL_BARRIER               'B'*/
#define SENSOR_CHAR_UDP_MESSAGE                   'U'


/******************************************************************************/
/******************************************************************************/

#include "simobject.h"
//#include "sbot.h"
//#include "swarmbot.h"
#include "epuck.h"
#include "simulator.h"

/******************************************************************************/
/******************************************************************************/

class CSensor : public CSimObject
{
public: 
    CSensor(const char* pch_name, unsigned int un_no_inputs);
    virtual ~CSensor();
    // This function should return the number of outputs
    virtual unsigned int GetNumberOfInputs();
    
    virtual double* ComputeSensorReadings(CEpuck* pc_epuck,
                                         CSimulator* pc_simulator) = 0; 
    // Get the latest computed sensor readings:
    virtual double* GetComputedSensorReadings();

    // Get the sensor type:
    virtual unsigned int GetType() = 0;


    // For debug purposes only - returns a NULL terminated string
    // containing the sensor readings in human readable form. 
    //
    // !NOTICE!: NOT THREAD-SAFE USES A STATIC VARIABLE!
    static char*  SensorReadingsToString(unsigned int un_number_of_readings, 
                                         double* pf_readings);

/*
    // Returns an array of sensor readings for the given sbot, member
    // of the given swarmbot and located in the given simulator. This
    // is called for each control step:
    //virtual double* ComputeSensorReadings(CSbot* pc_sbot, 
    //                                     CSwarmBot* pc_swarmbot, 
    //                                     CSimulator* pc_simulator) = 0; 

*/


protected:
    virtual void   SetInput(unsigned int un_index, double f_value);
    virtual double* GetInputs();
    virtual double GetInput(unsigned int un_index);
		/*virtual float NormalizeAngle ( float angle );*/

protected: 
    unsigned int m_unNumberOfInputs;    
    double*      m_pfInputs;

    static char  m_pchSensorReadingsString[10240];

};

/******************************************************************************/
/******************************************************************************/

#endif
