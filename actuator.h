#ifndef ACTUATOR_H_
#define ACTUATOR_H_

/******************************************************************************/
/******************************************************************************/


#include <math.h>
#include <vector>
#include <list>

using namespace std;

class CActuator;

typedef vector<CActuator*>            TActuatorVector;
typedef vector<CActuator*>::iterator  TActuatorIterator;

/******************************************************************************/
/******************************************************************************/

#include "simobject.h"
#include "epuck.h"

/******************************************************************************/
/******************************************************************************/

#define ACTUATOR_WHEELS             0x0001
#define ACTUATOR_WHEELS_TEST        0x0004
#define ACTUATOR_GRIPPER_ELEVATION  0x0008
#define ACTUATOR_CONFIDENCE         0x1000
#define ACTUATOR_SIMPLESOUND        0x0010
#define ACTUATOR_INTENSITYSOUND     0x0011
#define ACTUATOR_UDP_MESSAGE        0x001A
#define ACTUATOR_TEMPORARY_MESSAGE	    0x0111

#define ACTUATOR_TX2				0x0020

#define ACTUATOR_PROJECTION							0x3000

#define ACTUATOR_CHAR_WHEEL               'W'
#define ACTUATOR_CHAR_GRIPPER_APERTURE    'G'
#define ACTUATOR_CHAR_GRIPPER_ELEVATION   'E'
#define ACTUATOR_CHAR_CONFIDENCE          'C'
#define ACTUATOR_CHAR_SIMPLESOUND         'S'
#define ACTUATOR_CHAR_INTENSITYSOUND      'I'
#define ACTUATOR_CHAR_UDP_MESSAGE         'U'
#define ACTUATOR_CHAR_TEMPORARY_MESSAGE		  'O'

#define ACTUATOR_CHAR_TX2					'T'

/******************************************************************************/
/******************************************************************************/

class CActuator : public CSimObject
{
public:
    CActuator(const char* pch_name, CEpuck* pc_epuck, unsigned int un_number_of_outputs);

    virtual unsigned int GetNumberOfOutputs();
    virtual void SetOutput(unsigned int un_output_index, double f_value) = 0;
    
    virtual unsigned int GetType() = 0;

protected:
    virtual CEpuck* GetEpuck();
protected:

    unsigned int  m_unNumberOfOutputs;
    CEpuck*        m_pcEpuck;     
};

/******************************************************************************/
/******************************************************************************/

#endif
