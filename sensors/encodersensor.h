#ifndef ENCODERSENSOR_H_
#define ENCODERSENSOR_H_

/******************************************************************************/
/******************************************************************************/


/* This is the class of the encoder sensor */
/* The sensor undo the kinematic equiations giving liner displacement of each wheel */
class CEncoderSensor;

/******************************************************************************/
/******************************************************************************/

#include "sensor.h"
#include "arena.h"
#include "random.h"

#define NUM_ENCODER_SENSORS 2

/******************************************************************************/
/******************************************************************************/

class CEncoderSensor : public CSensor
{
public:
    CEncoderSensor(const char* pch_name,CArena* pc_Arena, float encoder_error, float initX, float initY);
    ~CEncoderSensor();

    // Get the sensor type:
    virtual unsigned int GetType();

		//Compute Readings
		double* ComputeSensorReadings(CEpuck* p_pcEpuck, CSimulator* p_pcSimulator);
	 	
		//Get Reading. Returns linear movement of each wheel (left, right).
    /*void getSensorReading( CEpuck *p_pcEpuck, float *f_enc_left, float *f_enc_right );*/
		double* GetSensorReading (CEpuck *p_pcEpuck);

		//Init Sensor
		void InitEncoderSensor (CEpuck *p_pcEpuck);
    
protected:
    CArena* m_pcArena;
		/* Initial variables */
		dVector2 m_vInitialPosition;
		float m_fInitialOrientation;
		
		/* Last known values */
		dVector2 m_vLastPosition;
		float m_fLastOrientation;

		float m_fEncoderError;
};

/******************************************************************************/
/******************************************************************************/

#endif
