#ifndef PROXIMITYSENSORPOLYNOMIAL_H_
#define PROXIMITYSENSORPOLYNOMIAL_H_

/******************************************************************************/
/******************************************************************************/

#define NUM_PROXIMITY_SENSORS	8
//#define MAXRANGE 0.205
#define MAXRANGE 0.075
//#define MAXRANGE 0.1
//#define MAXRANGE 0.035
#define MINRANGE 0.0
#define DISTANCE_UNIT 100


class CProximitySensorPolynomial;

/******************************************************************************/
/******************************************************************************/

#include "sensor.h"
#include "arena.h"
#include "random.h"

/******************************************************************************/
/******************************************************************************/

class CProximitySensorPolynomial : public CSensor
{
public:
    CProximitySensorPolynomial(const char* pch_name);
    ~CProximitySensorPolynomial();

    virtual double* ComputeSensorReadings(CEpuck* pc_epuck, CSimulator* pc_simulator); 
    // Get the sensor type:
    virtual unsigned int GetType();
	//Reading values
    const double* GetIRReadings();
	//Heading of sensors with respect to the robot
	const double* GetIRSensorDirections();
	//Returns the maximum value for the sensor reading
    static double GetProximityMaxValue( void );

	void NormalizeReadings();
	const float GetMaxRange(){return MAXRANGE;}
	
protected:
    double m_fIRReadings[NUM_PROXIMITY_SENSORS];
	float vf_MinDistances[NUM_PROXIMITY_SENSORS]; //Min Distances "recorded" by each sensor
    static double m_fIRSensorDir[NUM_PROXIMITY_SENSORS];
	void ComputeEpuckReading(CEpuck*,CEpuck*);
	void ComputeStraightWallReading(CEpuck*,float,int);
	float ReadingGivenDistance(float);

    static const double MAX_MEASURE=3377;
	//static const double MAX_MEASURE=3493;
    //static const double MIN_MEASURE=503;
	static const double MIN_MEASURE=1.0;


private:
	bool CircleLineIntersections(float,float,float,float,float,float,float,float*,float*,float*,float*);
};

/******************************************************************************/
/******************************************************************************/

#endif
