#ifndef EPUCKPROXIMITYSENSOR_H_
#define EPUCKPROXIMITYSENSOR_H_

/******************************************************************************/
/******************************************************************************/

#define PROXIMITY_MIN_DISTANCE	0.00
#define PROXIMITY_MAX_DISTANCE	0.28

#define APERTURE_ANGLE		0.17
#define NUM_PROXIMITY_SENSORS	8

#define EPUCK_2_EPUCK_PROXIMITY	1
//#define EPUCK_2_STOY_PROXIMITY	2
#define EPUCK_2_WALL_PROXIMITY	3

//#include "common.h"

class CEpuckProximitySensor;

/******************************************************************************/
/******************************************************************************/

#include "sensor.h"
#include "arena.h"

/******************************************************************************/
/******************************************************************************/

class CEpuckProximitySensor : public CSensor
{
public:
    CEpuckProximitySensor(int n_robotNumber);
    ~CEpuckProximitySensor();

    virtual double* ComputeSensorReadings(CEpuck* pc_epuck, 
                                          CSimulator* pc_simulator); 
    
    // Get the sensor type:
    virtual unsigned int GetType();
    const double* GetIRReadings();
    void PrintIRReadings();
    void ApplyNoise(double& f_value);

    void LoadIRSamplesFromFile( const char* pc_filename , int n_type );
    void LoadIRSamplesFromFile( int n_robotnumber, int n_type );

    void AddObjectToIRReadings( double f_distance , double f_direction , int n_type );
    unsigned int GetIndexFast(double f_distance, double f_direction, int n_type);
    unsigned int GetIndexWithManhattanDistance(double f_distance, double f_direction, int n_type);
    const double* GetSensorDirections();

    void NormalizeReadings();

    void GetMaxReading( unsigned int* un_sensor_index , double* f_sensor_value );
		static double GetMaxRange ( void );

    static void   SetProximityMaxValue( double f_max_value );
    static double GetProximityMaxValue( void );
    static void   SetNoiseMaxValue( double f_noise );
    static double GetNoiseMaxValue( void );
    void          SetNoiseStatus( int n_noise_status );
    int           GetNoiseStatus( void );
		double* GetSensorReading (CEpuck *p_pcEpuck); 

		static unsigned int SENSOR_NUMBER;
protected:

		double m_fIRReadings[NUM_PROXIMITY_SENSORS];
    static double PROXIMITY_RANGE[];
    static double PROXIMITY_MAX_VALUE;

    static unsigned int** m_ppunReadings_epuck2epuck;
    static double**       m_ppunSituationIndex_epuck2epuck;
    static unsigned int   m_unNumSensors_epuck2epuck;
    static unsigned int   m_unNumDistances_epuck2epuck;
    static unsigned int   m_unNumDirections_epuck2epuck;
    static unsigned int   m_unNumReadings_epuck2epuck;
    static double         m_fRange_epuck2epuck;
    //static double         m_fRange_epuck2stoy;//readings for stoy are identical to the ones of epuck

    static unsigned int** m_ppunReadings_epuck2wall;
    static double**       m_ppunSituationIndex_epuck2wall;
    static unsigned int   m_unNumSensors_epuck2wall;
    static unsigned int   m_unNumDistances_epuck2wall;
    static unsigned int   m_unNumDirections_epuck2wall;
    static unsigned int   m_unNumReadings_epuck2wall;
    static double         m_fRange_epuck2wall;

/*     static double	  m_fArenaSizeX; */
/*     static double	  m_fArenaSizeY; */
/*     static double 	  m_vCellSize; */
/*     static unsigned int	  m_unArenaNumCellsX; */
/*     static unsigned int	  m_unArenaNumCellsY; */
    
    static double      	  m_fIRSensorDir[NUM_PROXIMITY_SENSORS];
    
    static double         NOISE_MAX_VALUE;
    int                   m_nNoiseStatus;
};

/******************************************************************************/
/******************************************************************************/

#endif 
