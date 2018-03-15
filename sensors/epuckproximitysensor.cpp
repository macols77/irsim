#include "epuckproximitysensor.h"
#include <iostream>
#include <fstream>

#include "random.h"

/******************************************************************************/
/******************************************************************************/

//#include "samples/ir_proximity_epuck2epuck_249.h"
//#include "samples/ir_proximity_epuck2wall_249.h"
//#include "samples/ir_proximity_epuck2epuck_252.h"
//#include "samples/ir_proximity_epuck2wall_252.h"

/******************************************************************************/
/******************************************************************************/

// proximity sensing range wrt object: sbot , wall;
// the **pointers are the tables containing the readings for the following objects
//   from epuck to wall: 29 angles
// 			34 distances (0 cm - 23 cm by 1 cm steps + 25 cm + 30 cm - 110 cm by 10 cm steps)
//   from epuck to epuck: 29 angles
// 			34 distances (0 cm - 23 cm by 1 cm steps + 25 cm + 30 cm - 110 cm by 10 cm steps)
// 8 proximity reading for each configuration
// the values are appr. in the range 0-4000 (most important range 500-2500),
// 	  (and are the medians over 100 samples)
//
// ANGLES:
//  0 ->  17
//  1 ->  50
//  2 ->  90
//  3 -> 150
//  4 -> 210
//  5 -> 270
//  6 -> 310
//  7 -> 343

double CEpuckProximitySensor::PROXIMITY_MAX_VALUE = 4095;
double CEpuckProximitySensor::NOISE_MAX_VALUE    	= 20;
unsigned int CEpuckProximitySensor::SENSOR_NUMBER = NUM_PROXIMITY_SENSORS;

unsigned int** CEpuckProximitySensor::m_ppunReadings_epuck2epuck 		= NULL;
double**       CEpuckProximitySensor::m_ppunSituationIndex_epuck2epuck  = NULL;
unsigned int   CEpuckProximitySensor::m_unNumSensors_epuck2epuck 		= 0;
unsigned int   CEpuckProximitySensor::m_unNumDistances_epuck2epuck 		= 0;
unsigned int   CEpuckProximitySensor::m_unNumDirections_epuck2epuck 		= 0;
unsigned int   CEpuckProximitySensor::m_unNumReadings_epuck2epuck 		= 0;
double         CEpuckProximitySensor::m_fRange_epuck2epuck 			= 0;
//double         CEpuckProximitySensor::m_fRange_epuck2stoy 			= 0;

unsigned int** CEpuckProximitySensor::m_ppunReadings_epuck2wall 		= NULL;
double**       CEpuckProximitySensor::m_ppunSituationIndex_epuck2wall   = NULL;
unsigned int   CEpuckProximitySensor::m_unNumSensors_epuck2wall 		= 0;
unsigned int   CEpuckProximitySensor::m_unNumDistances_epuck2wall 		= 0;
unsigned int   CEpuckProximitySensor::m_unNumDirections_epuck2wall 		= 0;
unsigned int   CEpuckProximitySensor::m_unNumReadings_epuck2wall 		= 0;
double         CEpuckProximitySensor::m_fRange_epuck2wall 			= 0;

// double         CEpuckProximitySensor::m_fArenaSizeX				= 0;
// double         CEpuckProximitySensor::m_fArenaSizeY				= 0;
// double	       CEpuckProximitySensor::m_vCellSize				= 0;
// unsigned int   CEpuckProximitySensor::m_unArenaNumCellsX			= 0;
// unsigned int   CEpuckProximitySensor::m_unArenaNumCellsY			= 0;

// unsigned int   CEpuckProximitySensor::unIRSensorDir[NUM_PROXIMITY_SENSORS] 	= {17,50,90,150,210,270,310,343};

double   CEpuckProximitySensor::m_fIRSensorDir[NUM_PROXIMITY_SENSORS] 	= {0.2967, 0.8727, 1.5708, 2.6180, 3.6652, 4.7124, 5.4105, 5.9865};
 
/******************************************************************************/
/******************************************************************************/

CEpuckProximitySensor::CEpuckProximitySensor(int n_robotNumber) :
    CSensor("proxSensor", NUM_PROXIMITY_SENSORS )
{
    //if( m_ppunReadings_epuck2epuck == NULL ) 
    //{
        //LoadIRSamplesFromFile( n_robotNumber , EPUCK_2_EPUCK_PROXIMITY );
    //}
    
    //if( m_ppunReadings_epuck2wall == NULL ) 
    //{
        //LoadIRSamplesFromFile( n_robotNumber , EPUCK_2_WALL_PROXIMITY );
    //}
	m_nNoiseStatus = false;
}

/******************************************************************************/
/******************************************************************************/

CEpuckProximitySensor::~CEpuckProximitySensor()
{
	//delete [] m_ppunReadings_epuck2epuck;
	//delete [] m_ppunReadings_epuck2wall;
	//delete [] m_ppunSituationIndex_epuck2epuck;
	//delete [] m_ppunSituationIndex_epuck2wall;
}

/******************************************************************************/
/******************************************************************************/

void CEpuckProximitySensor::SetProximityMaxValue( double f_max_value ) 
{
    CEpuckProximitySensor::PROXIMITY_MAX_VALUE = f_max_value;
}

/******************************************************************************/
/******************************************************************************/

double CEpuckProximitySensor::GetProximityMaxValue( void )
{
    return CEpuckProximitySensor::PROXIMITY_MAX_VALUE;
}

/******************************************************************************/
/******************************************************************************/

// this function computes the content of m_unIRReadings,
// an array of NUM_PROXIMITY_SENSORS (8) unsigned int values
double* CEpuckProximitySensor::ComputeSensorReadings(CEpuck* pc_epuck, CSimulator* pc_simulator)
{
	// Initialization to 0
	for (int j = 0; j < NUM_PROXIMITY_SENSORS ; j++)
	{// Initialization to 0
		SetInput(j,PROXIMITY_MAX_DISTANCE);
	}

	double fPosX=0.0, fPosY=0.0;
	pc_epuck->GetPosition(&fPosX, &fPosY);

	// first check proximity to epucks
	TEpuckVector* epucks = pc_simulator->GetEpucks();
	TEpuckIterator i;
	for (i = epucks->begin(); i != epucks->end(); i++)
	{
		// avoid sensing oneself
		if( (*i) != pc_epuck )
		{
			double xCurrentEpuck, yCurrentEpuck, xTargetEpuck, yTargetEpuck;
			pc_epuck->GetPosition(&xCurrentEpuck, &yCurrentEpuck);
			(*i)->GetPosition(&xTargetEpuck, &yTargetEpuck);
			double fDistance;
			double dx = xTargetEpuck - xCurrentEpuck;
			double dy = yTargetEpuck - yCurrentEpuck;
			fDistance = sqrt(dx*dx + dy*dy);

			//subtract robot radius
			double fTurretRadius = CEpuck::CHASSIS_RADIUS;
			fDistance -= fTurretRadius;
			//if( fDistance < m_fRange_epuck2epuck )
			//{
				//double fAngleToTarget;
				//fAngleToTarget = atan2(dy,dx);

				//double fDirection = NormalizeAngle (fAngleToTarget - pc_epuck->GetRotation());
				//AddObjectToIRReadings(fDistance,fDirection,EPUCK_2_EPUCK_PROXIMITY);
			//}

			if (fDistance < PROXIMITY_MAX_DISTANCE )
			{
				double fAngleToTarget = atan2(dy,dx);
				double fDirection = NormalizeAngle (fAngleToTarget - pc_epuck->GetRotation());
			
				/* Calc angles to the robot's body */
				double fTanToRobot = NormalizeAngle (atan2(CEpuck::CHASSIS_RADIUS, fDistance));
				//printf("fDirection: %2f - fTanToRobot: %2f\n\n", fDirection,fTanToRobot);

				for ( int sensorIndex = 0 ; sensorIndex < NUM_PROXIMITY_SENSORS ; sensorIndex++)
				{
					/* If within a 10º of aperture angle */
					//if ( (fDirection > ( m_fIRSensorDir[sensorIndex] - 0.17 )) && (fDirection < ( m_fIRSensorDir[sensorIndex] + 0.17 )))
					
					/* If no problem with 0º */
					if (fDirection > fTanToRobot ){
						if ( ( ( fDirection + fTanToRobot ) > ( m_fIRSensorDir[sensorIndex] - APERTURE_ANGLE ) && ( fDirection - fTanToRobot ) < ( m_fIRSensorDir[sensorIndex] - APERTURE_ANGLE ) ) ||
							 ( ( ( fDirection + fTanToRobot ) > ( m_fIRSensorDir[sensorIndex] + APERTURE_ANGLE ) && ( fDirection - fTanToRobot ) < ( m_fIRSensorDir[sensorIndex] + APERTURE_ANGLE ) ) ) )
						{
							SetInput(sensorIndex,fDistance);	
						}
					}
					/* If problems on 0º */
					else
					{
						if ( ( ( fDirection + fTanToRobot ) > ( m_fIRSensorDir[sensorIndex] - APERTURE_ANGLE ) && ( fDirection - fTanToRobot + 2*M_PI) < ( m_fIRSensorDir[sensorIndex] - APERTURE_ANGLE + 2*M_PI) ) ||
							 ( ( ( fDirection + fTanToRobot + 2*M_PI) > ( m_fIRSensorDir[sensorIndex] + APERTURE_ANGLE ) && ( fDirection - fTanToRobot + 2*M_PI) < ( m_fIRSensorDir[sensorIndex] + APERTURE_ANGLE ) ) ) )
						{
							SetInput(sensorIndex,fDistance);	
						}
					}
				}
			}
		}
	}

	// then check for wall proximity
	CArena* pcArena = pc_simulator->GetArena();

	/* NEW */
	if( pcArena->GetArenaType() == ARENA_TYPE_SQUARE )
	{
		double fSizeX, fSizeY;
		unsigned int unNumCellsX, unNumCellsY;
		pcArena->GetSize(&fSizeX,&fSizeY);
		pcArena->GetResolution(&unNumCellsX,&unNumCellsY);
		double fCellSizeX = fSizeX/(double)unNumCellsX;
		double fCellSizeY = fSizeY/(double)unNumCellsY;
		if( fabs(fCellSizeX - fCellSizeY)>1.23456e-7 )
		{
			printf("ERROR: in CEpuckProximitySensor::ComputeSensorReadings wrong cell size: fSizeX/unNumCellsX:%f , fSizeY/unNumCellsY:%f , diff:%f\n",fCellSizeX,fCellSizeY,fCellSizeX-fCellSizeY);
		}
		double fCellSize = fCellSizeX;


		double fX = fPosX;
		double fY = fPosY;

		double epuckRotation = pc_epuck->GetRotation();
		/* For EverySensor */
		for ( int sensorIndex = 0 ; sensorIndex < NUM_PROXIMITY_SENSORS ; sensorIndex++)
		{
			/* Init values */
			bool bWallEncountered = false;
			fX = fPosX;
			fY = fPosY;
			double fRayLength = 0;

			/* Get Ray of sensor  */
			double fRayAngle = NormalizeAngle ( epuckRotation + m_fIRSensorDir[sensorIndex] );
		
			//printf("Robot PosX: %2f, Robot PosY: %2f, Robot Orien: %2f, fRayAngle: %2f\n", fPosX, fPosY, epuckRotation, fRayAngle);
			/* Whle not wall and ray less than range */
			while( !bWallEncountered && fRayLength < PROXIMITY_MAX_DISTANCE)
			{
				/* Calc ray on next cell */
				fX += (fCellSize/50) * cos(fRayAngle);
				fY += (fCellSize/50) * sin(fRayAngle);
				
				/* Calc fRayLength */
				fRayLength = sqrt ( pow (fX - fPosX, 2) + pow (fY - fPosY, 2 ) );
			
				/* If obstacle on that position */
				if( pcArena->GetHeight(fX,fY) >= HEIGHT_OBSTACLE )
				{
					/* If closer than previos meausurement */
					if (GetInput(sensorIndex) > fRayLength)
					{
						SetInput(sensorIndex,fRayLength);	
						bWallEncountered = true;
					}
				}
				
				//printf("fRayLength: %2f\n", fRayLength);
			}
		}
	}

	NormalizeReadings();//between 0 and 1

	return GetInputs();
}


/******************************************************************************/
/******************************************************************************/

const double* CEpuckProximitySensor::GetIRReadings()
{
    return m_fIRReadings;
}

/******************************************************************************/
/******************************************************************************/

void CEpuckProximitySensor::PrintIRReadings()
{
    //printf("IR proximity readings (no noise): ");
    //double* pfCurrentInput = GetInputs();
    //for( int i=0 ; i<NUM_PROXIMITY_SENSORS ; i++ )
    //{
        //printf("%f  ", pfCurrentInput[i]);
    //}
    //printf("\n");
}

/******************************************************************************/
/******************************************************************************/

void CEpuckProximitySensor::LoadIRSamplesFromFile( int n_robotnumber, int n_type )
{
    //if( n_type == EPUCK_2_EPUCK_PROXIMITY)
    //{
        //switch( n_robotnumber ) {
        //case 249:
        //{
            //m_unNumSensors_epuck2epuck = unNumSensors_249E;
            //m_unNumDistances_epuck2epuck = unNumDistances_249E;
            //m_unNumDirections_epuck2epuck = unNumDirections_249E;
            //m_unNumReadings_epuck2epuck = unNumDirections_249E * unNumDistances_249E;
            //m_fRange_epuck2epuck = fRange_249E;

            //if( m_unNumSensors_epuck2epuck != NUM_PROXIMITY_SENSORS ) {
                //printf("ERROR5: 'LoadIRSamplesFromFile' inconsistent number of proximity sensors.\n");
                //return;
            //}
            //DoReadings_249E();
            //m_ppunReadings_epuck2epuck = ppunReadings_249E;
            //m_ppunSituationIndex_epuck2epuck = ppunSituationIndex_249E;
            //break;
        //}
        //case 250:
        //{
        //}
        //case 252:
        //{
            //m_unNumSensors_epuck2epuck = unNumSensors_252E;
            //m_unNumDistances_epuck2epuck = unNumDistances_252E;
            //m_unNumDirections_epuck2epuck = unNumDirections_252E;
            //m_unNumReadings_epuck2epuck = unNumDirections_252E * unNumDistances_252E;
            //m_fRange_epuck2epuck = fRange_252E;

            //if( m_unNumSensors_epuck2epuck != NUM_PROXIMITY_SENSORS ) {
                //printf("ERROR5: 'LoadIRSamplesFromFile' inconsistent number of proximity sensors.\n");
                //return;
            //}
            //DoReadings_252E();
            //m_ppunReadings_epuck2epuck = ppunReadings_252E;
            //m_ppunSituationIndex_epuck2epuck = ppunSituationIndex_252E;
            //break;
        //}
        //default:
        //{
            //printf("ERROR: You try to load a sampling with a wrong robot number\n");
            //exit(1);
        //}
        //}
    //}
    //else if( n_type == EPUCK_2_WALL_PROXIMITY) {
        //switch( n_robotnumber ) {
        //case 249:
        //{
            //m_unNumSensors_epuck2wall = unNumSensors_249W;
            //m_unNumDistances_epuck2wall = unNumDistances_249W;
            //m_unNumDirections_epuck2wall = unNumDirections_249W;
            //m_unNumReadings_epuck2wall = unNumDirections_249W * unNumDistances_249W;
            //m_fRange_epuck2wall = fRange_249W;

            //if( m_unNumSensors_epuck2epuck != NUM_PROXIMITY_SENSORS ) {
                //printf("ERROR5: 'LoadIRSamplesFromFile' inconsistent number of proximity sensors.\n");
                //return;
            //}
            //DoReadings_249W();
            //m_ppunReadings_epuck2wall = ppunReadings_249W;
            //m_ppunSituationIndex_epuck2wall = ppunSituationIndex_249W;
            //break;
        //}
        //case 250:
        //{
        //}
        //case 252:
        //{
            //m_unNumSensors_epuck2wall = unNumSensors_252W;
            //m_unNumDistances_epuck2wall = unNumDistances_252W;
            //m_unNumDirections_epuck2wall = unNumDirections_252W;
            //m_unNumReadings_epuck2wall = unNumDirections_252W * unNumDistances_252W;
            //m_fRange_epuck2wall = fRange_252W;

            //if( m_unNumSensors_epuck2epuck != NUM_PROXIMITY_SENSORS ) {
                //printf("ERROR5: 'LoadIRSamplesFromFile' inconsistent number of proximity sensors.\n");
                //return;
            //}
            //DoReadings_252W();
            //m_ppunReadings_epuck2wall = ppunReadings_252W;
            //m_ppunSituationIndex_epuck2wall = ppunSituationIndex_252W;
            //break;
        //}
        //default:
        //{
            //printf("ERROR: You try to load a sampling with a wrong robot number\n");
            //exit(1);
        //}
        //}
    //}
    //else {
        //printf("ERROR: You try to use a wrong proximity situation\n");
        //exit(1);
    //}
}

/******************************************************************************/
/******************************************************************************/

void CEpuckProximitySensor::LoadIRSamplesFromFile(  const char* pc_filename , int n_type )
{
    //ifstream in;
    //in.open( pc_filename, ios::in );
    //if( !in ) {
        //printf("ERROR1: 'LoadIRSamplesFromFile' could not open file '%s'.\n",  pc_filename );
        //return;
    //}

    //unsigned int unNumSensors, unNumDistances, unNumDirections;
    //double fRange;

    //if( !(in >> unNumSensors) ||
        //!(in >> unNumDistances) ||
        //!(in >> unNumDirections) ||
        //!(in >> fRange) ) {
        //printf("ERROR2: 'LoadIRSamplesFromFile' in reading file '%s'.\n", pc_filename );
        //return;
    //}

   ////printf("Proximity sensor loaded \n");
   ////printf("num_sensors %ld , num_distances %ld , num_directions %ld , prox_range %f \n",unNumSensors,unNumDistances,unNumDirections,fRange);
    //// allocate memory for the new reading table and the new index table
    //unsigned int unNumReadings = unNumDistances * unNumDirections;

    //int i;
    //unsigned int** ppunReadings = new unsigned int*[unNumReadings];
    //double** ppunSituationIndex = new double*[unNumReadings];
    //for( i = 0; i < unNumReadings; i++ ) {
        //ppunReadings[i] = new unsigned int[unNumSensors];
        //ppunSituationIndex[i] = new double[2];
    //}

    //// reads the sensor values
    //for( i = 0; i < unNumReadings; i++ ) {
        //double unDistance;
        //double unDirection;
        //if( !(in >> unDistance) ||
            //!(in >> unDirection) ) {
            //printf("ERROR3: 'LoadIRSamplesFromFile' in reading file '%s'.\n", pc_filename );
            //return;
        //}
        //else {
            //ppunSituationIndex[i][0] = unDistance;
            //ppunSituationIndex[i][1] = unDirection;
        //}
        //for( int j = 0; j < unNumSensors; j++ ) {
            //if( !(in >> ppunReadings[i][j]) ) {
                //printf("ERROR4: 'LoadIRSamplesFromFile' in reading file '%s'.\n", pc_filename );
                //return;
            //}
        //}
    //}

    //// assign the new table to the correct variables
    //switch( n_type ) {
    //case EPUCK_2_EPUCK_PROXIMITY:
    //{
        //m_unNumSensors_epuck2epuck 		= unNumSensors;
        //m_unNumDistances_epuck2epuck 	= unNumDistances;
        //m_unNumDirections_epuck2epuck 	= unNumDirections;
	//m_unNumReadings_epuck2epuck     = unNumReadings;
        //m_fRange_epuck2epuck		= fRange;
        ////m_fRange_epuck2stoy		= m_fRange_epuck2epuck;

        //if( m_unNumSensors_epuck2epuck != NUM_PROXIMITY_SENSORS ) {
            //printf("ERROR5: 'LoadIRSamplesFromFile' inconsistent number of proximity sensors in file '%s'.\n", pc_filename );
            //return;
        //}
        //m_ppunReadings_epuck2epuck		 = ppunReadings;
        //m_ppunSituationIndex_epuck2epuck = ppunSituationIndex;
        //break;
    //}
    //case EPUCK_2_WALL_PROXIMITY:
    //{
        //m_unNumSensors_epuck2wall 		= unNumSensors;
        //m_unNumDistances_epuck2wall 	= unNumDistances;
        //m_unNumDirections_epuck2wall 	= unNumDirections;
	//m_unNumReadings_epuck2wall     = unNumReadings;
        //m_fRange_epuck2wall		= fRange;

        //if( m_unNumSensors_epuck2wall != NUM_PROXIMITY_SENSORS ) {
            //printf("ERROR5: 'LoadIRSamplesFromFile' inconsistent number of proximity sensors in file '%s'.\n", pc_filename );
            //return;
        //}
        //m_ppunReadings_epuck2wall		= ppunReadings;
        //m_ppunSituationIndex_epuck2wall = ppunSituationIndex;
        //break;
    //}
    //default:
    //{
        //printf("ERROR6: 'LoadIRSamplesFromFile' unknown object type\n" );
        //break;
    //}
    //}

    //in.close();
}

/******************************************************************************/
/******************************************************************************/

//f_distance is the distance from centre of mass to centre of mass
//for proximity what counts is the distance to the surface
//this function calculates the proximity readings for one particular object
//and then sets each value of m_fIRReadings to the max of the new value itsself
void CEpuckProximitySensor::AddObjectToIRReadings( double f_distance , double f_direction , int n_type )
{
    //if( n_type == EPUCK_2_EPUCK_PROXIMITY )
    //{

        //if( f_distance > m_fRange_epuck2epuck )
        //{
            //f_distance = m_fRange_epuck2epuck;
        //}
        //else if( f_distance < PROXIMITY_MIN_DISTANCE )
        //{
            //f_distance = PROXIMITY_MIN_DISTANCE;
        //}

        //// compute the table index from distance and direction
        //// CAREFULLY CONTROL THIS PART
        //unsigned int unIndex = GetIndexFast(f_distance, f_direction, EPUCK_2_EPUCK_PROXIMITY);

        //double* pfCurrentInput = GetInputs();
				//for( int i=0 ; i<NUM_PROXIMITY_SENSORS ; i++ )
        //{
            //double fThisReading=fmax(pfCurrentInput[i],(double)(m_ppunReadings_epuck2epuck[unIndex][i]));
						//SetInput(i, fThisReading);
        //}
    //}
    //else if ( n_type == EPUCK_2_WALL_PROXIMITY )
    //{

        //if( f_distance > m_fRange_epuck2wall )
        //{
            //f_distance = m_fRange_epuck2wall;
        //}
        //else if( f_distance < PROXIMITY_MIN_DISTANCE )
        //{
            //f_distance = PROXIMITY_MIN_DISTANCE;
        //}

        //// compute the table index from distance and direction
        //// CAREFULLY CONTROL THIS PART
        //unsigned int unIndex = GetIndexFast(f_distance, f_direction, EPUCK_2_WALL_PROXIMITY);

        //double* pfCurrentInput = GetInputs();
				//for( int i=0 ; i<NUM_PROXIMITY_SENSORS ; i++ )
        //{
            //double fThisReading=fmax(pfCurrentInput[i],(double)(m_ppunReadings_epuck2wall[unIndex][i]));
            //if(m_nNoiseStatus)
                //ApplyNoise(fThisReading);
						//SetInput(i, fThisReading);
        //}
    //}
    //else
    //{
        //printf("ERROR: proximity type not implemented yet!! \n");
    //}
}

/******************************************************************************/
/******************************************************************************/

unsigned int CEpuckProximitySensor::GetIndexFast(double f_distance , double f_direction, int n_type)
{
    //unsigned int numDirections, numDistances, numReadings;
    //double** situationTable;

    //if(n_type == EPUCK_2_EPUCK_PROXIMITY) 
    //{
        //numDirections = m_unNumDirections_epuck2epuck;
        //numDistances = m_unNumDistances_epuck2epuck;
    //}
    //else if(n_type == EPUCK_2_WALL_PROXIMITY) 
    //{
        //numDirections = m_unNumDirections_epuck2wall;
        //numDistances = m_unNumDistances_epuck2wall;
    //}

    //// I assume the following is always true. If not, you should implement additionnal characs 
    //// samples go from [0 to 0.2] in distance by steps of 0.01
    //// samples go from [0 to 6.10865] in direction by steps of 0.174533
    //int distanceIndex = (int) rint (f_distance / 0.01);
    //int directionIndex = (int) rint (f_direction / 0.174533);
    //// implement circularity : after 6.2 radians, we fall back on 0.0 radians ...
    //if (directionIndex == 36) 
	//directionIndex = 0;

    //int indexFinal = directionIndex + distanceIndex * numDirections;

    //return indexFinal;
}

/******************************************************************************/
/******************************************************************************/

unsigned int CEpuckProximitySensor::GetIndexWithManhattanDistance(double f_distance , double f_direction, int n_type)
{
    //unsigned int unDistanceIndex, unDirectionIndex, unIndexFinal;
    //unsigned int unNumDirections, unNumDistances, unNumReadings;
    //double** ppunSituationIndex;

    //if(n_type == EPUCK_2_EPUCK_PROXIMITY) 
    //{
        //unNumDirections = m_unNumDirections_epuck2epuck;
        //unNumDistances = m_unNumDistances_epuck2epuck;
        //ppunSituationIndex = m_ppunSituationIndex_epuck2epuck;
	//unNumReadings = m_unNumReadings_epuck2epuck;
    //}
    //else if(n_type == EPUCK_2_WALL_PROXIMITY) 
    //{
        //unNumDirections = m_unNumDirections_epuck2wall;
        //unNumDistances = m_unNumDistances_epuck2wall;
        //ppunSituationIndex = m_ppunSituationIndex_epuck2wall;
	//unNumReadings = m_unNumReadings_epuck2wall;
    //}

    //// To find the unDistanceIndex, the idea is to navigate in the ppunSituationIndex matrix
    //// by steps of lenght = unNumDirections (to consider only the first line of each different
    //// distance). If our f_distance is between the distance of the current line and the one
    //// of the next distance step, we check which distance in the matrix is the closer one.
    //for (int i=0, j=0; i < unNumReadings; i+=unNumDirections, j++ )
    //{
        //if ( f_distance >= ppunSituationIndex[i][0] && f_distance <= ppunSituationIndex[i+unNumDirections][0] )
	//{
            //if (i == unNumDistances-1 || fabs(f_distance-ppunSituationIndex[i][0]) < fabs(f_distance-ppunSituationIndex[i+unNumDirections][0]) )
	    //{
		//unDistanceIndex = j;
		//break;
	    //}
            //else
	    //{
		//unDistanceIndex = j+1;
		//break;
	    //}
	//}
    //}

    //// To find the unDirectionIndex, the idea is to focalize on the lines with the good distance
    //// and to search in there where we have the closest direction to f_direction
    //for (int i=0, j=0; i < unNumDirections; i++, j++ )
    //{
        //if( f_direction >= ppunSituationIndex[i][1] && (i == unNumDirections-1 || f_direction < ppunSituationIndex[i+1][1]) )
	//{
	    //// we are at the last line for direction
            //if(i == unNumDirections - 1)
	    //{
		////angles are originally < 2*M_PI
                //if (fabs(f_direction-ppunSituationIndex[i][1]) < fabs(ppunSituationIndex[0][1] + 2.0 * M_PI-f_direction) )
		//{
                    //unDirectionIndex = j;
		//}
                //else
		//{
		    //unDirectionIndex = 0;
		//}
	    //}
	    //else
	    //{
		//if (fabs(f_direction-ppunSituationIndex[i][1]) < fabs(f_direction-ppunSituationIndex[j+1][1]) )
		//{
		    //unDirectionIndex = j; 
		//}
		//else
		//{
		    //unDirectionIndex = j+1;
		//}
	    //}
	    //break;
	//}
    //}

    //unIndexFinal = unDirectionIndex + unDistanceIndex * unNumDirections;

    //return unIndexFinal;
}

/******************************************************************************/
/******************************************************************************/

void CEpuckProximitySensor::ApplyNoise(double& f_value)
{
    double fNoise = Random::nextDouble(-NOISE_MAX_VALUE, NOISE_MAX_VALUE);
    f_value += fNoise;
}

/******************************************************************************/
/******************************************************************************/

const double* CEpuckProximitySensor::GetSensorDirections()
{
    return m_fIRSensorDir;
}

/******************************************************************************/
/******************************************************************************/

void CEpuckProximitySensor::NormalizeReadings()
{
    for( int i=0 ; i<m_unNumberOfInputs ; i++ )
    {
			//double fNormalizedInput = m_pfInputs[i]/CEpuckProximitySensor::PROXIMITY_MAX_VALUE;
        double fNormalizedInput = -(m_pfInputs[i]/PROXIMITY_MAX_DISTANCE) + 1;
				if (fNormalizedInput > 1.0 ) fNormalizedInput = 1.0;
				if (fNormalizedInput < 0.0 ) fNormalizedInput = 0.0;

				SetInput(i,fNormalizedInput);
    }
}

/******************************************************************************/
/******************************************************************************/

void CEpuckProximitySensor::GetMaxReading( unsigned int* un_sensor_index , double* f_sensor_value )
{
    *un_sensor_index = 0;
    *f_sensor_value = 0;
    for( int i=0 ; i<m_unNumberOfInputs ; i++ )
    {
        if( m_pfInputs[i] > *f_sensor_value )
        {
            *f_sensor_value  = m_pfInputs[i];
            *un_sensor_index = i;
        }
    }
    *f_sensor_value *= CEpuckProximitySensor::PROXIMITY_MAX_VALUE;
}

/******************************************************************************/
/******************************************************************************/

void CEpuckProximitySensor::SetNoiseMaxValue( double f_noise ) {
  NOISE_MAX_VALUE = f_noise;
}

/******************************************************************************/
/******************************************************************************/

double CEpuckProximitySensor::GetNoiseMaxValue( void ) {
  return NOISE_MAX_VALUE;
}

/******************************************************************************/
/******************************************************************************/

void CEpuckProximitySensor::SetNoiseStatus( int n_noise_status ) {
  m_nNoiseStatus = n_noise_status;
}

/******************************************************************************/
/******************************************************************************/

int CEpuckProximitySensor::GetNoiseStatus( void ) {
  return m_nNoiseStatus;
}


/******************************************************************************/
/******************************************************************************/

double* CEpuckProximitySensor::GetSensorReading ( CEpuck *p_pcEpuck )
{
	//double* readings = GetInputs();
	//double* anticlockwise = new double[m_unNumberOfInputs];

	//int i,j;
	//for( i=0, j=( m_unNumberOfInputs - 1 ) ; i<m_unNumberOfInputs ; i++,j-- )
	//{
		//anticlockwise[j] = readings[i];
	//}
	//return anticlockwise;
	return GetInputs();
}


/******************************************************************************/
/******************************************************************************/

double CEpuckProximitySensor::GetMaxRange ( void ) 
{
	return PROXIMITY_MAX_DISTANCE; 
}

/******************************************************************************/
/******************************************************************************/

unsigned int CEpuckProximitySensor::GetType()
{
    return SENSOR_PROXIMITY;
}

