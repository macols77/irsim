
/******************* INCLUDES ******************/
/***********************************************/

/******************** General ******************/

#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <sys/time.h>

/******************** Simulator ****************/
#include <vector>
#include "simmath.h"
#include "random.h"
#include "programmedarena.h"

#include "testneuronexp.h"

/******************** Sensors ******************/
#include "contactsensor.h"
#include "epuckproximitysensor.h"
#include "reallightsensor.h"
#include "lightsensor.h"
#include "realbluelightsensor.h"
#include "realredlightsensor.h"
#include "groundsensor.h"
#include "groundmemorysensor.h"
#include "batterysensor.h"
#include "bluebatterysensor.h"
#include "redbatterysensor.h"

/******************** Actuators ****************/
#include "wheelsactuator.h"

/******************** Controllers **************/
#include "nndistributedcontroller.h"

using namespace std;

/*Create Arena */
static char* pchHeightMap = 
"%%%%%%%%%%%%%%%%%%%%"
"%##################%"
"%##################%"
"%##################%"
"%##################%"
"%##################%"
"%##################%"
"%##################%"
"%##################%"
"%##################%"
"%##################%"
"%##################%"
"%##################%"
"%##################%"
"%##################%"
"%##################%"
"%##################%"
"%##################%"
"%##################%"
"%%%%%%%%%%%%%%%%%%%%";

//"%%%%%%%%%%%%%%%%%%%%"
//"%##################%"
//"%##################%"
//"%##########%#######%"
//"%##########%#######%"
//"%#%%%######%#######%"
//"%############%%%###%"
//"%##################%"
//"%##########%#######%"
//"%###########%%%%%##%"
//"%##################%"
//"%%%%%%#############%"
//"%##################%"
//"%##################%"
//"%#####%%###########%"
//"%#####%%###########%"
//"%#####%%%%%########%"
//"%##################%"
//"%##################%"
//"%%%%%%%%%%%%%%%%%%%%";

//"%%%%%%%%%%%%%%%%%%%%"
//"%##################%"
//"%##################%"
//"%##################%"
//"%###########%%%####%"
//"%##################%"
//"%##################%"
//"%##################%"
//"%##################%"
//"%##################%"
//"%##%%%#############%"
//"%##################%"
//"%##################%"
//"%##################%"
//"%##################%"
//"%##################%"
//"%###########%%%####%"
//"%##################%"
//"%##################%"
//"%%%%%%%%%%%%%%%%%%%%";
extern gsl_rng* rng;
extern long int rngSeed;

/*******************************************************************************/
///*******************************************************************************/
//
CTestNeuronExp::CTestNeuronExp(const char* pch_name, const char* paramsFile,
										 unsigned int un_chromosome_length, unsigned int un_fitness_function,
										 double f_evaluation_time, double f_upper_bounds, double f_lower_bounds, 
										 bool b_evolutionary_flag, bool b_learning_flag) :
	CExperiment(pch_name, COLLISION_MODEL_SIMPLE, COLLISION_HANDLER_POSITION)
{

	m_bEvolutionaryFlag = b_evolutionary_flag;
	m_bLearningFlag = b_learning_flag;

	/* If there is not a parameter file input get default values*/
	if (paramsFile == NULL )
	{

		/* EXTRA */
		m_nRobotsNumber = 1;
		SetNumberOfEpucks(m_nRobotsNumber);
		m_nWriteToFile = 0;
		m_pcvRobotPositions = new dVector2[m_nRobotsNumber];
		m_fRobotOrientations = new double[m_nRobotsNumber];
		for ( int i = 0 ; i < m_nRobotsNumber ; i++)
		{
			m_pcvRobotPositions[i].x 	= 0.0;
			m_pcvRobotPositions[i].y 	= 0.0;
			m_fRobotOrientations[i] 	= 0.0;
		}
		m_nRunTime = 10000;

		/* ENVIRONMENT */
		/* LIGHTS */
		m_nLightObjectNumber = 1;
		m_pcvLightObjects = new dVector2[m_nLightObjectNumber];
		for ( int i = 0 ; i < m_nLightObjectNumber; i++){
			m_pcvLightObjects[i].x = 0.25;
			m_pcvLightObjects[i].y = 0.25;
		}
	
		/* BLUE Lights */
		m_nBlueLightObjectNumber = 0;
		m_pcvBlueLightObjects = new dVector2[m_nBlueLightObjectNumber];
		
		/* RED Lights */
		m_nRedLightObjectNumber = 0;
		m_pcvRedLightObjects = new dVector2[m_nRedLightObjectNumber];

		/* PREYS */
		/* Get Ground Area Objects */
		m_nNumberOfGroundArea = 0;
		m_vGroundAreaCenter = new dVector2[m_nNumberOfGroundArea];
		m_fGroundAreaExternalRadius = new double[m_nNumberOfGroundArea];
		m_fGroundAreaInternalRadius = new double[m_nNumberOfGroundArea];
		m_fGroundAreaColor = new double[m_nNumberOfGroundArea];

		
		/* SENSORS */
		/* Light sensor reange */
		m_fLightSensorRange = 1.0; //1 meter
		/* Blue Light sensor reange */
		m_fBlueLightSensorRange = 1.0; //1 meter
		/* Red Light sensor reange */
		m_fRedLightSensorRange = 1.0; //1 meter
		/* Get Battery load range */
		m_fBatterySensorRange = 0.5;
		/* Get batttery charge coef */
		m_fBatteryChargeCoef = 0.01;
		/* Get batttery charge coef */
		m_fBatteryDischargeCoef = 0.0001;


		/* MORPHOLOGY */
		/* Proximity */
		m_unProximitySensorsUsedNumber = CEpuckProximitySensor::SENSOR_NUMBER;
		m_unProximitySensorsUsedValue = new unsigned int[CEpuckProximitySensor::SENSOR_NUMBER];
		for ( int i = 0 ; i < CEpuckProximitySensor::SENSOR_NUMBER ; i++)
		{
			m_unProximitySensorsUsedValue[i] = 1;
		}

		/* Contact */
		m_unContactSensorsUsedNumber = CContactSensor::SENSOR_NUMBER;
		m_unContactSensorsUsedValue = new unsigned int[CContactSensor::SENSOR_NUMBER];
		for ( int i = 0 ; i < CContactSensor::SENSOR_NUMBER ; i++)
		{
			m_unContactSensorsUsedValue[i] = 1;
		}

		/* Light */
		m_unLightSensorsUsedNumber = CRealLightSensor::SENSOR_NUMBER;
		m_unLightSensorsUsedValue = new unsigned int[CRealLightSensor::SENSOR_NUMBER];
		for ( int i = 0 ; i < CRealLightSensor::SENSOR_NUMBER ; i++)
		{
			m_unLightSensorsUsedValue[i] = 1;
		}
		
		/* Blue Light */
		m_unBlueLightSensorsUsedNumber = CRealBlueLightSensor::SENSOR_NUMBER;
		m_unBlueLightSensorsUsedValue = new unsigned int[CRealBlueLightSensor::SENSOR_NUMBER];
		for ( int i = 0 ; i < CRealBlueLightSensor::SENSOR_NUMBER ; i++)
		{
			m_unBlueLightSensorsUsedValue[i] = 1;
		}
		
		/* Red Light */
		m_unRedLightSensorsUsedNumber = CRealRedLightSensor::SENSOR_NUMBER;
		m_unRedLightSensorsUsedValue = new unsigned int[CRealRedLightSensor::SENSOR_NUMBER];
		for ( int i = 0 ; i < CRealRedLightSensor::SENSOR_NUMBER ; i++)
		{
			m_unRedLightSensorsUsedValue[i] = 1;
		}

		/* Ground */
		m_unGroundSensorsUsedNumber = CGroundSensor::SENSOR_NUMBER;
		m_unGroundSensorsUsedValue = new unsigned int[CGroundSensor::SENSOR_NUMBER];
		for ( int i = 0 ; i < CGroundSensor::SENSOR_NUMBER ; i++)
		{
			m_unGroundSensorsUsedValue[i] = 1;
		}

		/* GENETIC */
		m_fUpperBounds = f_upper_bounds;
		m_fLowerBounds = f_lower_bounds;


		/* NEURONAL */
		m_unNumberOfLayers = 0;
		m_unLayerSensorType = new unsigned int[m_unNumberOfLayers];
		m_unLayersOutputs = new unsigned int[m_unNumberOfLayers];
		m_mAdjacencyMatrix = new unsigned int*[m_unNumberOfLayers];
		for ( int i = 0 ; i < m_unNumberOfLayers ; i++ )
		{
			m_mAdjacencyMatrix[i] = new unsigned int[m_unNumberOfLayers];
		}

		/* LEARNING */
		/* Learning parameters */
		m_unLearningLayerFlag = new unsigned int [m_unNumberOfLayers];
		m_unEvoDevoLayerFlag = new unsigned int [m_unNumberOfLayers];
		m_unLearningDiagonalFlag = new unsigned int [m_unNumberOfLayers];	
		/* learning coef */
		m_fEta = 0.02;
		/* forget coef */
		m_fEpsilon = 0.002;	
	}

	/* Read From param file */
	else
	{

		ifstream pfile(paramsFile);
		if(!pfile) {
			cerr << "Can't find parameters file " << endl;
			exit(0);
		}	
		
		/* EXTRA */
		
		/* Get number of robots */
		m_nRobotsNumber = getInt('=',pfile);
		/* Set numer of robots */
		SetNumberOfEpucks(m_nRobotsNumber);
		/* For each robot get position and orientation */
		m_pcvRobotPositions = new dVector2[m_nRobotsNumber];
		m_fRobotOrientations = new double[m_nRobotsNumber];

		for ( int i = 0 ; i < m_nRobotsNumber ; i++)
		{
			m_pcvRobotPositions[i].x 	= getDouble('=',pfile);
			m_pcvRobotPositions[i].y 	= getDouble('=',pfile);
			m_fRobotOrientations[i] 	= getDouble('=',pfile);
		}
		
		/* Get write to file flag */
		m_nWriteToFile 	= getInt('=',pfile);
		m_nRunTime = getInt('=',pfile);
	
		/* ENVIRONMENT */

		/* Lights */
		/* Get Light Objects Number */
		m_nLightObjectNumber = getInt('=',pfile);
		/* Create Objects */
		m_pcvLightObjects = new dVector2[m_nLightObjectNumber];
		for ( int i = 0 ; i < m_nLightObjectNumber; i++){
			/* Get X position */
			m_pcvLightObjects[i].x = getDouble('=',pfile);
			/* Get Y Position */
			m_pcvLightObjects[i].y = getDouble('=',pfile);
		}
		
		/* Blue Lights */
		/* Get Blue Light Objects Number */
		m_nBlueLightObjectNumber = getInt('=',pfile);
		/* Create Objects */
		m_pcvBlueLightObjects = new dVector2[m_nBlueLightObjectNumber];
		for ( int i = 0 ; i < m_nBlueLightObjectNumber; i++){
			/* Get X position */
			m_pcvBlueLightObjects[i].x = getDouble('=',pfile);
			/* Get Y Position */
			m_pcvBlueLightObjects[i].y = getDouble('=',pfile);
		}
		
		/* Red Lights */
		/* Get Red Light Objects Number */
		m_nRedLightObjectNumber = getInt('=',pfile);
		/* Create Objects */
		m_pcvRedLightObjects = new dVector2[m_nRedLightObjectNumber];
		for ( int i = 0 ; i < m_nRedLightObjectNumber; i++){
			/* Get X position */
			m_pcvRedLightObjects[i].x = getDouble('=',pfile);
			/* Get Y Position */
			m_pcvRedLightObjects[i].y = getDouble('=',pfile);
		}
		
		/* Ground Areas */
		/* Get GroundArea Objects */
		m_nNumberOfGroundArea = getInt('=',pfile);
		m_vGroundAreaCenter = new dVector2[m_nNumberOfGroundArea];
		m_fGroundAreaExternalRadius = new double[m_nNumberOfGroundArea];
		m_fGroundAreaInternalRadius = new double[m_nNumberOfGroundArea];
		m_fGroundAreaColor = new double[m_nNumberOfGroundArea];

		for ( int i = 0 ; i < m_nNumberOfGroundArea ; i++)
		{
			m_vGroundAreaCenter[i].x = getDouble('=',pfile);
			m_vGroundAreaCenter[i].y = getDouble('=',pfile);
			m_fGroundAreaInternalRadius[i] = 0.0;
			m_fGroundAreaExternalRadius[i] = getDouble('=',pfile);
			m_fGroundAreaColor[i] = getDouble('=',pfile);

		}
		
		/* SENSORS */
		/* Get Light Range */
		m_fLightSensorRange = getDouble('=',pfile); 
		/* Get Blue Light Range */
		m_fBlueLightSensorRange = getDouble('=',pfile); 
		/* Get Red Light Range */
		m_fRedLightSensorRange = getDouble('=',pfile); 
		
		/* Get Battery load range */
		m_fBatterySensorRange = getDouble('=',pfile);
		/* Get batttery charge coef */
		m_fBatteryChargeCoef = getDouble('=',pfile);
		/* Get batttery charge coef */
		m_fBatteryDischargeCoef = getDouble('=',pfile);
		
		/* Get Blue Battery load range */
		m_fBlueBatterySensorRange = getDouble('=',pfile);
		/* Get Blue batttery charge coef */
		m_fBlueBatteryChargeCoef = getDouble('=',pfile);
		/* Get Blue batttery charge coef */
		m_fBlueBatteryDischargeCoef = getDouble('=',pfile);
		
		/* Get Red Battery load range */
		m_fRedBatterySensorRange = getDouble('=',pfile);
		/* Get Red batttery charge coef */
		m_fRedBatteryChargeCoef = getDouble('=',pfile);
		/* Get Red batttery charge coef */
		m_fRedBatteryDischargeCoef = getDouble('=',pfile);

		/* MORPHOLOGY */
		/* Get Proximity Sensors */
		m_unProximitySensorsUsedNumber = 0;
		m_unProximitySensorsUsedValue = new unsigned int[CEpuckProximitySensor::SENSOR_NUMBER];

		/* Get first element */
		m_unProximitySensorsUsedValue[0] = getInt('=',pfile);
		if ( m_unProximitySensorsUsedValue[0] == 1)
			m_unProximitySensorsUsedNumber++;
		/* Get the others */
		for ( int i = 1 ; i < CEpuckProximitySensor::SENSOR_NUMBER ; i++)
		{
			m_unProximitySensorsUsedValue[i] = getInt(' ',pfile);
			if ( m_unProximitySensorsUsedValue[i] == 1)
				m_unProximitySensorsUsedNumber++;
		}

		/* DEBUG */
		printf("PROXIMITY SENSORS NUMBER: %d -- ",m_unProximitySensorsUsedNumber);
		for ( int i = 0 ; i < CEpuckProximitySensor::SENSOR_NUMBER ; i++)
		{
			printf("%d ",m_unProximitySensorsUsedValue[i]);
		}
		printf("\n");
		/* DEBUG */

		/* Get Contact Sensors */
		m_unContactSensorsUsedNumber = 0;
		m_unContactSensorsUsedValue = new unsigned int[CContactSensor::SENSOR_NUMBER];
		
		/* Get first element */
		m_unContactSensorsUsedValue[0] = getInt('=',pfile);
		if ( m_unContactSensorsUsedValue[0] == 1)
			m_unContactSensorsUsedNumber++;
		/* Get the others */
		for ( int i = 1 ; i < CContactSensor::SENSOR_NUMBER ; i++)
		{
			m_unContactSensorsUsedValue[i] = getInt(' ',pfile);
			if ( m_unContactSensorsUsedValue[i] == 1)
				m_unContactSensorsUsedNumber++;
		}

		/* DEBUG */
		printf("CONTACT SENSORS NUMBER: %d -- ",m_unContactSensorsUsedNumber);
		for ( int i = 0 ; i < CContactSensor::SENSOR_NUMBER ; i++)
		{
			printf("%d ",m_unContactSensorsUsedValue[i]);
		}
		printf("\n");
		/* DEBUG */

		/* Get Light Sensors */
		m_unLightSensorsUsedNumber = 0;
		m_unLightSensorsUsedValue = new unsigned int[CRealLightSensor::SENSOR_NUMBER];
		
		/* Get first element */
		m_unLightSensorsUsedValue[0] = getInt('=',pfile);
		if ( m_unLightSensorsUsedValue[0] == 1)
			m_unLightSensorsUsedNumber++;
		/* Get the others */
		for ( int i = 1 ; i < CRealLightSensor::SENSOR_NUMBER ; i++)
		{
			m_unLightSensorsUsedValue[i] = getInt(' ',pfile);
			if ( m_unLightSensorsUsedValue[i] == 1)
				m_unLightSensorsUsedNumber++;
		}

		/* DEBUG */
		printf("LIGHT SENSORS NUMBER: %d -- ",m_unLightSensorsUsedNumber);
		for ( int i = 0 ; i < CRealLightSensor::SENSOR_NUMBER ; i++)
		{
			printf("%d ",m_unLightSensorsUsedValue[i]);
		}
		printf("\n");
		/* DEBUG */

		/* Get BLUE Light Sensors */
		m_unBlueLightSensorsUsedNumber = 0;
		m_unBlueLightSensorsUsedValue = new unsigned int[CRealBlueLightSensor::SENSOR_NUMBER];
		
		/* Get first element */
		m_unBlueLightSensorsUsedValue[0] = getInt('=',pfile);
		if ( m_unBlueLightSensorsUsedValue[0] == 1)
			m_unBlueLightSensorsUsedNumber++;
		/* Get the others */
		for ( int i = 1 ; i < CRealBlueLightSensor::SENSOR_NUMBER ; i++)
		{
			m_unBlueLightSensorsUsedValue[i] = getInt(' ',pfile);
			if ( m_unBlueLightSensorsUsedValue[i] == 1)
				m_unBlueLightSensorsUsedNumber++;
		}

		/* DEBUG */
		printf("BLUE LIGHT SENSORS NUMBER: %d -- ",m_unBlueLightSensorsUsedNumber);
		for ( int i = 0 ; i < CRealBlueLightSensor::SENSOR_NUMBER ; i++)
		{
			printf("%d ",m_unBlueLightSensorsUsedValue[i]);
		}
		printf("\n");
		/* DEBUG */
		
		/* Get RED Light Sensors */
		m_unRedLightSensorsUsedNumber = 0;
		m_unRedLightSensorsUsedValue = new unsigned int[CRealRedLightSensor::SENSOR_NUMBER];
		
		/* Get first element */
		m_unRedLightSensorsUsedValue[0] = getInt('=',pfile);
		if ( m_unRedLightSensorsUsedValue[0] == 1)
			m_unRedLightSensorsUsedNumber++;
		/* Get the others */
		for ( int i = 1 ; i < CRealRedLightSensor::SENSOR_NUMBER ; i++)
		{
			m_unRedLightSensorsUsedValue[i] = getInt(' ',pfile);
			if ( m_unRedLightSensorsUsedValue[i] == 1)
				m_unRedLightSensorsUsedNumber++;
		}

		/* DEBUG */
		printf("RED LIGHT SENSORS NUMBER: %d -- ",m_unRedLightSensorsUsedNumber);
		for ( int i = 0 ; i < CRealRedLightSensor::SENSOR_NUMBER ; i++)
		{
			printf("%d ",m_unRedLightSensorsUsedValue[i]);
		}
		printf("\n");
		/* DEBUG */
		
		/* Get Ground Sensors */
		m_unGroundSensorsUsedNumber = 0;
		m_unGroundSensorsUsedValue = new unsigned int[CGroundSensor::SENSOR_NUMBER];
		
		/* Get first element */
		m_unGroundSensorsUsedValue[0] = getInt('=',pfile);
		if ( m_unGroundSensorsUsedValue[0] == 1)
			m_unGroundSensorsUsedNumber++;
		/* Get the others */
		for ( int i = 1 ; i < CGroundSensor::SENSOR_NUMBER ; i++)
		{
			m_unGroundSensorsUsedValue[i] = getInt(' ',pfile);
			if ( m_unGroundSensorsUsedValue[i] == 1)
				m_unGroundSensorsUsedNumber++;
		}

		/* DEBUG */
		printf("GROUND SENSORS NUMBER: %d -- ",m_unGroundSensorsUsedNumber);
		for ( int i = 0 ; i < CGroundSensor::SENSOR_NUMBER ; i++)
		{
			printf("%d ",m_unGroundSensorsUsedValue[i]);
		}
		printf("\n");
		/* DEBUG */

		/* GENETIC */
		/* Careful, the genetic lines have been already parsed, so skip. But no areas*/
		for ( int i = 0 ; i < 12 ; i++)
		{
			getDouble('=', pfile);
		}

		m_fInitAreaX = getDouble('=', pfile);
		m_fInitAreaY = getDouble('=', pfile);

		/* Skip upper/lowe bounds already parsed */
		getDouble('=', pfile);
		getDouble('=', pfile);
		
		m_fUpperBounds = f_upper_bounds;
		m_fLowerBounds = f_lower_bounds;

		/* NEURONAL */
		/* Get Number of Layers */
		m_unNumberOfLayers = getInt('=',pfile);

		/* Get Sensors to Layers */
		m_unLayerSensorType = new unsigned int[m_unNumberOfLayers];
		for ( int i = 0 ; i < m_unNumberOfLayers ; i++ )
		{
			m_unLayerSensorType[i] = getInt('=',pfile);
      if (m_unLayerSensorType[i] == SENSOR_LIGHT)
        m_unLayerSensorType[i] = SENSOR_REAL_LIGHT;
      if (m_unLayerSensorType[i] == SENSOR_BLUE_LIGHT)
        m_unLayerSensorType[i] = SENSOR_REAL_BLUE_LIGHT;
      if (m_unLayerSensorType[i] == SENSOR_RED_LIGHT)
        m_unLayerSensorType[i] = SENSOR_REAL_RED_LIGHT;
		}

		/* Get Activation Function */
		m_unActivationFunction = new unsigned int[m_unNumberOfLayers];
		for ( int i = 0 ; i < m_unNumberOfLayers ; i++)
		{
			m_unActivationFunction[i] = getInt('=',pfile);
		}
		
		/* Get Number of Outputs */
		m_unLayersOutputs = new unsigned int[m_unNumberOfLayers];
		for ( int i = 0 ; i < m_unNumberOfLayers ; i++)
		{
			m_unLayersOutputs[i] = getInt('=',pfile);
		}
		
		/* Create Adjacency Matrix */
		m_mAdjacencyMatrix = new unsigned int*[m_unNumberOfLayers];
		for ( int i = 0 ; i < m_unNumberOfLayers ; i++ )
		{
			m_mAdjacencyMatrix[i] = new unsigned int[m_unNumberOfLayers];
		}

		for ( int i = 0 ; i < m_unNumberOfLayers ; i++)
		{
			int j = 0;
			m_mAdjacencyMatrix[i][j] = getInt('=',pfile);
			for ( j = 1 ; j < m_unNumberOfLayers ; j++ )
			{
				m_mAdjacencyMatrix[i][j] = getInt(' ',pfile);
			}
		}

		/* LEARNING */
		/* Learning parameters */
		m_unLearningLayerFlag = new unsigned int [m_unNumberOfLayers];
		m_unEvoDevoLayerFlag = new unsigned int [m_unNumberOfLayers];
		m_unLearningDiagonalFlag = new unsigned int [m_unNumberOfLayers];	
		/* learning coef */
		m_fEta = 0.02;
		/* forget coef */
		m_fEpsilon = 0.002;	
	}
}

/******************************************************************************/
/******************************************************************************/

CTestNeuronExp::~CTestNeuronExp ( void )
{
	/* Extra */
	delete [] m_pcvRobotPositions;
	delete [] m_fRobotOrientations;
	
	/* Environment */	
	delete [] m_pcvLightObjects;
	delete [] m_pcvBlueLightObjects;
	delete [] m_pcvRedLightObjects;
	delete [] m_vGroundAreaCenter;
	delete [] m_fGroundAreaExternalRadius;
	delete [] m_fGroundAreaInternalRadius;
	delete [] m_fGroundAreaColor;

	/* Morphology */
	delete [] m_unProximitySensorsUsedValue;
	delete [] m_unContactSensorsUsedValue;
	delete [] m_unLightSensorsUsedValue;
	delete [] m_unBlueLightSensorsUsedValue;
	delete [] m_unRedLightSensorsUsedValue;
	delete [] m_unGroundSensorsUsedValue;

	/* Genetic */

	/* Neural */
	delete [] m_unLayerSensorType;
	delete [] m_unActivationFunction;
	delete [] m_unLayersOutputs;

	for ( int i = 0 ; i < m_unNumberOfLayers ; i++ ) 
	{
		delete [] m_mAdjacencyMatrix[i];
	}

	/* Learning */
	delete [] m_unLearningLayerFlag;
	delete [] m_unEvoDevoLayerFlag;
	delete [] m_unLearningDiagonalFlag;


}

	/******************************************************************************/
/******************************************************************************/
CArena* CTestNeuronExp::CreateArena()
{
	/* Create Arena */
	CArena* pcArena = NULL;
	pcArena = new CProgrammedArena("CProgrammedArena", 20, 20, 3.0, 3.0);
	((CProgrammedArena*)pcArena)->SetHeightPixelsFromChars(pchHeightMap, ' ', '#', '%');

	/* Create and add Light Object */
	char pchTemp[128];
	CLightObject* pcLightObject = NULL;
	for( int i = 0 ; i < m_nLightObjectNumber ; i++){
		sprintf(pchTemp, "LightObject%d", i);
		CLightObject* pcLightObject = new CLightObject (pchTemp);
		pcLightObject->SetCenter(m_pcvLightObjects[i]);
		pcArena->AddLightObject(pcLightObject);
	}

	/* Create and add Blue Light Object */
	CBlueLightObject* pcBlueLightObject = NULL;
	for( int i = 0 ; i < m_nBlueLightObjectNumber ; i++){
		sprintf(pchTemp, "BlueLightObject%d", i);
		CBlueLightObject* pcBlueLightObject = new CBlueLightObject (pchTemp);
		pcBlueLightObject->SetCenter(m_pcvBlueLightObjects[i]);
		pcArena->AddBlueLightObject(pcBlueLightObject);
	}
	
	/* Create and add Red Light Object */
	CRedLightObject* pcRedLightObject = NULL;
	for( int i = 0 ; i < m_nRedLightObjectNumber ; i++){
		sprintf(pchTemp, "RedLightObject%d", i);
		CRedLightObject* pcRedLightObject = new CRedLightObject (pchTemp);
		pcRedLightObject->SetCenter(m_pcvRedLightObjects[i]);
		pcArena->AddRedLightObject(pcRedLightObject);
	}
	
	/* Create GroundArea */
	char sGroundAreaName[100]="epuck";

	for ( int i = 0 ; i < m_nNumberOfGroundArea ; i++)
	{
		sprintf(sGroundAreaName,"groundArea%d",i);
		CGroundArea* groundArea = new CGroundArea(sGroundAreaName);
		groundArea->SetCenter(m_vGroundAreaCenter[i]);
		groundArea->SetExtRadius(m_fGroundAreaExternalRadius[i]);
		groundArea->SetIntRadius(m_fGroundAreaInternalRadius[i]);
		groundArea->SetColor(m_fGroundAreaColor[i]);
		groundArea->SetHeight(0.20);
		pcArena->AddGroundArea(groundArea);
	}

	return pcArena;
}

/******************************************************************************/
/******************************************************************************/

void CTestNeuronExp::AddActuators(CEpuck* pc_epuck)
{
	/* Create and Add Wheels */
	char pchTemp[128];
	sprintf(pchTemp, "actuators_%s", pc_epuck->GetName());
	CActuator* pcActuator = NULL;
	pcActuator = new CWheelsActuator(pchTemp, pc_epuck);
	pc_epuck->AddActuator(pcActuator);
}

/******************************************************************************/
/******************************************************************************/

void CTestNeuronExp::AddSensors(CEpuck* pc_epuck)
{
	//
	/* Create and add Proximity Sensor */
	CSensor* pcProxSensor = NULL;
	pcProxSensor = new CEpuckProximitySensor(252);
	pc_epuck->AddSensor(pcProxSensor);

	//Light Sensor
	CSensor* pcLightSensor = NULL;
	pcLightSensor = new CRealLightSensor("Real Light Sensor", m_fLightSensorRange);
	pc_epuck->AddSensor(pcLightSensor);
	
  //Light Sensor
  //CSensor* pcLightSensorB = NULL;
  //pcLightSensorB = new CLightSensor("Light Sensor", m_fLightSensorRange);
  //pc_epuck->AddSensor(pcLightSensorB);
	
	//Blue Light Sensor
	CSensor* pcBlueLightSensor = NULL;
	pcBlueLightSensor = new CRealBlueLightSensor("Real Blue Light Sensor", m_fBlueLightSensorRange);
	pc_epuck->AddSensor(pcBlueLightSensor);
	
	//Red Light Sensor
	CSensor* pcRedLightSensor = NULL;
	pcRedLightSensor = new CRealRedLightSensor("Real Red Light Sensor", m_fRedLightSensorRange);
	pc_epuck->AddSensor(pcRedLightSensor);
	
	//Contact Sensor
	CSensor* pcContactSensor = NULL;
	pcContactSensor = new CContactSensor("Contact Sensor");
	pc_epuck->AddSensor(pcContactSensor);
	
	//Ground Sensor
	CSensor* pcGroundSensor = NULL;
	pcGroundSensor = new CGroundSensor("Ground Sensor");
	pc_epuck->AddSensor(pcGroundSensor);
	
	//Ground Memory Sensor
	CSensor* pcGroundMemorySensor = NULL;
	pcGroundMemorySensor = new CGroundMemorySensor("Ground Memory Sensor");
	pc_epuck->AddSensor(pcGroundMemorySensor);
	
	//Battery Sensor
	CSensor* pcBatterySensor = NULL;
	pcBatterySensor = new CBatterySensor("Battery Sensor", m_fBatterySensorRange, m_fBatteryChargeCoef, m_fBatteryDischargeCoef);
	pc_epuck->AddSensor(pcBatterySensor);
	
	//Blue Battery Sensor
	CSensor* pcBlueBatterySensor = NULL;
	pcBlueBatterySensor = new CBlueBatterySensor("Battery Sensor", m_fBlueBatterySensorRange, m_fBlueBatteryChargeCoef, m_fBlueBatteryDischargeCoef);
	pc_epuck->AddSensor(pcBlueBatterySensor);
	
	//Red Battery Sensor
	CSensor* pcRedBatterySensor = NULL;
	pcRedBatterySensor = new CRedBatterySensor("Battery Sensor", m_fRedBatterySensorRange, m_fRedBatteryChargeCoef, m_fRedBatteryDischargeCoef);
	pc_epuck->AddSensor(pcRedBatterySensor);
}

/******************************************************************************/
/******************************************************************************/

void CTestNeuronExp::SetController(CEpuck* pc_epuck)
{
	char pchTemp[128];
	sprintf(pchTemp, "Neuron");
	CNNDistributedController* pcController = new CNNDistributedController(pchTemp, pc_epuck, m_unNumberOfLayers, m_unLayersOutputs, m_unLayerSensorType, m_unActivationFunction, m_mAdjacencyMatrix, m_unLearningLayerFlag, m_unEvoDevoLayerFlag, m_unLearningDiagonalFlag, m_fLowerBounds, m_fUpperBounds, m_bEvolutionaryFlag, m_bLearningFlag, m_fEta, m_fEpsilon, m_nWriteToFile, m_unProximitySensorsUsedNumber, m_unProximitySensorsUsedValue, m_unContactSensorsUsedNumber, m_unContactSensorsUsedValue, m_unLightSensorsUsedNumber, m_unLightSensorsUsedValue,  m_unGroundSensorsUsedNumber, m_unGroundSensorsUsedValue, m_unBlueLightSensorsUsedNumber, m_unBlueLightSensorsUsedValue, m_unRedLightSensorsUsedNumber, m_unRedLightSensorsUsedValue);
	//pcController->SetUpperBounds(m_fUpperBounds);
	//pcController->SetLowerBounds(m_fLowerBounds);
	pc_epuck->SetControllerType( CONTROLLER_NEURON );
	pc_epuck->SetController(pcController);

}

/******************************************************************************/
/******************************************************************************/

void CTestNeuronExp::CreateAndAddEpucks(CSimulator* pc_simulator)
{
	/* Create and add epucks */
	char label[100] = "epuck";    
	for (int i = 0; i < m_nRobotsNumber; i++)
	{
		sprintf(label, "epuck%0.4d", i);
		CEpuck* pcEpuck = CreateEpuck(label, m_pcvRobotPositions[i].x, m_pcvRobotPositions[i].y, m_fRobotOrientations[i]);
		pc_simulator->AddEpuck(pcEpuck);
		pc_simulator->SetTimeLimit(m_nRunTime);
	}

	//Reset();
}


/******************************************************************************/
/******************************************************************************/

void CTestNeuronExp::Reset ( void )
{
	m_fFitness=0.0;
	
	double* pfpWeights = new double[m_unChromosomeLength];                       
	if(m_unSampleNumber==0){//If not the controller must be the same               
		for (int i = 0; i < m_unChromosomeLength; i++)
		{
			pfpWeights[i] = m_pfChromosome[i];
		}
	}

	TEpuckVector* vEpucks=m_pcSimulator->GetEpucks();
	TEpuckIterator it=(*vEpucks).begin();

	int i=0;
	while(it!=(*vEpucks).end()){                        
		(*it)->SetCollisions(0);
		if(m_unSampleNumber==0){                                                                  
			((CNNDistributedController*)((*it)->GetController()))->SetWeights(m_unChromosomeLength, pfpWeights);
		}
		((CNNDistributedController*)((*it)->GetController()))->Reset();        
		//(*it)->SetPosition(0.0,0.0);
		(*it)->SetPosition(m_pcvRobotPositions[i].x, m_pcvRobotPositions[i].y);
		(*it)->SetRotation(m_fRobotOrientations[i]);                 

		((CCollisionEpuck*)(*it))->UpdateCollisionPosition();

		/* Reset Battery sensor, needed to evolutionary */
		CBatterySensor* battery = (CBatterySensor*) (*it)->GetSensor(SENSOR_BATTERY);
		battery->Reset();
		/* Reset Blue Battery sensor, needed to evlutionary */
		CBlueBatterySensor* blueBattery = (CBlueBatterySensor*) (*it)->GetSensor(SENSOR_BLUE_BATTERY);
		blueBattery->Reset();
		/* Reset Red Battery sensor, needed to evolutionary */
		CRedBatterySensor* redBattery = (CRedBatterySensor*) (*it)->GetSensor(SENSOR_RED_BATTERY);
		redBattery->Reset();
		/* Reset ground memory sensor, needed to evolutionary */
		CGroundMemorySensor* ground = (CGroundMemorySensor*) (*it)->GetSensor(SENSOR_GROUND_MEMORY);
		ground->Reset();

    /* Get arena */
    CArena* pc_arena = m_pcSimulator->GetArena();
	
    vector<CLightObject*> vLightObject=pc_arena->GetLightObject();
	  vector<CLightObject*>::iterator it_lightobject=vLightObject.begin();

	  while(it_lightobject!=vLightObject.end())
    {
      (*it_lightobject)->Reset();
      it_lightobject++;
    }
	
    vector<CBlueLightObject*> vBlueLightObject=pc_arena->GetBlueLightObject();
    vector<CBlueLightObject*>::iterator it_bluelightobject=vBlueLightObject.begin();
    while(it_bluelightobject!=vBlueLightObject.end())
    {
      (*it_bluelightobject)->Reset();
      it_bluelightobject++;
    }
	
    vector<CRedLightObject*> vRedLightObject=pc_arena->GetRedLightObject();
    vector<CRedLightObject*>::iterator it_redlightobject=vRedLightObject.begin();
    while(it_redlightobject!=vRedLightObject.end())
    {
      (*it_redlightobject)->Reset();
      it_redlightobject++;
    }

		
		it++;
		i++;
	}

	delete [] pfpWeights;  

	m_pcCollisionManager->Reset();
}

/******************************************************************************/
/******************************************************************************/

void CTestNeuronExp::RandomPositionAndOrientation ( void )
{
	TEpuckVector* vEpucks=m_pcSimulator->GetEpucks();
	TEpuckIterator it=(*vEpucks).begin();

	int i;
	while(it!=(*vEpucks).end()){                        
		(*it)->SetCollisions(0);
		
		double randomX;
		double randomY;
		int robotsMounted;
		double dist;

		//int initAreaRadius = 1.5;
		if (m_nRobotsNumber > 1 )
		{
			do
			{
				robotsMounted = 0;

				randomX = ( Random::nextDouble() * m_fInitAreaX * 2.0 ) - m_fInitAreaX;
				randomY = ( Random::nextDouble() * m_fInitAreaY * 2.0 ) - m_fInitAreaY;
				dist = sqrt(randomX*randomX + randomY*randomY);

				for (TEpuckIterator j = vEpucks->begin() ; j != vEpucks->end() ; j ++){
					float aux1 = (*j)->GetPosition().x - randomX;
					float aux2 = (*j)->GetPosition().y - randomY;

					float dist2 = sqrt(aux1*aux1 + aux2*aux2);

					if( dist2 < 2 * (CEpuck::CHASSIS_RADIUS) ) { robotsMounted = 1; }
				}
			}	
			while (robotsMounted == 1);
		}

		else
		{
				randomX = ( Random::nextDouble() * m_fInitAreaX * 2.0 ) - m_fInitAreaX;
				randomY = ( Random::nextDouble() * m_fInitAreaY * 2.0 ) - m_fInitAreaY;
		}

		(*it)->SetPosition(randomX, randomY);

		float fInitialHeading=Random::nextDouble(-M_PI/2,M_PI/2);
		(*it)->SetRotation(fInitialHeading);                 

		//printf("X: %2f, Y: %2f, O: %2f\n", randomX, randomY, fInitialHeading);
		((CCollisionEpuck*)(*it))->UpdateCollisionPosition();
		
		it++;
		i++;
	}

	m_pcCollisionManager->Reset();
}

