
/******************* INCLUDES ******************/
/***********************************************/

/******************** General ******************/

#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <sys/time.h>

/******************** Simulator ****************/
#include <vector>
#include "simmath.h"
#include "testevoexp.h"
#include "programmedarena.h"
#include "random.h"

/******************** Sensors ******************/
#include "proxsensor.h"

/******************** Actuators ****************/
#include "wheelsactuator.h"

/******************** Controllers **************/
#include "perceptroncontroller.h"

using namespace std;

/*Create Arena */
static char* pchHeightMap = 
"%%%%%%%%%%%%%%%%%%%%"
"%##################%"
"%##################%"
"%##################%"
"%###########%%%####%"
"%##################%"
"%##################%"
"%##################%"
"%##################%"
"%##################%"
"%##%%%#############%"
"%##################%"
"%##################%"
"%##################%"
"%##################%"
"%##################%"
"%###########%%%####%"
"%##################%"
"%##################%"
"%%%%%%%%%%%%%%%%%%%%";


extern gsl_rng* rng;
extern long int rngSeed;

/*******************************************************************************/
/*******************************************************************************/

CTestEvoExp::CTestEvoExp(const char* pch_name, const char* paramsFile) :
	CExperiment(pch_name, COLLISION_MODEL_SIMPLE, COLLISION_HANDLER_POSITION)
{

	/* If there is not a parameter file input get default values*/
	if (paramsFile == NULL )
	{
		m_nRobotsNumber = 1;
		SetNumberOfEpucks(m_nRobotsNumber);
		m_fUpperBounds = 5.0;
		m_fLowerBounds = -5.0;

	}
	/* Else, extract info from the file */
	/* (NOTE: STILL NOT IMPLEMENTED */
	else
	{
		/* I SHOULD WORK ON THIS */
		m_nRobotsNumber = 1;
		SetNumberOfEpucks(m_nRobotsNumber);
		m_fUpperBounds = 5.0;
		m_fLowerBounds = -5.0;
	}
}

/******************************************************************************/
/******************************************************************************/

CArena* CTestEvoExp::CreateArena()
{
	/* Create Arena */
	CArena* pcArena = NULL;
	pcArena = new CProgrammedArena("CProgrammedArena", 20, 20, 3.0, 3.0);
	((CProgrammedArena*)pcArena)->SetHeightPixelsFromChars(pchHeightMap, ' ', '#', '%');

	return pcArena;
}

/******************************************************************************/
/******************************************************************************/

void CTestEvoExp::AddActuators(CEpuck* pc_epuck)
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

void CTestEvoExp::AddSensors(CEpuck* pc_epuck)
{

	/* Create and add Proximity Sensor */
	CSensor* pcProxSensor = NULL;
	pcProxSensor = new CEpuckProximitySensor(252);
	pc_epuck->AddSensor(pcProxSensor);

}

/******************************************************************************/
/******************************************************************************/

void CTestEvoExp::SetController(CEpuck* pc_epuck)
{
	/* Create PERCEPTRON controller */
	char pchTemp[128];
	sprintf(pchTemp, "PerceptronController");
	CPerceptronController* pcController = new CPerceptronController(pchTemp, pc_epuck);
	pcController->SetUpperBounds(m_fUpperBounds);
	pcController->SetLowerBounds(m_fLowerBounds);

	//m_unChromosomeLength = 18;
	///* Set Weights */
	//double* pfpWeights = new double[m_unChromosomeLength];
	//for (int i = 0; i < m_unChromosomeLength; i++)
	//{
		//pfpWeights[i] = m_pfChromosome[i];
	//}

	//pcController->SetWeights(m_unChromosomeLength, pfpWeights);
	//delete[] pfpWeights;  

	//printf("SIZE: %d",sizeof(m_pfChromosome));
	//pcController->SetWeights(sizeof(m_pfChromosome), m_pfChromosome);
	/* Add PERCEPTRON controller */
	pc_epuck->SetControllerType( CONTROLLER_PERCEPTRON );
	pc_epuck->SetController(pcController);
}

/******************************************************************************/
/******************************************************************************/

void CTestEvoExp::CreateAndAddEpucks(CSimulator* pc_simulator)
{
	/* Create and add epucks */
	char label[100] = "epuck";    
	for (int i = 0; i < m_nRobotsNumber; i++)
	{
		sprintf(label, "epuck%0.4d", i);
		CEpuck* pcEpuck = CreateEpuck(label, 1.3, 1.3, 1.57);
		pc_simulator->AddEpuck(pcEpuck);
	}

	Reset();
}


/******************************************************************************/
/******************************************************************************/

void CTestEvoExp::Reset ( void )
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


	float fInitialHeading;
	while(it!=(*vEpucks).end()){                         
		fInitialHeading=Random::nextDouble(-M_PI/2,M_PI/2);
		(*it)->SetCollisions(0);
		if(m_unSampleNumber==0){                                                                   
			((CNNController*)((*it)->GetController()))->SetWeights(m_unChromosomeLength, pfpWeights);
		}
		((CNNController*)((*it)->GetController()))->Reset();        
		//(*it)->SetPosition(Random::nextDouble(-0.5,0.5),Random::nextDouble(-0.5,0.5));
		(*it)->SetPosition(0.0,0.0);
		//(*it)->SetRotation(fInitialHeading);                 
		(*it)->SetRotation(0.0);                 
		((CCollisionEpuck*)(*it))->UpdateCollisionPosition();
		it++;
	}

	delete[] pfpWeights;  

	m_pcCollisionManager->Reset();

}
