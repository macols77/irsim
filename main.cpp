#include <iostream>
#include <errno.h>
#include <sys/types.h>
#include <fstream>
#include <unistd.h>
#include <sys/stat.h>

//random
#include "random.h"
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <sys/time.h>

//renders
#include "render.h"
#include "nullrender.h"
#include "simpledrawstuffrender.h"

//experiments
#include "experiment.h"
#include "testwheelsexp.h"
#include "testlightexp.h"
#include "testbluelightexp.h"
#include "testproxexp.h"
#include "testcontactexp.h"
#include "braitenbergvehicle2experiment.h"
#include "testevoexp.h"
#include "iri1exp.h"
#include "iri2exp.h"
#include "iri3exp.h"
#include "testgroundexp.h"
#include "testbatteryexp.h"
#include "testneuronexp.h"
#include "subsumptiongarbageexp.h"
#include "subsumptionlightexp.h"
#include "testcomexp.h"
#include "testswitchlightexp.h"
#include "testredlightexp.h"
#include "testswitchlightexp.h"
#include "testencoderexp.h"
#include "testcompassexp.h"
#include "ctrnnexp.h"

//genetic algorithms
#include "population.h"
#include "standardgapopulation.h"
#include "individual.h"

//fitnessfunctions
#include "fitnessfunction.h"
#include "avoidcollisionsfitnessfunction.h"
#include "garbagefitnessfunction.h"
#include "loadfitnessfunction.h"
#include "irifitnessfunction.h"
#include "lightfitnessfunction.h"

using namespace std;

/******************************************************************************/
/******************************************************************************/

/* EXPERIMENTS */

#define WHEELS_EXP 1
#define CONTACT_EXP 2
#define PROX_EXP 3
#define LIGHT_EXP 4
#define BLUE_LIGHT_EXP 5
#define RED_LIGHT_EXP 6
#define TEST_SWITCH_LIGHT 7
#define GROUND_EXP 8
#define BATTERY_EXP 9
#define ENCODER_EXP 10
#define COMPASS_EXP 11
//#define TEST_COM 10

#define BRAITENBERG_VEHICLE2 20
#define NEURON_EXP 21
#define SUBSUMPTION_LIGHT 	22
#define SUBSUMPTION_GARBAGE 23
#define CTRNN_EXP 24
//#define TEST_EVO 22

#define IRI1 30
#define IRI2 31
#define IRI3 32


/******************************************************************************/
/******************************************************************************/

#define POPULATION_STANDARD   0

#define FITNESSFUNCTION_NONE 		0
#define FITNESSFUNCTION_AVOID 	1
#define FITNESSFUNCTION_GARBAGE 2
#define FITNESSFUNCTION_LOAD 		3
#define FITNESSFUNCTION_IRI			4			
#define FITNESSFUNCTION_LIGHT 5			

#define AVERAGE_FITNESS   0
#define BEST_FITNESS      1
#define WORST_FITNESS     2

//filenames

#define FILENAME_MAXGENERATION         "maxgeneration"
#define FILENAME_GENERATION            "generation%d.log"
#define FILENAME_BESTCHROMOSOME        "best%d.log"
#define FILENAME_CURRENTBESTCHROMOSOME "currentbest"
#define FILENAME_FITNESSLOG            "fitness.log"

/******************************************************************************/
/******************************************************************************/
/* FUNCTIONS */

void SetUpExperiment ( unsigned int exp ); 					// Sets up the experiment
void usage ( void ); 																//Prints simulator command line options
void printExpId ( void ); 													//Prints the experiment identifiers

void init_rng ( gsl_rng** RNG );										// Init random chain
void del_rng ( gsl_rng* RNG );											// Del random chain
void set_seed_rng ( gsl_rng* RNG , long int seed );	// Set random seed

void 							RunEvolutionaryAlgorithm ( void ); 																						// Run evolution
CPopulation* 			GetPopulation ( void );																												// Get Population
unsigned int 			GetNumberOfLastGeneration ( void );																						// Get last Generation
void 							LoadGeneration(CPopulation*  pc_population, unsigned int un_generation); 			// Load Generation
double 						EvaluateIndividual(double* pf_chromosome, unsigned int un_chromosome_length);	// Evaluate an individual	
CFitnessFunction* GetFitnessFunction(int n_number, CSimulator* pc_simulator);										// Get fitness function


void GetGeneticParameters ( const char* param_file );
/******************************************************************************/
/******************************************************************************/

/* GLOBAL VARIABLES */

char* 					filename 						= NULL; // Input Filename
char* 					chromosomefilename 	= NULL; // Chromosome Input Filename
unsigned int    g_unExperiment 	= 0;		// Experiment
gsl_rng* 				rng;										// Random variable
long int 				rngSeed;								// Random seed

CRender* 			g_pcRender 			= NULL;	// Render object
CExperiment* 	g_pcExperiment 	= NULL; // Experiment object
CSimulator* 	g_pcSimulator 	= NULL;	// Simulator object

/* Flags */
bool g_bProduceFrames = false;					// Write frames on HD flagi

/* Evolutionary */
unsigned int	g_unFitnessFunction 							= FITNESSFUNCTION_NONE;	// No Fitness function
unsigned int  g_unUseFitnessFromSample 					= AVERAGE_FITNESS;			// Fitness to use (default AVERAGE)
float 				g_fFitnessLimit 									= 0;										// Max fitness allowed
int 					g_nFitnessStagnationLimit 				= 0;										// Number of generations allowed without improvement (0 -> All, other --> number)
unsigned int 	g_unFitnessLimitGenerations 			= 0;										// Max number of generations with max fitness allowed

unsigned int  g_unPopulation 										= POPULATION_STANDARD;	// Population
int  					g_nRestartFromGeneration 					= -1;										// Number to generation to be restarted. -1 None, any other number the generation
unsigned int	g_unNumberOfSamplesPerChromosome 	= 1;										// Number of evaluations per Chromosome (default to 1)
unsigned int 	g_unDeleteSomePopulations					= 25; 									//each g_unDeleteSomePopulations generations the data of current one are written to file (default 25)
char 					g_pchDirectory[256] 							= "geneticDataFiles";		// Genetic dir
unsigned int	g_unRandomPositionOrientation			= 0;										// if 1, it Orders the experiment to create random position and orientation
unsigned int  g_unChromosomeLength  						= 0;										// Lenght of a Chromosome. Should be given by the experiment. ( default 0)
unsigned int  g_unChromosomes       						= 100;									// Number of Individuals (default to 100)
unsigned int	g_unGenerations 									= 100000;								// Number of generations (default to 100)
double 				g_fEvaluationTime     						= 100; 									// Evaluation time limit (default to 100)
double 				g_fRunTime     										= 10000; 									// Evaluation time limit (default to 100)
bool 					g_bIsCrossoverOn 									= true;									// Crossover flag
int 					g_nNumberOfCrossovers 						= 1;										// Number of Crossovers
int 					g_nCrossoverDistance  						= 1;										// Crossover Distance
double 				g_fMutationRate       						= 0.05;									// Mutation rate
int 					g_nNumberOfElites     						= 5;										// Number of Elites

double				g_fUpperBounds										= 1.0;										// Upper Weight Limit 
double 				g_fLowerBounds										= 0.0;										// Lower Weight Limit

bool 					g_bEvolutionaryExperiment 	= false;	// Evolutionary experiment flag
bool 					g_bRestartEvolution 				= false;	// Restart evolution flag
bool 					g_bRandomizeFirstGeneration = false;	// First Generation random flag
bool 					g_bWriteToFiles             = true;		// Write to files flag		
bool 					g_bNNParameters							= false;

bool 					g_bVisual 									= true;
/******************************************************************************/
/******************************************************************************/


int main(int argc, char** argv)
{

  //srand(1357911);
	for(int i=1; i<argc; i++) {

		if(argv[i][0] != '-') {
			usage();
			exit(-1);
		}

		switch(argv[i][1]) {
			/* Evolutionary Experiment */
			case 'e': 
				g_bEvolutionaryExperiment=true;
				printf("Evolutionary mode selected\n");
				break;

			/* Set seed */
			case 's':   
				++i;  // go to seed value
				Random::seed = atoi(argv[i]);
				printf("Set the seed: %d\n",Random::seed);
				break;

			/* Produce frames */
			case 'f': 
				g_bProduceFrames=true;
				printf("Frames will be produced in frames-directory\n");
				break;

			/* Set Experiment */
			case 'E': 
				if(i==argc-1){
					printExpId();
					exit(0);
				}
				i++;
				if(argv[i][0] == '-'){
					printExpId();  
					exit(0);
				}
				g_unExperiment=atoi(argv[i]);
				printf("EXPERIMENT: %d CHOOSEN.\n",g_unExperiment);	
				sleep(1);
				break;

			/* Get Input File */
			case 'p':
				++i;
				filename = argv[i];
				printf("filename: %s CHOOSEN.\n",filename);
				break;

			/* Get Input File */
			case 'c':
				++i;
				chromosomefilename = argv[i];
				printf("chromosomefilename: %s CHOOSEN.\n",chromosomefilename);
				break;
		
			case 'l':
				g_bNNParameters = true;
				printf("Create NN Parameters\n");
				break;
			
			case 'v':
				g_bVisual = false;
				printf("No visual\n");
				break;
			/* Print Help */
			case 'h':
				usage();
				exit(0);
				break;

		}//switch on current argument
	}//loop on command line arguments


	/* Evolutionary Experiment */
	if ( g_bEvolutionaryExperiment )
	{
		/* Set up experiment */
		SetUpExperiment(g_unExperiment);
		/* Run evolution */
		RunEvolutionaryAlgorithm();
	}
	
	/* Standard Experiment */
	else
	{
		/* Set up experiment */
		SetUpExperiment(g_unExperiment);
		
		/* Select Arena */
		int ArenaType = g_pcExperiment->GetSimulator()->GetArena()->GetArenaType();
		/* Set if frames to be produce */
		g_pcRender->SetProduceFrames(g_bProduceFrames);
		/* Start simulation */
		g_pcRender->Start();
	}
}

/******************************************************************************/
/******************************************************************************/

void SetUpExperiment(unsigned int exp){

	switch (exp)
	{
		case WHEELS_EXP:
			
			printf("TEST WHEELS EXPERIMENT STARTED \n");
			init_rng(&rng);
			/* Create new experiment */
			g_pcExperiment = new CTestWheelsExp("Wheels Experiment", filename);
			/* Create new simulation */
			g_pcSimulator = g_pcExperiment->CreateSimulator();
			/* Add experiment to simulation */
			g_pcSimulator->AddChild(g_pcExperiment);
			/* Create render */
			if ( g_bVisual == true )
				g_pcRender=new CSimpleDrawStuffRender(g_pcExperiment->GetSimulator(),0,NULL);
			else
				g_pcRender=new CNullRender(g_pcSimulator);

			break;

		case CONTACT_EXP:

			printf("TEST CONTACT EXPERIMENT STARTED \n");
			init_rng(&rng);
			/* Create new experiment */
			g_pcExperiment = new CTestContactExp("Contact Experiment", filename);
			/* Create new simulation */
			g_pcSimulator = g_pcExperiment->CreateSimulator();
			/* Add experiment to simulation */
			g_pcSimulator->AddChild(g_pcExperiment);
			/* Create render */
			if ( g_bVisual == true )
				g_pcRender=new CSimpleDrawStuffRender(g_pcExperiment->GetSimulator(),0,NULL);
			else
				g_pcRender=new CNullRender(g_pcSimulator);

			break;


		case LIGHT_EXP:

			printf("TEST LIGHT EXPERIMENT STARTED \n");
			init_rng(&rng);
			/* Create new experiment */
			g_pcExperiment = new CTestLightExp("Light Experiment", filename);
			/* Create new simulation */
			g_pcSimulator = g_pcExperiment->CreateSimulator();
			/* Add experiment to simulation */
			g_pcSimulator->AddChild(g_pcExperiment);
			/* Create render */
			if ( g_bVisual == true )
				g_pcRender=new CSimpleDrawStuffRender(g_pcExperiment->GetSimulator(),0,NULL);
			else
				g_pcRender=new CNullRender(g_pcSimulator);

			break;

		case BLUE_LIGHT_EXP:

			printf("TEST BLUE EXPERIMENT STARTED \n");
			init_rng(&rng);
			/* Create new experiment */
			g_pcExperiment = new CTestBlueLightExp("Blue Light Experiment", filename);
			/* Create new simulation */
			g_pcSimulator = g_pcExperiment->CreateSimulator();
			/* Add experiment to simulation */
			g_pcSimulator->AddChild(g_pcExperiment);
			/* Create render */
			if ( g_bVisual == true )
				g_pcRender=new CSimpleDrawStuffRender(g_pcExperiment->GetSimulator(),0,NULL);
			else
				g_pcRender=new CNullRender(g_pcSimulator);

			break;
		
		case RED_LIGHT_EXP:

			printf("TEST RED EXPERIMENT STARTED \n");
			init_rng(&rng);
			/* Create new experiment */
			g_pcExperiment = new CTestRedLightExp("Red Light Experiment", filename);
			/* Create new simulation */
			g_pcSimulator = g_pcExperiment->CreateSimulator();
			/* Add experiment to simulation */
			g_pcSimulator->AddChild(g_pcExperiment);
			/* Create render */
			if ( g_bVisual == true )
				g_pcRender=new CSimpleDrawStuffRender(g_pcExperiment->GetSimulator(),0,NULL);
			else
				g_pcRender=new CNullRender(g_pcSimulator);

			break;

		case PROX_EXP:

			printf("TEST PROX EXPERIMENT STARTED \n");
			init_rng(&rng);
			/* Create new experiment */
			g_pcExperiment = new CTestProxExp("Prox Experiment", filename);
			/* Create new simulation */
			g_pcSimulator = g_pcExperiment->CreateSimulator();
			/* Add experiment to simulation */
			g_pcSimulator->AddChild(g_pcExperiment);
			/* Create render */
			if ( g_bVisual == true )
				g_pcRender=new CSimpleDrawStuffRender(g_pcExperiment->GetSimulator(),0,NULL);
			else
				g_pcRender=new CNullRender(g_pcSimulator);

			break;

    case ENCODER_EXP:
			printf("TEST ENCODER EXPERIMENT STARTED \n");
			init_rng(&rng);
			/* Create new experiment */
			g_pcExperiment = new CTestEncoderExp("Encoder Experiment", filename);
			/* Create new simulation */
			g_pcSimulator = g_pcExperiment->CreateSimulator();
			/* Add experiment to simulation */
			g_pcSimulator->AddChild(g_pcExperiment);
			/* Create render */
			if ( g_bVisual == true )
				g_pcRender=new CSimpleDrawStuffRender(g_pcExperiment->GetSimulator(),0,NULL);
			else
				g_pcRender=new CNullRender(g_pcSimulator);
      break;
    
    case COMPASS_EXP:
      printf("TEST COMPASS EXPERIMENT STARTED \n");
      init_rng(&rng);
      /* Create new experiment */
      g_pcExperiment = new CTestCompassExp("Compass Experiment", filename);
      /* Create new simulation */
      g_pcSimulator = g_pcExperiment->CreateSimulator();
      /* Add experiment to simulation */
      g_pcSimulator->AddChild(g_pcExperiment);
      /* Create render */
      if ( g_bVisual == true )
        g_pcRender=new CSimpleDrawStuffRender(g_pcExperiment->GetSimulator(),0,NULL);
      else
        g_pcRender=new CNullRender(g_pcSimulator);
      break;
		case BRAITENBERG_VEHICLE2:
		
			printf("BRAITENBERG VEHICLE 2 \n");
			init_rng(&rng);
			/* Create new experiment */
			g_pcExperiment = new CBraitenbergVehicle2Experiment("Braitenberg Vehicle 2", filename);
			/* Create new simulation */
			g_pcSimulator = g_pcExperiment->CreateSimulator();
			/* Add experiment to simulation */
			g_pcSimulator->AddChild(g_pcExperiment);
			/* Create render */
			if ( g_bVisual == true )
				g_pcRender=new CSimpleDrawStuffRender(g_pcExperiment->GetSimulator(),0,NULL);
			else
				g_pcRender=new CNullRender(g_pcSimulator);

			break;
	
		case GROUND_EXP:
		
			printf("TEST GROUND EXPERIMENT STARTED\n");
			init_rng(&rng);
			/* Create new experiment */
			g_pcExperiment = new CTestGroundExp("Ground Exp", filename);
			/* Create new simulation */
			g_pcSimulator = g_pcExperiment->CreateSimulator();
			/* Add experiment to simulation */
			g_pcSimulator->AddChild(g_pcExperiment);
			/* Create render */
			if ( g_bVisual == true )
				g_pcRender=new CSimpleDrawStuffRender(g_pcExperiment->GetSimulator(),0,NULL);
			else
				g_pcRender=new CNullRender(g_pcSimulator);

			break;
	
		case BATTERY_EXP:
		
			printf("TEST BATTERY EXPERIMENT STARTED \n");
			init_rng(&rng);
			/* Create new experiment */
			g_pcExperiment = new CTestBatteryExp("Battery Experiment", filename);
			/* Create new simulation */
			g_pcSimulator = g_pcExperiment->CreateSimulator();
			/* Add experiment to simulation */
			g_pcSimulator->AddChild(g_pcExperiment);
			/* Create render */
			if ( g_bVisual == true )
				g_pcRender=new CSimpleDrawStuffRender(g_pcExperiment->GetSimulator(),0,NULL);
			else
				g_pcRender=new CNullRender(g_pcSimulator);

			break;
		
		//case TEST_EVO:
			
			//printf("TEST EVOLUTIONARY ROBOTICS \n");
			//init_rng(&rng);
			///* Evolutionary Part */
			//g_unChromosomeLength	= 18;
			//g_unFitnessFunction   = FITNESSFUNCTION_LIGHT;

			///* Create new experiment */
			//g_pcExperiment = new CTestEvoExp("Evo Exp", filename);
			///* Create new simulation */
			//g_pcSimulator = g_pcExperiment->CreateSimulator();
			///* Add experiment to simulation */
			//g_pcSimulator->AddChild(g_pcExperiment);
			
			///* If evolutionary, evol */
			//if(g_bEvolutionaryExperiment){
				//g_pcRender=new CNullRender(g_pcSimulator);
				//g_pcSimulator->SetTimeLimit(g_fEvaluationTime);
			//}
			///* If not, load weights and create render */
			//else{
				//char inputFile[128];
				///* If not chromosome file given */
				//if ( chromosomefilename == NULL)
				//{
					//sprintf(inputFile, "%s/" FILENAME_CURRENTBESTCHROMOSOME, g_pchDirectory);
					////sprintf(inputFile, "%s/best0.log", g_pchDirectory);
				//}
				///* If chromosome file given */
				//else
				//{
					//sprintf(inputFile,"%s",chromosomefilename);
				//}
					
				//ifstream in(inputFile);

				//in >> g_unChromosomeLength; 
				//double*      pfChromosome;
				//pfChromosome = new double[g_unChromosomeLength];
				//for ( int i = 0 ; i < g_unChromosomeLength ; i++ )
				//{
					//in >> pfChromosome[i];
				//}
				//g_pcExperiment->SetChromosome(pfChromosome, g_unChromosomeLength);
				//g_pcExperiment->Reset();

				//g_pcRender=new CSimpleDrawStuffRender(g_pcExperiment->GetSimulator(),0,NULL);
			//}

			//break;

		case NEURON_EXP:
			
			printf("TEST NEURON EXP \n");
			init_rng(&rng);

			/* Get genetic Parameters */
			GetGeneticParameters ( filename );
			/* Create new experiment */
			g_pcExperiment = new CTestNeuronExp("Neuron Exp", filename, g_unChromosomeLength, g_unFitnessFunction, g_fEvaluationTime, g_fUpperBounds, g_fLowerBounds, g_bEvolutionaryExperiment, g_bNNParameters);
			/* Create new simulation */
			g_pcSimulator = g_pcExperiment->CreateSimulator();
			/* Add experiment to simulation */
			g_pcSimulator->AddChild(g_pcExperiment);
			
			/* If evolutionary, evol */
			if(g_bEvolutionaryExperiment)
			{
				//g_pcRender=new CSimpleDrawStuffRender(g_pcExperiment->GetSimulator(),0,NULL);
				g_pcRender=new CNullRender(g_pcSimulator);
				g_pcSimulator->SetTimeLimit(g_fEvaluationTime);
			}
	
			/* If not genetic and nor learning, load weights and create render */
			else
			{
				//g_pcSimulator->SetTimeLimit(g_fRunTime);
				//if ( g_bNNParameters )
				//{
					//FILE* fileLearning = fopen("learningFiles/initFile","a");
					//fprintf(fileLearning,"%d ",g_unChromosomeLength);
					//fclose(fileLearning);
					//printf("HOLA\n");
					//fflush(stdout);
				//}

				char inputFile[128];
				/* If not chromosome file given */
				if ( chromosomefilename == NULL)
				{
					sprintf(inputFile, "%s/" FILENAME_CURRENTBESTCHROMOSOME, g_pchDirectory);
					//sprintf(inputFile, "%s/best0.log", g_pchDirectory);
				}
				/* If chromosome file given */
				else
				{
					sprintf(inputFile,"%s",chromosomefilename);
				}
					
				ifstream in(inputFile);

				in >> g_unChromosomeLength; 
				double*      pfChromosome;
				pfChromosome = new double[g_unChromosomeLength];
				for ( int i = 0 ; i < g_unChromosomeLength ; i++ )
				{
					in >> pfChromosome[i];
				}
				g_pcExperiment->SetChromosome(pfChromosome, g_unChromosomeLength);
				g_pcExperiment->Reset();

				if ( g_bVisual == true )
					g_pcRender=new CSimpleDrawStuffRender(g_pcExperiment->GetSimulator(),0,NULL);
				else
					g_pcRender=new CNullRender(g_pcSimulator);
			}

			break;

		case CTRNN_EXP:
			
			printf("TEST NEURON EXP \n");
			init_rng(&rng);

			/* Get genetic Parameters */
			GetGeneticParameters ( filename );
			/* Create new experiment */
			g_pcExperiment = new CCTRNNExp("Neuron Exp", filename, g_unChromosomeLength, g_unFitnessFunction, g_fEvaluationTime, g_fUpperBounds, g_fLowerBounds, g_bEvolutionaryExperiment, g_bNNParameters);
			/* Create new simulation */
			g_pcSimulator = g_pcExperiment->CreateSimulator();
			/* Add experiment to simulation */
			g_pcSimulator->AddChild(g_pcExperiment);
			
			/* If evolutionary, evol */
			if(g_bEvolutionaryExperiment)
			{
				//g_pcRender=new CSimpleDrawStuffRender(g_pcExperiment->GetSimulator(),0,NULL);
				g_pcRender=new CNullRender(g_pcSimulator);
				g_pcSimulator->SetTimeLimit(g_fEvaluationTime);
			}
	
			/* If not genetic and nor learning, load weights and create render */
			else
			{
				//g_pcSimulator->SetTimeLimit(g_fRunTime);
				//if ( g_bNNParameters )
				//{
					//FILE* fileLearning = fopen("learningFiles/initFile","a");
					//fprintf(fileLearning,"%d ",g_unChromosomeLength);
					//fclose(fileLearning);
					//printf("HOLA\n");
					//fflush(stdout);
				//}

				char inputFile[128];
				/* If not chromosome file given */
				if ( chromosomefilename == NULL)
				{
					sprintf(inputFile, "%s/" FILENAME_CURRENTBESTCHROMOSOME, g_pchDirectory);
					//sprintf(inputFile, "%s/best0.log", g_pchDirectory);
				}
				/* If chromosome file given */
				else
				{
					sprintf(inputFile,"%s",chromosomefilename);
				}
					
				ifstream in(inputFile);

				in >> g_unChromosomeLength; 
				double*      pfChromosome;
				pfChromosome = new double[g_unChromosomeLength];
				for ( int i = 0 ; i < g_unChromosomeLength ; i++ )
				{
					in >> pfChromosome[i];
				}
				g_pcExperiment->SetChromosome(pfChromosome, g_unChromosomeLength);
				g_pcExperiment->Reset();

				if ( g_bVisual == true )
					g_pcRender=new CSimpleDrawStuffRender(g_pcExperiment->GetSimulator(),0,NULL);
				else
					g_pcRender=new CNullRender(g_pcSimulator);
			}

			break;



		case IRI1:
		
			printf("TEST IRI 1 EXPERIMENT STARTED \n");
			init_rng(&rng);
			/* Create new experiment */
			g_pcExperiment = new CIri1Exp("Iri1 Experiment", filename);
			/* Create new simulation */
			g_pcSimulator = g_pcExperiment->CreateSimulator();
			/* Add experiment to simulation */
			g_pcSimulator->AddChild(g_pcExperiment);
			/* Create render */
			if ( g_bVisual == true )
				g_pcRender=new CSimpleDrawStuffRender(g_pcExperiment->GetSimulator(),0,NULL);
			else
				g_pcRender=new CNullRender(g_pcSimulator);

			break;
		
		case IRI2:
		
			printf("TEST IRI 2 EXPERIMENT STARTED \n");
			init_rng(&rng);
			/* Create new experiment */
			g_pcExperiment = new CIri2Exp("Iri2 Experiment", filename);
			/* Create new simulation */
			g_pcSimulator = g_pcExperiment->CreateSimulator();
			/* Add experiment to simulation */
			g_pcSimulator->AddChild(g_pcExperiment);
			/* Create render */
			if ( g_bVisual == true )
				g_pcRender=new CSimpleDrawStuffRender(g_pcExperiment->GetSimulator(),0,NULL);
			else
				g_pcRender=new CNullRender(g_pcSimulator);

			break;


		case IRI3:
		
			printf("TEST IRI 3 EXPERIMENT STARTED \n");
			init_rng(&rng);
			/* Create new experiment */
			g_pcExperiment = new CIri3Exp("Iri3 Experiment", filename);
			/* Create new simulation */
			g_pcSimulator = g_pcExperiment->CreateSimulator();
			/* Add experiment to simulation */
			g_pcSimulator->AddChild(g_pcExperiment);
			/* Create render */
			if ( g_bVisual == true )
				g_pcRender=new CSimpleDrawStuffRender(g_pcExperiment->GetSimulator(),0,NULL);
			else
				g_pcRender=new CNullRender(g_pcSimulator);

			break;
		
		case SUBSUMPTION_GARBAGE:
		
			printf("SUBSUMPTION GARBAGE EXPERIMENT STARTED \n");
			init_rng(&rng);
			/* Create new experiment */
			g_pcExperiment = new CSubsumptionGarbageExp("Subsumption Garbage Experiment", filename);
			/* Create new simulation */
			g_pcSimulator = g_pcExperiment->CreateSimulator();
			/* Add experiment to simulation */
			g_pcSimulator->AddChild(g_pcExperiment);
			/* Create render */
			if ( g_bVisual == true )
				g_pcRender=new CSimpleDrawStuffRender(g_pcExperiment->GetSimulator(),0,NULL);
			else
				g_pcRender=new CNullRender(g_pcSimulator);
			
			break;

		case SUBSUMPTION_LIGHT:
		
			printf("SUBSUMPTION LIGHT EXPERIMENT STARTED \n");
			init_rng(&rng);
			/* Create new experiment */
			g_pcExperiment = new CSubsumptionLightExp("Subsumption Light Experiment", filename);
			/* Create new simulation */
			g_pcSimulator = g_pcExperiment->CreateSimulator();
			/* Add experiment to simulation */
			g_pcSimulator->AddChild(g_pcExperiment);
			/* Create render */
			if ( g_bVisual == true )
				g_pcRender=new CSimpleDrawStuffRender(g_pcExperiment->GetSimulator(),0,NULL);
			else
				g_pcRender=new CNullRender(g_pcSimulator);
			
			break;
	
		//case TEST_COM:
			//printf("TEST COM EXPERIMENT STARTED\n");
			//init_rng(&rng);
			///* Create new experiment */
			//g_pcExperiment = new CTestComExp("Test Com Experiment", filename);
			///* Create new simulation */
			//g_pcSimulator = g_pcExperiment->CreateSimulator();
			///* Add experiment to simulation */
			//g_pcSimulator->AddChild(g_pcExperiment);
			///* Create render */
			//if ( g_bVisual == true )
				//g_pcRender=new CSimpleDrawStuffRender(g_pcExperiment->GetSimulator(),0,NULL);
			//else
				//g_pcRender=new CNullRender(g_pcSimulator);
			
			//break;

		case TEST_SWITCH_LIGHT:
			printf("TEST SWITCH LIGHT EXPERIMENT STARTED\n");
			init_rng(&rng);
			/* Create new experiment */
			g_pcExperiment = new CTestSwitchLightExp("Test Switch Light Experiment", filename);
			/* Create new simulation */
			g_pcSimulator = g_pcExperiment->CreateSimulator();
			/* Add experiment to simulation */
			g_pcSimulator->AddChild(g_pcExperiment);
			/* Create render */
			if ( g_bVisual == true )
				g_pcRender=new CSimpleDrawStuffRender(g_pcExperiment->GetSimulator(),0,NULL);
			else
				g_pcRender=new CNullRender(g_pcSimulator);
			
			break;

		/* No valid experiment */
		default:
			printf("SIMULATOR ERROR, UNKNOWN EXPERIMENT: %d\n",exp);
			exit(1);

			break;
	}

}

/******************************************************************************/
/******************************************************************************/

void RunEvolutionaryAlgorithm()
{
  time_t       tStartTime;
  time(&tStartTime);

  unsigned int unGenerationWithBestFitness = 0;
  double       fBestFitnessRecorded = -10000000;
  unsigned int unEvaluations = 0;

  char pchTerminationReason[1024] = "Error or user interruption";

  if (g_unFitnessFunction == FITNESSFUNCTION_NONE)
  {
    printf("You are trying to run an evolution without using a fitness function\n");
    fflush(stdout);
    exit(-1);
  }

  double fGlobalFitness     = 0;
  double fBestFitness       = 0;
  double fWorstFitness      = 0;
  double fAverageFitness    = 0;
  int    nStartGeneration   = 0;

  CPopulation* pcPopulation = GetPopulation();

  if (g_bRestartEvolution)
  {
    g_nRestartFromGeneration = GetNumberOfLastGeneration();
  }

  // Load a generation from file if necessary:

  if (g_nRestartFromGeneration >= 0)
  {
    LoadGeneration(pcPopulation, g_nRestartFromGeneration);

    pcPopulation->GetGenerationStats(&fWorstFitness, &fAverageFitness, &fBestFitness);
    fGlobalFitness = fBestFitness;

    nStartGeneration = g_nRestartFromGeneration + 1;
  }

  //if (g_bWriteToFiles)
  //{
  //WriteGNUPlotFile();
  //}


  unsigned int unConsecutiveFitnessLimitGenerations = 0;
  unsigned int unCurrentGeneration;


  double*      pfChromosome;
  unsigned int unChromosomeLength;
  void*        pvIndividual;

  for (unCurrentGeneration = nStartGeneration;
      unCurrentGeneration < g_unGenerations;
      unCurrentGeneration++)
  {

    /* Check if GENERATION is STAGNATED */
    if (g_nFitnessStagnationLimit > 0 && unCurrentGeneration != nStartGeneration &&
        unGenerationWithBestFitness + g_nFitnessStagnationLimit < unCurrentGeneration)
    {
      sprintf(pchTerminationReason, "Fitness stagnated and has not improved for %d generations. Stopping evolution.\n", g_nFitnessStagnationLimit);
      break;
    }

    /* Check if FITNESS reached its LIMIT */
    if (fGlobalFitness >= g_fFitnessLimit  )
    {
      unConsecutiveFitnessLimitGenerations++;
      if (unConsecutiveFitnessLimitGenerations >= g_unFitnessLimitGenerations && g_unFitnessLimitGenerations > 0)
      {
        sprintf(pchTerminationReason, "Fitness limit (%.5f) exceeded %.5f by generation %d", g_fFitnessLimit, fGlobalFitness, unCurrentGeneration-1);
        break;
      }
    } 
    else {
      unConsecutiveFitnessLimitGenerations = 0;
    }

    /* Check if GENERATION must be RANDOMIZED */
    if (unCurrentGeneration == 0 || (g_bRandomizeFirstGeneration && unCurrentGeneration == nStartGeneration))
    {
      printf("Creating random generation...\n");
      pcPopulation->CreateRandomPopulation(g_unChromosomes);
    } 
    else {
      pcPopulation->CreateNextGeneration();
    }

    /* reset POPULATION */
    pfChromosome = NULL;
    unChromosomeLength = 0;
    pvIndividual = NULL;

    // Let the experiment know the current generation and fitness
    // such that the arena, sensors, etc. can be modified accordingly.
    // Useful if the experiment relies on incremental evolution:
    g_pcExperiment->SetGeneration(unCurrentGeneration);
    g_pcExperiment->SetFitness(fGlobalFitness);
    //printf("Calling initialize generation method, evolutionary algorithm already started\n");
    g_pcExperiment->InitializeGeneration();

    int unCurrentChromosome = 0;
    while (pcPopulation->GetNextChromosomeToEvaluate(&pvIndividual,
          &pfChromosome,
          &unChromosomeLength))
    {
      time_t tTimeUsed;
      time(&tTimeUsed);
      tTimeUsed -= tStartTime;
      //        CoolPrompt(unCurrentGeneration, g_unGenerations, unCurrentChromosome, pcPopulation->GetNumberOfChromosomes(), unEvaluations, tTimeUsed);

      double fAccumulatedFitness = 0;
      double fBestSampleFitness  = -1e10;
      double fWorstSampleFitness = 1e10;

      for (int nSample = 0; nSample < g_unNumberOfSamplesPerChromosome; nSample++)
      {
        //printf("SAMPLE: %d\n", nSample);
        /*    if (g_bUseMultipleExperiments)
              {
              SetNextExperimentInMultipleExperimentList(nSample);
              g_pcExperiment->SetGeneration(unCurrentGeneration);
              g_pcExperiment->SetFitness(fGlobalFitness);
              }
              */
        //	printf("Setting experiment sample number: %d\n",nSample);
        g_pcExperiment->SetSampleNumber(nSample);
        //printf("NEW SAMPLE NEW SAMPLE NEW SAMPLE: %d    ###########################################\n",nSample);
        //fflush(stdout);
        //printf("Evaluating chromosome: %d\n",unCurrentChromosome);
        double fThisSampleFitness = EvaluateIndividual(pfChromosome, unChromosomeLength);
        //printf("IN MAIN  Eval result: %f    ############################################\n",fThisSampleFitness);
        //			sleep(1);
        fAccumulatedFitness += fThisSampleFitness;
        if (fThisSampleFitness < fWorstSampleFitness){
          fWorstSampleFitness = fThisSampleFitness;
        }
        if (fThisSampleFitness > fBestSampleFitness){
          fBestSampleFitness = fThisSampleFitness;
        }	
      }
      //printf("Number of samples for this chromosome has reached max, next one\n");
      // Set the result depending on if we should use the best, average or worst fitess among the samples:
      if (g_unUseFitnessFromSample == AVERAGE_FITNESS)
        pcPopulation->SetEvaluationResult(pvIndividual, fAccumulatedFitness / g_unNumberOfSamplesPerChromosome);
      else if (g_unUseFitnessFromSample == BEST_FITNESS)
        pcPopulation->SetEvaluationResult(pvIndividual, fBestSampleFitness);
      else
        pcPopulation->SetEvaluationResult(pvIndividual, fWorstSampleFitness);

      unCurrentChromosome++;
      unEvaluations++;
      //printf("End evaluation, fitness obtained: %f\n",fAccumulatedFitness/g_unNumberOfSamplesPerChromosome);
      //sleep(2);
    }

    double fTemp;

    double fPreviousGenerationsFitness = fGlobalFitness;
    pcPopulation->GetGenerationStats(&fWorstFitness, &fAverageFitness, &fBestFitness);
    fGlobalFitness = fBestFitness;

    printf("Generation %d - best fitness: %f, average: %f, worst: %f                                                \n",
        unCurrentGeneration, fBestFitness, fAverageFitness, fWorstFitness);

    if (fGlobalFitness > fBestFitnessRecorded)
    {

      unGenerationWithBestFitness 	= unCurrentGeneration;
      fBestFitnessRecorded          = fGlobalFitness;
    }


    //-----------------------------------------
    // The code for writing files starts here!
    //-----------------------------------------

    if (g_bWriteToFiles)
    {
      char pchMaxGenerationFilename[1024];
      sprintf(pchMaxGenerationFilename, "%s/" FILENAME_MAXGENERATION, g_pchDirectory);
      FILE* fileMaxGeneration = fopen(pchMaxGenerationFilename, "w");
      fprintf(fileMaxGeneration, "%d", unCurrentGeneration);
      fclose(fileMaxGeneration);
      //delete [] fileMaxGeneration;

      // Write the generation:
      char pchGenerationFilename[1024];
      sprintf(pchGenerationFilename, "%s/" FILENAME_GENERATION, g_pchDirectory, unCurrentGeneration);
      pcPopulation->SavePopulation(pchGenerationFilename);

      /* If some GENERATIONS DO NOT need to be WRITTED */
      if (g_unDeleteSomePopulations > 1)
      {
        /* Always write the FIRST generation */
        if (unCurrentGeneration != nStartGeneration)
        {
          if ((unCurrentGeneration - 1) % g_unDeleteSomePopulations != 0)
          {
            sprintf(pchGenerationFilename, "%s/" FILENAME_GENERATION, g_pchDirectory, unCurrentGeneration - 1);
            char pchCommand[1024];
            sprintf(pchCommand, "rm -rf %s", pchGenerationFilename);
            system(pchCommand);
          }
        }
      }

      // Write the best chromosome:
      unsigned int unChromosomeLength;
      //double*      pfChromosome;
      pcPopulation->GetBestChromosome(&pfChromosome, &unChromosomeLength);

      char pchBestIndividualFilename[1024];
      sprintf(pchBestIndividualFilename, "%s/" FILENAME_BESTCHROMOSOME, g_pchDirectory, unCurrentGeneration);
      FILE* fileBestChromosome = fopen(pchBestIndividualFilename, "w");
      if (fileBestChromosome == NULL)
      {
        printf("Error opening file %s - %s", pchBestIndividualFilename, strerror(errno));
      } else {
        fprintf(fileBestChromosome, "%d ", unChromosomeLength);

        for (int i = 0; i < unChromosomeLength; i++)
        {
          fprintf(fileBestChromosome, "%2.20f ", pfChromosome[i]);
        }
        fclose(fileBestChromosome);
      }
      //delete [] fileBestChromosome;

      char pchCommand[1024];
      sprintf(pchCommand, "cp %s %s/" FILENAME_CURRENTBESTCHROMOSOME, pchBestIndividualFilename, g_pchDirectory);
      system(pchCommand);

      /*		WRITE THE SHOWBEST FILE
      // Write the bestrun file:
      FILE* fileBestRun;
      char pchBestRunFileName[1024];
      sprintf(pchBestRunFileName, "%s/" FILENAME_RUNBESTCHROMOSOME, g_pchDirectory, unCurrentGeneration);

      fileBestRun = fopen(pchBestRunFileName, "w");
      if (fileBestRun == NULL)
      {
      printf("Error creating file: %s - %s", pchBestRunFileName, strerror(errno));
      } else {
      //HOW TO MANAGE THIS WITHOUT ARGUMENTS?
      fprintf(fileBestRun, STANDARDOPTIONS_BESTCHROMOSOME);
      fprintf(fileBestRun, "--set-fitness %f\n", fPreviousGenerationsFitness);
      fprintf(fileBestRun, "--set-generation %d\n", unCurrentGeneration);
      fprintf(fileBestRun, "--load-chromosome %s\n", pchBestIndividualFilename);
      fprintf(fileBestRun, "--random-seed %d\n", pcPopulation->GetGenerationRandomSeed());
      fprintf(fileBestRun, g_pchRunBestChromosome);
      fclose(fileBestRun);
      }
      */
      // Write the fitness log:
      FILE* fileFitness;
      char pchFitnessFileName[1024];
      sprintf(pchFitnessFileName, "%s/" FILENAME_FITNESSLOG, g_pchDirectory);

      fileFitness = fopen(pchFitnessFileName, "a");
      if (fileFitness == NULL)
      {
        printf("Error creating file: %s - %s", pchFitnessFileName, strerror(errno));
      } else {
        if (unCurrentGeneration == nStartGeneration)
        {
          time_t tCurrentTime;
          time(&tCurrentTime);

          fprintf(fileFitness,
              "###############################################################\n"
              "## Evolution started on %s"
              "###############################################################\n",
              ctime(&tCurrentTime));
        }

        fprintf(fileFitness, "%d \t %1.10f \t %1.10f \t %1.10f\n", unCurrentGeneration,
            fBestFitness, fAverageFitness, fWorstFitness);
      }
      fclose(fileFitness);
      //delete [] fileFitness;
    }

  }

  if (unCurrentGeneration == g_unGenerations)
  {
    sprintf(pchTerminationReason, "Evolved till last generation %d", unCurrentGeneration - 1);
  }


  if (g_bWriteToFiles)
  {
    // Write the fitness log:
    FILE* fileFitness;
    char pchFitnessFileName[1024];
    sprintf(pchFitnessFileName, "%s/" FILENAME_FITNESSLOG, g_pchDirectory);

    fileFitness = fopen(pchFitnessFileName, "a");
    if (fileFitness == NULL)
    {
      printf("Error creating file: %s - %s", pchFitnessFileName, strerror(errno));
    } else {
      time_t tCurrentTime;
      time(&tCurrentTime);

      fprintf(fileFitness,
          "###############################################################\n"
          "## Evolution ended on %s"
          "## Reason: %s\n"
          "## The best individual was found in generation %d (fitness: %2.8f)\n"
          "###############################################################\n",
          ctime(&tCurrentTime), pchTerminationReason, unGenerationWithBestFitness, fBestFitnessRecorded);
    }
  }

  printf("\nDone - the best individual was found in generation %d (fitness: %2.8f)\n",
      unGenerationWithBestFitness, fBestFitnessRecorded);
  printf("%s\n", pchTerminationReason);


  //printf("EVERYTHING CLEAR HERE1\n");
  delete [] pfChromosome;
  //printf("EVERYTHING CLEAR HERE2\n");
  //delete pcPopulation;
  //printf("EVERYTHING CLEAR HERE3\n");
  //free(pvIndividual);
  //printf("EVERYTHING CLEAR HERE4\n");
}

/******************************************************************************/
/******************************************************************************/

CPopulation* GetPopulation()
{
	CPopulation* pcPopulation;

	if (g_unPopulation == POPULATION_STANDARD)
	{
		pcPopulation = new CStandardGAPopulation(g_unChromosomeLength);
		((CStandardGAPopulation*)pcPopulation)->SetCrossoverOn(g_bIsCrossoverOn);
	}
	else
	{
		printf("Unknown population type: %d", g_unPopulation);
		fflush(stdout);
	}

	pcPopulation->SetMutationRate(g_fMutationRate);
	pcPopulation->SetNumberOfCrossovers(g_nNumberOfCrossovers);
	pcPopulation->SetElitism(g_nNumberOfElites);
	pcPopulation->SetCrossoverDistance(g_nCrossoverDistance);

	return pcPopulation;
}


/******************************************************************************/
/******************************************************************************/

unsigned int GetNumberOfLastGeneration()
{
	char pchMaxGenerationFilename[256];
	sprintf(pchMaxGenerationFilename, "%s/" FILENAME_MAXGENERATION, g_pchDirectory);
	FILE* fileMaxGeneration = fopen(pchMaxGenerationFilename, "r");
	if (fileMaxGeneration == 0)
	{
		printf("Error opening %s (%s) - cannot restart evolution", pchMaxGenerationFilename, strerror(errno));
		exit(errno);

	}

	int nReturn;
	fscanf(fileMaxGeneration, "%d", &nReturn);
	fclose(fileMaxGeneration);
	fprintf(stderr, "Last generation evolved: %d\n", nReturn);

	return nReturn;

}

/******************************************************************************/
/******************************************************************************/

void LoadGeneration(CPopulation*  pc_population, unsigned int un_generation)
{
	char pchGenerationFilename[256];
	sprintf(pchGenerationFilename, "%s/" FILENAME_GENERATION,
			g_pchDirectory,
			un_generation);

	fprintf(stderr, "Restarting evolution from generation %d (file: %s)\n",
			un_generation, pchGenerationFilename);

	if (!pc_population->LoadPopulation(pchGenerationFilename))
	{
		printf("Error loading generation %d from file %s",
				un_generation,
				pchGenerationFilename);
		exit(-1);
	}
}

/******************************************************************************/
/******************************************************************************/

double EvaluateIndividual(double* pf_chromosome,
		unsigned int un_chromosome_length)
{
	/* DEBUG */
	//printf("Begin evalutation   ----------------------------------------------------\n");
	//printf("Simulation time: %f\n",g_pcSimulator->GetTime());
	//printf("Simulation step: %d\n",g_pcSimulator->GetControlStepNumber());
	//printf("Simulation has ended: %d\n",g_pcSimulator->HasEnded());
	//fflush(stdout);
	/* DEBUG */

	// Construct the chromosome - in case we are using neural arrays,
	// append previously evolves weights weights to the chromosome:
	double* pfNeuralArrayChromosome = NULL;
	//if (g_fNeuralArrayWeights != NULL)
	//{
	//pfNeuralArrayChromosome = (double*) malloc(sizeof(double) * (un_chromosome_length + g_unNeuralArrayWeightsLength));
	//memcpy(pfNeuralArrayChromosome, pf_chromosome, sizeof(double) * un_chromosome_length);
	//memcpy(&pfNeuralArrayChromosome[un_chromosome_length], g_fNeuralArrayWeights, sizeof(double) * g_unNeuralArrayWeightsLength);
	//g_pcExperiment->SetChromosome(pfNeuralArrayChromosome, un_chromosome_length + g_unNeuralArrayWeightsLength);

	//}
	//// Not using neural arrays:
	//else
	//{
	// Set the individual's chromosome:
	//printf("Setting experiment's chromosome\n");
	g_pcExperiment->SetChromosome(pf_chromosome, un_chromosome_length);
	//}



	// Create the simulator:

	//CSimulator* pcSimulator = g_pcExperiment->CreateSimulator();

	// Add the experiment as a child of the sim, so that the experiment can
	// handle events too:
	//pcSimulator->AddChild(g_pcExperiment);


	// Let the render know that we are using a new simulator now:
	//g_pcRender->SetSimulator(pcSimulator);
	// Create a fitness function for this run:


	//CFitnessFunction* pcFitnessFunction = GetFitnessFunction(g_unFitnessFunction, //pcSimulator);

	//printf("B4 exoeriment reset\n");
	g_pcExperiment->Reset();
	if (g_unRandomPositionOrientation)
		g_pcExperiment->RandomPositionAndOrientation();
	//printf("After experiment reset\n");
	CFitnessFunction* pcFitnessFunction = GetFitnessFunction(g_unFitnessFunction, g_pcSimulator);
	//	printf("Got Fitness function\n");
	//pcSimulator->SetTimeLimit(g_fEvaluationTime);
	if (pcFitnessFunction != NULL)
	{
		g_pcSimulator->AddChild(pcFitnessFunction);
		//pcSimulator->AddChild(pcFitnessFunction);
	}
	/*
		 if (g_bDumpObjectHierarchy)
		 {
		 printf("Dumping object hierarchy before starting:\n");

		 pcSimulator->PrintfChildren(0);
		 }
		 */
	// Start the evaluation:
	//printf("Calling start on the render\n");
	g_pcRender->Start();
	//printf("Start called\n");

	/*
		 if (g_bDumpObjectHierarchy)
		 {
		 printf("Dumping object hierarchy after finishing:\n");

		 pcSimulator->PrintfChildren(0);
		 g_bDumpObjectHierarchy = false;
		 }

*/

	// Remove the experiment from the list of children in the sim to
	// keep things clean:
	//pcSimulator->RemoveChild(g_pcExperiment);

	if (pfNeuralArrayChromosome)
		delete pfNeuralArrayChromosome;

	double fFitness = pcFitnessFunction != NULL ? pcFitnessFunction->GetFitness() : 0.0;
	//printf("Obtained fitness vaule: %f\n",fFitness);
	g_pcSimulator->RemoveChild(pcFitnessFunction);
	delete pcFitnessFunction;
	g_pcSimulator->ResetSimulation();
	// Check that fFitness is not "nan", yes, I do mean fFitness != fFitness
	if (fFitness != fFitness)
	{
		fFitness = 0.0;
	}
	//printf("End evaluation\n");
	//fflush(stdout);
	//printf("Evaluation end, fitness obtained: %f   --------------------------\n\n",fFitness);
	//fflush(stdout);
	return fFitness;
}

/******************************************************************************/
/******************************************************************************/
CFitnessFunction* GetFitnessFunction(int n_number, CSimulator* pc_simulator)
{
	CFitnessFunction* pcFitnessFunction = NULL;
	bool bMoveOrDieAdded = false;


	switch (n_number)
	{
		case FITNESSFUNCTION_NONE:
			pcFitnessFunction = NULL;
			break;

		case FITNESSFUNCTION_AVOID:
			pcFitnessFunction=new CAvoidCollisionsFitnessFunction("AvoidCollisionsFitnessFunction", pc_simulator, 1);	
			break;

		case FITNESSFUNCTION_GARBAGE:
			pcFitnessFunction=new CGarbageFitnessFunction("GarbageFitnessFunction", pc_simulator, 1);	
			break;

		case FITNESSFUNCTION_LOAD:
			pcFitnessFunction=new CLoadFitnessFunction("LoadFitnessFunction", pc_simulator, 1);	
			break;
	
		case FITNESSFUNCTION_IRI:
			pcFitnessFunction=new CIriFitnessFunction("IriFitnessFunction", pc_simulator, 1);	
			break;

		case FITNESSFUNCTION_LIGHT:
			pcFitnessFunction = new CLightFitnessFunction("LightFitnessFunction", pc_simulator, 1);
			break;

		default:
			pcFitnessFunction = NULL;
			printf("Unknown fitness function %d", n_number);
			break;
	}
	
	return pcFitnessFunction;
}

/******************************************************************************/
/******************************************************************************/

void usage(){
	printf("RENDSIM, command line options:\n");
	printf("-s seed, uses seed as random seed\n");
	printf("-e, to run the evolutionary mode\n");
	printf("-x id, uses id as suffix for geneticDataFiles folder");
	printf("-f, saves frames in ./frames\n");
	printf("-r num, restarts evolution from generation num\n");
	printf("-E exp_id, runs the experiment identified by exp_id (integer).\n If none is given it prints experiments id\n");
	printf("-H sets the light sensor heading for group taxis experiment\n");
	printf("-S sets the light sensor span for group taxis experiment\n");
	printf("-p filename, introduce paramfile\n");
	printf("-c chromosomefilename, introduce chromosome paramfile\n");
	printf("-l create random genome for the learing function\n");
	printf("-v work with the NULL render\n");
	printf("\n\n -h, prints this help\n");
	exit(0);
}


/******************************************************************************/
/******************************************************************************/

void printExpId()
{
	printf("\nEXPERIMENTS IDENTIFIERS\n");	
	printf("\n");
	printf("1 - TEST WHEELS EXPERIMENT\n");
	printf("2 - TEST CONTACT SENSOR EXPERIMENT\n");
	printf("3 - TEST PROXIMITY SENSOR EXPERIMENT\n");
	printf("4 - TEST LIGHT SENSOR EXPERIMENT\n");
	printf("5 - TEST BLUE LIGHT SENSOR EXPERIMENT\n");
	printf("6 - TEST RED LIGHT SENSOR EXPERIMENT\n");
	printf("7 - TEST SWITCH LIGHT EXPERIMENT\n");
	printf("8 - TEST GROUND SENSOR EXPERIMENT\n");
	printf("9 - TEST BATTERY SENSOR EXPERIMENT\n");
	printf("10 - TEST ENCODER SENSOR EXPERIMENT\n");
	printf("11 - TEST COMPASS SENSOR EXPERIMENT\n");
	printf("\n");
	printf("20 - TEST BRAITENBERG VEHICLE EXPERIMENT\n");
	printf("21 - NEURON EXPERIMENT\n");
	//printf("22 - TEST EVOLUTIONARY EXPERIMENT\n");
	printf("22 - TEST SUBSUMPTION LIGHT EXPERIMENT\n");
	printf("23 - TEST SUBSUMPTION GARBAGE EXPERIMENT\n");
	printf("24 - CTRNN EXPERIMENT\n");
	printf("\n");
	printf("30 - TEST IRI1 EXPERIMENT\n");
	printf("31 - TEST IRI2 EXPERIMENT\n");
	printf("32 - TEST IRI3 EXPERIMENT\n");
}


/******************************************************************************/
/******************************************************************************/


//** Functions for gsl random numbers
void init_rng(gsl_rng** RNG)
{
	// init random numbers
	*RNG = gsl_rng_alloc (gsl_rng_taus);

	// reset random number generator
	struct timeval tv;
	gettimeofday(&tv, 0);
	srandom(tv.tv_sec ^ tv.tv_usec);
	rngSeed = random();
	gsl_rng_set (*RNG, rngSeed);
}

/******************************************************************************/
/******************************************************************************/

void del_rng(gsl_rng* RNG)
{
	// delete random number generator
	gsl_rng_free (RNG);
}

/******************************************************************************/
/******************************************************************************/

void set_seed_rng(gsl_rng* RNG, long int seed)
{
	gsl_rng_set (RNG, seed);
}

/******************************************************************************/
/******************************************************************************/

void GetGeneticParameters ( const char* param_file )
{
	ifstream pfile(param_file);
	if(!pfile) {
		cerr << "Can't find parameters file " << endl;
		exit(0);
	}

	/* Skip not genetic data */

	/* skip robots conf */
	double fNumRobots = getDouble('=',pfile);
	for ( int i = 0 ; i < fNumRobots ; i ++)
	{
		getDouble('=',pfile);
		getDouble('=',pfile);
		getDouble('=',pfile);
	}
	/* Skip write to file */
	getDouble('=',pfile);
	/* Skip RunTime */
	getDouble('=',pfile);

	/* Skip light objects */
	double fLightObjects = getDouble('=',pfile);
	for ( int i = 0 ; i < fLightObjects ; i++ )
	{
		getDouble('=',pfile);
		getDouble('=',pfile);
	}
	/* Skip blue light objects */
	double fBlueLightObjects = getDouble('=',pfile);
	for ( int i = 0 ; i < fBlueLightObjects ; i++ )
	{
		getDouble('=',pfile);
		getDouble('=',pfile);
	}
	
	/* Skip red light objects */
	double fRedLightObjects = getDouble('=',pfile);
	for ( int i = 0 ; i < fRedLightObjects ; i++ )
	{
		getDouble('=',pfile);
		getDouble('=',pfile);
	}

	/* Skip ground objects */
	double fGroundObjects = getDouble('=',pfile);
	for ( int i = 0 ; i < fGroundObjects ; i++)
	{
		getDouble('=',pfile);
		getDouble('=',pfile);
		getDouble('=',pfile);
		getDouble('=',pfile);
	}

	/* Skip sensors param */
	getDouble('=',pfile);
	getDouble('=',pfile);
	getDouble('=',pfile);
	getDouble('=',pfile);
	getDouble('=',pfile);
	getDouble('=',pfile);
	getDouble('=',pfile);
	getDouble('=',pfile);
	getDouble('=',pfile);
	getDouble('=',pfile);
	getDouble('=',pfile);
	getDouble('=',pfile);

	/* SKip morphology param */
	getDouble('=',pfile);
	getDouble('=',pfile);
	getDouble('=',pfile);
	getDouble('=',pfile);
	getDouble('=',pfile);
	getDouble('=',pfile);

	/* Get GENETIC info */
	g_unChromosomeLength 	= getInt('=', pfile);  						
	g_unChromosomes 			= getInt('=',pfile);       						
	g_unGenerations 			= getInt('=',pfile);									
	g_fEvaluationTime 		= getInt('=',pfile);    						
	g_bIsCrossoverOn 			= getInt('=',pfile);							
	g_nNumberOfCrossovers = getInt('=',pfile);						
	g_nCrossoverDistance  = getInt('=',pfile);						
	g_fMutationRate       = getDouble('=',pfile);						
	g_nNumberOfElites     = getInt('=',pfile);					
	g_unFitnessFunction		= getInt('=',pfile);
	g_unNumberOfSamplesPerChromosome = getInt('=',pfile);
	g_unRandomPositionOrientation = getInt('=',pfile); 
	// Skip Init Area -> Will be obtained by neuron exp
	getDouble('=',pfile); 
	getDouble('=',pfile); 
	
	/* Weights info from NEURAL */
	g_fUpperBounds 				= getDouble('=',pfile);
	g_fLowerBounds 				= getDouble('=',pfile);

	/* DEBUG */
	printf("CHROMOSOME LENGTH: %d\n",g_unChromosomeLength);
	printf("POPULATION SIZE: %d\n",g_unChromosomes);
	printf("NUMBER OF GENERATIONS %d\n",g_unGenerations);
	printf("EVALUATION TIME: %2f\n",g_fEvaluationTime);
	printf("CROSSOVER ON: %d\n",g_bIsCrossoverOn);
	printf("NUMBER OF CROSSOVERS: %d\n",g_nNumberOfCrossovers);
	printf("CROSSOVERDISTANCE: %d\n",g_nCrossoverDistance);
	printf("MUTATION RATE: %2f\n",g_fMutationRate);
	printf("NUMBER OF ELITES: %d\n",g_nNumberOfElites);
	printf("FITNESS FUNCTION: %d\n",g_unFitnessFunction);
	printf("UPPER BOUND: %2f\n",g_fUpperBounds);
	printf("LOWER BOUND: %2f\n",g_fLowerBounds);

	/* DEBUG */
}
