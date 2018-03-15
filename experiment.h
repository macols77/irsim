#ifndef EXPERIMENT_H_
#define EXPERIMENT_H_

/******************************************************************************/
/******************************************************************************/

#include "simulator.h"
#include "random.h"
#include "simmath.h"

//Collisions stuff:
#include "collisionepuck.h"
#include "collisionepucksimple.h"
#include "collisionepuckgripper.h"
#include "collisionpuck.h"
#include "collisionmanager.h"

// Sensors:
#include "epuckproximitysensor.h"
#include "proximitysensorpolynomial.h"
#include "groundsensor.h"
#include "compasssensor.h"
#include "encodersensor.h"
#include "randbsensor.h"
#include "contactsensor.h"
#include "proxsensor.h"

// Actuators:
#include "wheelsactuator.h"
#include "testwheel.h"

//Arenas:
#include "programmedarena.h"

//Managig files
#include "general.h"
/******************************************************************************/
/******************************************************************************/

enum EControllerType
{
	CONTROLLER_NONE,
	CONTROLLER_TEST_WHEELS,
	CONTROLLER_TEST_CONTACT,
	CONTROLLER_TEST_PROX,
	CONTROLLER_TEST_LIGHT,
	CONTROLLER_TEST_BLUE_LIGHT,
	CONTROLLER_BRAITENBERG_VEHICLE2,
	CONTROLLER_PERCEPTRON,
	CONTROLLER_IRI1,
	CONTROLLER_IRI2,
	CONTROLLER_IRI3,
	CONTROLLER_TEST_GROUND,
	CONTROLLER_TEST_BATTERY,
	CONTROLLER_NEURON,
	CONTROLLER_SUBSUMPTION_GARBAGE,
	CONTROLLER_SUBSUMPTION_LIGHT,
	CONTROLLER_TEST_COM,
	CONTROLLER_TEST_RED_LIGHT,
	CONTROLLER_TEST_ENCODER,
	CONTROLLER_TEST_COMPASS,
};

enum ECollisionModel
{
	COLLISION_MODEL_NONE,
	COLLISION_MODEL_SIMPLE,
	COLLISION_MODEL_DETAILED,
	COLLISION_MODEL_GRIPPER
};

enum ECollisionHandler
{
	COLLISION_HANDLER_NONE,
	COLLISION_HANDLER_RESET,
	COLLISION_HANDLER_POSITION//,
	//COLLISION_HANDLER_RUBBER
};

/******************************************************************************/
/******************************************************************************/

class CExperiment : public CSimObject
{
	public:
		CExperiment(const char* pch_name, ECollisionModel e_collisions_model = COLLISION_MODEL_NONE, ECollisionHandler e_collision_handler = COLLISION_HANDLER_NONE);
		//	CExperiment(const char* pch_name);

		virtual ~CExperiment();

		virtual void SetNumberOfEpucks(unsigned int un_number_of_epucks);
		virtual void SetNumberOfPucks(unsigned int un_number_of_pucks);
		virtual void SetControllerType(EControllerType e_controller_type);

		virtual void SetStartPositionInterval(double x1, double y1, 
				double x2, double y2);

		virtual void SetUseRandomStartRotation(bool b_yes_no);

		virtual void SetSensorList(char* pch_sensor_list);
		virtual void SetActuatorList(char* pch_actuator_list);

		//This method is the one called at the end of each evaluation in evolutionary experiments
		virtual void Reset();
		virtual void RandomPositionAndOrientation();
		//This one is called before the first experiment run, to setup parameters which have to be the same for all the individuals in
		//that generation (es: set a fixed starting point for the individuals and change it only for the next generation)
		virtual void InitializeGeneration();

		virtual CSimulator* CreateSimulator();
		virtual CSimulator* GetSimulator(); 

		void SetChainingParameters( double f_p_ec , double f_p_ce );

		virtual void SetChromosome(double* pf_chromosome, unsigned int un_chromosome_length);
		virtual void SetChromosomeLength( unsigned int un_chromosome_length);
		virtual void SetGeneration(unsigned int un_current_generation);
		virtual void SetSampleNumber(unsigned int un_sample_number);
		virtual void SetFitness(double f_fitness);
		virtual void SetNumberOfHiddenNodes(unsigned int un_number_of_hidden_nodes);

		static CExperiment* GetInstance();
		/*        
							virtual void SetExperimentArguments(CArguments* pc_experiment_arguments);
							virtual void SetControllerArguments(const char* pch_controller_arguments);

							virtual void SetSwarmBotSize(unsigned int un_x, unsigned int un_y);
							virtual void SetNumberOfSbots(unsigned int un_number_of_sbots);
							virtual void SetControllerType(EControllerType e_controller_type);
							virtual void SetNumberOfNetworkNodes(unsigned int un_number_of_network_nodes);
							virtual void SetNumberOfNetworksInNeuralArray(unsigned int un_number_of_networks);

							virtual void SetUseRandomChassisStartRotation(bool b_yes_no);
							void 	    CreateAndAddSBotsInDonut(unsigned int n_num_sbots,double f_donut_inner_radius,double f_donut_outer_radius);
							void 	    CreateAndAddSBotsInArena(unsigned int n_num_sbots, CSimulator* pc_simulator);
							void        CreateAndAddSBotsInRectangle(CSimulator* pc_simulator, unsigned int n_num_sbots, 
							double f_center_x, double f_center_y, 
							double f_size_x, double f_size_y);
							*/
	protected:
		virtual void SetSimulator(CSimulator* pc_simulator);

		virtual CEpuck* CreateEpuck(const char* pch_name, 
				double f_xpos, 
				double f_ypos, 
				double f_rotation);
		virtual CPuck* CreatePuck(const char* pch_name, 
				double f_xpos, 
				double f_ypos, 
				double f_rotation);

		virtual void    AddSensors(CEpuck* pc_epuck);
		virtual void    AddActuators(CEpuck* pc_epuck);
		virtual void    SetController(CEpuck* pc_epuck);
		virtual CArena* CreateArena();

		virtual void    CreateAndAddEpucks(CSimulator* pc_simulator);
		virtual void    CreateAndAddPucks(CSimulator* pc_simulator);
		virtual void    CreateCollisionManager();


		/*
			 virtual CSbot* CreateSbot(const char* pch_name, 
			 double f_xpos, 
			 double f_ypos, 
			 double f_rotation);
			 virtual void    AddSensors(CSbot* pc_sbot);
			 virtual void    AddActuators(CSbot* pc_sbot);
			 virtual void    SetController(CSbot* pc_sbot);
			 virtual void    SetCommonInterfaceController(CSbot* pc_sbot, class CCIController* pc_cicontroller);
			 virtual void    CreateAndAddSwarmBots(CSimulator* pc_simulator);
			 static CSbot* SbotFactory(const char* pch_name, double x, double y, 
			 double rotation);

			 CSwarmBot*  CreateNewSwarmBot(const char* pch_name, double f_x, double f_y, double f_rotation);
		// Creates and adds a single s-bot (and a corresponding swarm-bot) to a simulator.   
		//virtual CSbot*  CreateAndAddSbot(CSimulator* pc_simulator, const char* pch_name, double f_x, double f_y, double f_rotation);
		*/

	protected:
		CSimulator*         m_pcSimulator;
		EControllerType     m_eControllerType;

		static CExperiment* m_pcExperimentInstance;

		double              m_fStartPositionX1;
		double              m_fStartPositionY1;
		double              m_fStartPositionX2;
		double              m_fStartPositionY2;

		bool                m_bUseRandomStartRotation;

		char*               m_pchSensorList;
		char*               m_pchActuatorList;

		unsigned int        m_unNumberOfEpucks;
		unsigned int        m_unNumberOfPucks;

		ECollisionModel	m_eCollisionModel;
		ECollisionHandler	m_eCollisionHandler;

		CCollisionManager*  m_pcCollisionManager;

		double 		        m_fPsx;
		double 		        m_fPxs;

		//Genetic algorithms stuff
		double*             m_pfChromosome;
		unsigned int        m_unChromosomeLength;

		double              m_fFitness;
		unsigned int        m_unGeneration;
		unsigned int        m_unSampleNumber;

		//NN stuff
		unsigned int        m_unNumberOfHiddenNodes;

		/*
		//unsigned int        m_unSwarmBotSizeX;
		//unsigned int        m_unSwarmBotSizeY;

		unsigned int        m_unNumberOfNetworksInNeuralArray;
		unsigned int        m_unNumberOfNetworkNodes;
		bool                m_bUseRandomChassisRotation;

		unsigned int 	    m_unNumberOfSbots;
		CArguments*         m_pcExperimentArguments;
		char*               m_pchControllerArguments;
		*/
};

/******************************************************************************/
/******************************************************************************/

#endif
