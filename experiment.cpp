#include "experiment.h"

/******************************************************************************/
/******************************************************************************/

CExperiment* CExperiment::m_pcExperimentInstance = NULL;

/******************************************************************************/
/******************************************************************************/

CExperiment::CExperiment(const char* pch_name, ECollisionModel e_collisions_model, ECollisionHandler e_collision_handler) : 
	CSimObject(pch_name), m_pcSimulator(NULL), 
	m_eControllerType(CONTROLLER_NONE), 
	m_fStartPositionX1(0),
	m_fStartPositionY1(0),
	m_fStartPositionX2(0),
	m_fStartPositionY2(0),
	m_bUseRandomStartRotation(false),
	m_pchSensorList(NULL),
	m_pchActuatorList(NULL),
	m_eCollisionModel(e_collisions_model),
	m_eCollisionHandler(e_collision_handler),
	m_pcCollisionManager(NULL),
	m_pfChromosome(NULL), 
	m_unChromosomeLength(0),
	m_unGeneration(0), 
	m_unSampleNumber(0), 
	m_fFitness(0),
	m_unNumberOfEpucks(1),
	m_unNumberOfPucks(0)
{
	m_pcExperimentInstance = this;
	//CColoredWall* test=new CColoredWall("aaa");
}

/******************************************************************************/
/******************************************************************************/

/*
	 CExperiment::CExperiment(const char* pch_name):
	 CSimObject(pch_name)
	 {
	 m_pcExperimentInstance = this;
	 }
	 */

/******************************************************************************/
/******************************************************************************/

CExperiment::~CExperiment()
{
	if (m_pcSimulator)
		delete m_pcSimulator;

	if (m_pchSensorList)
		delete[] m_pchSensorList;

	if (m_pchActuatorList)
		delete[] m_pchActuatorList;

	if (m_pcCollisionManager)
		delete m_pcCollisionManager;
	/*    
				if (m_pcExperimentArguments)
				delete m_pcExperimentArguments;

				if (m_pchControllerArguments)
				delete[] m_pchControllerArguments;
				*/

}

/******************************************************************************/
/******************************************************************************/

CExperiment* CExperiment::GetInstance()
{
	return m_pcExperimentInstance;
}

/******************************************************************************/
/******************************************************************************/

CSimulator* CExperiment::GetSimulator()
{
	if (m_pcSimulator == NULL)
	{
		printf("Trying to get a non-existing simulator - create an environment first by calling\n"
				"CreateEnvironment(..)");        
	}
	return m_pcSimulator;
}

/******************************************************************************/
/******************************************************************************/

CSimulator* CExperiment::CreateSimulator()
{
	// Delete any previous simulator:
	if (m_pcSimulator)
		delete m_pcSimulator;

	// Delete any previous simulator:
	if (m_pcCollisionManager)
		delete m_pcCollisionManager;

	// Create a new simulator:
	SetSimulator(new CSimulator("Simulator"));

	// Add an arena:
	m_pcSimulator->SetArena(CreateArena());

	// Create collision manger:
	CreateCollisionManager();

	//Setting the model for collisions 
	m_pcSimulator->SetCollisionModelPresent(m_eCollisionModel);
	m_pcSimulator->SetCollisionHandler(m_eCollisionHandler);

	// Create a swarmbot:
	//CreateAndAddSwarmBots(m_pcSimulator);

	// Create epucks and pucks:
	CreateAndAddPucks(m_pcSimulator);
	CreateAndAddEpucks(m_pcSimulator);
	return m_pcSimulator;
}

/******************************************************************************/
/******************************************************************************/

void CExperiment::SetSimulator(CSimulator* pc_simulator)
{
	m_pcSimulator = pc_simulator;
}

/******************************************************************************/
/******************************************************************************/

CEpuck* CExperiment::CreateEpuck(const char* pch_name, 
		double f_xpos, 
		double f_ypos, 
		double f_rotation)
{
	CEpuck* pcEpuck;

	if (m_eCollisionModel == COLLISION_MODEL_NONE)
	{
		m_pcSimulator->SetCollisionModelPresent(0);
		pcEpuck = new CEpuck(pch_name, f_xpos, f_ypos, f_rotation);
	}

	else if (m_eCollisionModel == COLLISION_MODEL_SIMPLE)
	{
		m_pcSimulator->SetCollisionModelPresent(1);
		pcEpuck = new CCollisionEpuckSimple(pch_name, f_xpos, f_ypos, f_rotation);
	}

	else if (m_eCollisionModel == COLLISION_MODEL_GRIPPER)
	{
		m_pcSimulator->SetCollisionModelPresent(1);
		pcEpuck = new CCollisionEpuckGripper(pch_name, f_xpos, f_ypos, f_rotation);
	}
	/*
		 else if (m_eCollisionModel == COLLISION_MODEL_DETAILED)
		 {
		 pcEpuck = new CCollisionSbotDetailed(pch_name, f_xpos, f_ypos, f_rotation);
		 }
		 */
	else
	{
		printf("Invalid value for epuck collision model %d \n",m_eCollisionModel);
	}
	AddSensors(pcEpuck);
	AddActuators(pcEpuck);
	SetController(pcEpuck);

	return pcEpuck;
}

/******************************************************************************/
/******************************************************************************/

CPuck* CExperiment::CreatePuck(const char* pch_name, 
		double f_xpos, 
		double f_ypos, 
		double f_rotation)
{
	CPuck* pcPuck;

	if (m_eCollisionModel == COLLISION_MODEL_NONE)
	{
		m_pcSimulator->SetCollisionModelPresent(0);
		pcPuck = new CPuck(pch_name, f_xpos, f_ypos, f_rotation);
	}

	else if (m_eCollisionModel == COLLISION_MODEL_SIMPLE
			|| m_eCollisionModel == COLLISION_MODEL_GRIPPER)
	{
		m_pcSimulator->SetCollisionModelPresent(1);
		pcPuck = new CCollisionPuck(pch_name, f_xpos, f_ypos, f_rotation);
	}
	/*
		 else if (m_eCollisionModel == COLLISION_MODEL_DETAILED)
		 {
		 pcEpuck = new CCollisionSbotDetailed(pch_name, f_xpos, f_ypos, f_rotation);
		 }
		 */
	else
	{
		printf("Invalid value for puck collision model %d \n",m_eCollisionModel);
	}

	return pcPuck;
}

/******************************************************************************/
/******************************************************************************/

void CExperiment::AddSensors(CEpuck* pc_epuck)
{
	if (!m_pchSensorList)
		return;

	unsigned int nSensorListPosition = 0;
	unsigned int nSensorListLength = strlen(m_pchSensorList);

	char pchTemp[128];

	while (nSensorListPosition < nSensorListLength)
	{
		CSensor* pcSensor = NULL;
		switch (m_pchSensorList[nSensorListPosition])
		{
			case SENSOR_CHAR_PROXIMITY:
				pcSensor = new CEpuckProximitySensor(252);
				break;

			default:
				printf("Unknown sensor type: %c", m_pchSensorList[nSensorListPosition]);
				exit(-1);
		}

		nSensorListPosition++;

		// the selection string is comprised between two '+' signs. ex: +1,3,5,7+
		if (m_pchSensorList[nSensorListPosition] == '+')
		{
			char* pchTmpString       = strchr( m_pchSensorList + nSensorListPosition + 1, '+' );
			int   nSelectionLength   = pchTmpString - (m_pchSensorList + nSensorListPosition);
			char* pchSelectionString = new char[nSelectionLength];
			strncpy( pchSelectionString, m_pchSensorList+nSensorListPosition+1, nSelectionLength);
			pchSelectionString[nSelectionLength-1] = '\0';
			nSensorListPosition     += nSelectionLength+1;
		}

		pc_epuck->AddSensor(pcSensor);
	}
}

/******************************************************************************/
/******************************************************************************/

void CExperiment::AddActuators(CEpuck* pc_epuck)
{
    if (!m_pchActuatorList)
        return;
    
    unsigned int nActuatorListPosition = 0;
    unsigned int nActuatorListLength = strlen(m_pchActuatorList);

    char pchTemp[128];

    while (nActuatorListPosition < nActuatorListLength)
    {
        CActuator* pcActuator = NULL;
        switch (m_pchActuatorList[nActuatorListPosition])
        {
        case ACTUATOR_CHAR_WHEEL: 
            sprintf(pchTemp, "WheelsActuator_%s", pc_epuck->GetName());
            pcActuator = new CWheelsActuator(pchTemp, pc_epuck);
            break;
        default:
            printf("Unknown actuator type: %c", m_pchActuatorList[nActuatorListPosition]);
            exit(-1);
        }
        
        nActuatorListPosition++;
        pc_epuck->AddActuator(pcActuator);
    }
}

/******************************************************************************/
/******************************************************************************/

CArena* CExperiment::CreateArena()
{
    // Create the arena:
    CProgrammedArena* pcArena = new CProgrammedArena("CProgrammedArena", 3, 3, 6, 6);
    
    // Add an obstacle:
    pcArena->SetHeightPixel(2, 2, HEIGHT_OBSTACLE);
    
    return pcArena;
}

/******************************************************************************/
/******************************************************************************/

void CExperiment::SetController(CEpuck* pc_epuck)
{
    printf("\nController added to epuck in experiment\n");
    char pchTemp[128];

    // Find the appropriate controller and add it to the sbot:
    if (m_eControllerType == CONTROLLER_NONE)
    {
    } 
		//else if (m_eControllerType == CONTROLLER_OBSTACLE_AVOIDANCE)
		//{
		//sprintf(pchTemp, "ObstacleAvoidanceController");
		//CObstacleAvoidanceController* pcCObstacleAvoidanceController = new CObstacleAvoidanceController(pchTemp, pc_epuck);
		//pcCObstacleAvoidanceController->SetThreshold(5.0);
		//pc_epuck->SetController(pcCObstacleAvoidanceController);
		//}
    else 
    {
        printf("ERROR! This should never happen -- the controller type is invalid");
    }
}

/******************************************************************************/
/******************************************************************************/

void CExperiment::SetControllerType(EControllerType e_controller_type)
{
    m_eControllerType = e_controller_type;

}

/******************************************************************************/
/******************************************************************************/

void CExperiment::SetStartPositionInterval(double x1, double y1, 
                                           double x2, double y2)
{    
    m_fStartPositionX1 = x1;
    m_fStartPositionY1 = y1;
    m_fStartPositionX2 = x2;
    m_fStartPositionY2 = y2;
}

/******************************************************************************/
/******************************************************************************/

void CExperiment::SetUseRandomStartRotation(bool b_yes_no)
{
    m_bUseRandomStartRotation = b_yes_no;
}

/******************************************************************************/
/******************************************************************************/

void CExperiment::SetSensorList(char* pch_sensor_list)
{
    if (m_pchSensorList)
        delete m_pchSensorList;

    m_pchSensorList = (char*) malloc(sizeof(char) * strlen(pch_sensor_list) + 1);
    strcpy(m_pchSensorList, pch_sensor_list);
}

/******************************************************************************/
/******************************************************************************/

void CExperiment::SetActuatorList(char* pch_actuator_list)
{
    if (m_pchActuatorList)
        delete m_pchActuatorList;

    m_pchActuatorList = (char*) malloc(sizeof(char) * strlen(pch_actuator_list) + 1);
    strcpy(m_pchActuatorList, pch_actuator_list);
}

/******************************************************************************/
/******************************************************************************/

void CExperiment::CreateAndAddEpucks(CSimulator* pc_simulator)
{
    double fStartPositionX = m_fStartPositionX1;
    double fStartPositionY = m_fStartPositionY1;
    double fStartRotation = 0;
    
    // If the number of robots allows it, we create the first one
    if(m_unNumberOfEpucks > 0)
    {
      CEpuck* pcEpuck = CreateEpuck("epuck", fStartPositionX, fStartPositionY, fStartRotation);
      pc_simulator->AddEpuck(pcEpuck);      
    }
    
    // If the number of robots allows it, we create the other ones
    for(int i=1 ; i<m_unNumberOfEpucks ; i++)
    {
      double fChangeX, fChangeY, fChangeRotation=0;
      fChangeX = 5*CEpuck::CHASSIS_RADIUS;
      fChangeY = 5*CEpuck::CHASSIS_RADIUS;

      fStartPositionX += fChangeX;
      fStartPositionY += fChangeY;
      fStartRotation += fChangeRotation;
      
      CEpuck* pcEpuck = CreateEpuck("epuck", fStartPositionX, fStartPositionY, fStartRotation);
      pc_simulator->AddEpuck(pcEpuck);      
    }
}

/******************************************************************************/
/******************************************************************************/

void CExperiment::CreateAndAddPucks(CSimulator* pc_simulator)
{
    double fStartPositionX = m_fStartPositionX1 - 5*CPuck::CHASSIS_RADIUS;
    double fStartPositionY = m_fStartPositionY1 - 5*CPuck::CHASSIS_RADIUS;
    double fStartRotation = 0;
    
    // If the number of pucks allows it, we create the first one
    if(m_unNumberOfPucks > 0)
    {
      CPuck* pcPuck = CreatePuck("puck", fStartPositionX, fStartPositionY, fStartRotation);
      pc_simulator->AddPuck(pcPuck);      
    }
    
    // If the number of pucks allows it, we create the other ones
    for(int i=1 ; i<m_unNumberOfPucks ; i++)
    {
      double fChangeX, fChangeY, fChangeRotation=0;
      fChangeX = 5*CPuck::CHASSIS_RADIUS;
      fChangeY = 5*CPuck::CHASSIS_RADIUS;

      fStartPositionX -= fChangeX;
      fStartPositionY -= fChangeY;
      fStartRotation += fChangeRotation;
      
      CPuck* pcPuck = CreatePuck("puck", fStartPositionX, fStartPositionY, fStartRotation);
      pc_simulator->AddPuck(pcPuck);      
    }
}

/******************************************************************************/
/******************************************************************************/

void CExperiment::SetChainingParameters( double f_p_sx , double f_p_xs )
{
    m_fPsx = f_p_sx;
    m_fPxs = f_p_xs;
}

/******************************************************************************/
/******************************************************************************/

void CExperiment::CreateCollisionManager()
{
    if (m_eCollisionModel != COLLISION_MODEL_NONE)
        m_pcCollisionManager = new CCollisionManager("CollisionManager", CSimulator::GetInstance()->GetArena());
}

/******************************************************************************/
/******************************************************************************/

void CExperiment::SetNumberOfEpucks(unsigned int un_number_of_epucks)
{
    m_unNumberOfEpucks = un_number_of_epucks;
}

/******************************************************************************/
/******************************************************************************/

void CExperiment::SetNumberOfPucks(unsigned int un_number_of_pucks)
{
    m_unNumberOfPucks = un_number_of_pucks;
}

/******************************************************************************/
/******************************************************************************/

void CExperiment::SetChromosome(double* pf_chromosome,
                                unsigned int un_chromosome_length)
{
    m_pfChromosome       = pf_chromosome;
    m_unChromosomeLength = un_chromosome_length;
}

/******************************************************************************/
/******************************************************************************/

void CExperiment::SetChromosomeLength( unsigned int un_chromosome_length )
{
    m_unChromosomeLength = un_chromosome_length;
}

/******************************************************************************/
/******************************************************************************/
void CExperiment::SetGeneration(unsigned int un_current_generation)
{
    m_unGeneration = un_current_generation;
}

/******************************************************************************/
/******************************************************************************/

void CExperiment::SetSampleNumber(unsigned int un_sample_number)
{
    m_unSampleNumber = un_sample_number;
}

/******************************************************************************/
/******************************************************************************/

void CExperiment::SetFitness(double f_fitness)
{
    m_fFitness = f_fitness;
}

/******************************************************************************/
/******************************************************************************/

void CExperiment::SetNumberOfHiddenNodes(unsigned int un_number_of_hidden_nodes)
{
    m_unNumberOfHiddenNodes = un_number_of_hidden_nodes;
}

/******************************************************************************/
/******************************************************************************/

void CExperiment::Reset()
{
    //Does nothing, should be overridden in evolutionary experiments when needed
}

/******************************************************************************/
/******************************************************************************/

void CExperiment::RandomPositionAndOrientation()
{
    //Does nothing, should be overridden in evolutionary experiments when needed
}

/******************************************************************************/
/******************************************************************************/
void CExperiment::InitializeGeneration(){
	//Does nothing, should be overridden in evolutionary experiments when needed
}

/******************************************************************************/
/******************************************************************************/



