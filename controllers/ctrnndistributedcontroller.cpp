#include "ctrnndistributedcontroller.h"


/******************************************************************************/
/******************************************************************************/

CCTRNNDistributedController::CCTRNNDistributedController(const char* pch_name, CEpuck* pc_epuck, 
																									 unsigned int un_number_of_layers, 
																									 unsigned int* un_layers_outputs,	
																									 unsigned int* un_layer_sensor_type,
																									 unsigned int* un_activation_function,
																									 unsigned int** un_adjacency_matrix,
																									 unsigned int* un_learning_layer_flag,
																									 unsigned int* un_evo_devo_layer_flag,
																									 unsigned int* un_learning_diagonal_flag,
																									 double f_lower_bounds, 
																									 double f_upper_bounds,
																									 bool b_evolutionary_flag, 
																									 bool b_learning_flag, 
																									 double f_eta, double f_epsilon, 
																									 int n_write_to_file, 
																									 unsigned int un_proximity_mumber, 
																									 unsigned int* un_proximity_value, 
																									 unsigned int un_contact_number, 
																									 unsigned int* un_contact_value, 
																									 unsigned int un_light_number, 
																									 unsigned int* un_light_value, 
																									 unsigned int un_ground_number, 
																									 unsigned int* un_ground_value,
																									 unsigned int un_blue_light_number, 
																									 unsigned int* un_blue_light_value,
																									 unsigned int un_red_light_number, 
																									 unsigned int* un_red_light_value) :
    CController(pch_name, pc_epuck),
    m_pfInputs(NULL)
{

	m_pcEpuck = pc_epuck;
	m_fTime = 0;

	m_nWriteToFile = n_write_to_file;
	m_unNumberOfLayers = un_number_of_layers;
	m_unNumberOfLayerOutputs = un_layers_outputs;
	m_unLayerSensorType = un_layer_sensor_type;
	m_mAdjacencyMatrix = un_adjacency_matrix;
	m_unActivationFunction = un_activation_function;
	m_unLearningLayerFlag = un_learning_layer_flag;
	m_unLearningDiagonalFlag = un_learning_diagonal_flag;
	m_unEvoDevoLayerFlag = un_evo_devo_layer_flag;
	m_fLowerBounds = f_lower_bounds;
	m_fUpperBounds = f_upper_bounds;

	m_bEvolutionaryFlag = b_evolutionary_flag;
	m_bLearningFlag = b_learning_flag;

	m_fEta = f_eta;
	m_fEpsilon = f_epsilon;

	/* Morphology */
	m_unProximitySensorsUsedNumber = un_proximity_mumber;
	m_unProximitySensorsUsedValue = un_proximity_value;
	m_unContactSensorsUsedNumber = un_contact_number;
	m_unContactSensorsUsedValue = un_contact_value;
	m_unLightSensorsUsedNumber = un_light_number;
	m_unLightSensorsUsedValue = un_light_value;
	m_unGroundSensorsUsedNumber = un_ground_number;
	m_unGroundSensorsUsedValue = un_ground_value;
	m_unBlueLightSensorsUsedNumber = un_blue_light_number;
	m_unBlueLightSensorsUsedValue = un_blue_light_value;
	m_unRedLightSensorsUsedNumber = un_red_light_number;
	m_unRedLightSensorsUsedValue = un_red_light_value;


	/* DEBUG AGUTI */
	m_unDebugCollisions 		= 0;
	m_fDebugMaxLinearSpeed 	= 0.0;
	m_fDebugMaxDist 				= 0.0;
	m_vDebugOldPos.x 				= 0.0;
	m_vDebugOldPos.y 				= 0.0;
	/* END DEBUG AGUTI */

	//unsigned int unMaximumNumberOutput = 0;
	//unsigned int unMaximumNumberInput = 0;

	m_unNumberOfLayerInputs = new unsigned int[m_unNumberOfLayers];
	m_unNumberOfSensorInputs = new unsigned int[m_unNumberOfLayers];

	for ( int i = 0 ; i < m_unNumberOfLayers ; i++ )
	{
		m_unNumberOfSensorInputs[i] = 0;
		TSensorVector vecSensors = pc_epuck->GetSensors();
		for (TSensorIterator j = vecSensors.begin(); j != vecSensors.end(); j++)
		{
			if ( (*j)->GetType() == m_unLayerSensorType[i])
			{
				//m_unNumberOfSensorInputs[i] = (*j)->GetNumberOfInputs();
				/* Put only the used sensors */
				switch( (*j)->GetType())
				{
					case SENSOR_CONTACT:
						m_unNumberOfSensorInputs[i] = m_unContactSensorsUsedNumber;
						break;
					case SENSOR_PROXIMITY:
						m_unNumberOfSensorInputs[i] = m_unProximitySensorsUsedNumber;
						break;
					case SENSOR_REAL_LIGHT:
						m_unNumberOfSensorInputs[i] = m_unLightSensorsUsedNumber;
						break;
					case SENSOR_REAL_BLUE_LIGHT:
						m_unNumberOfSensorInputs[i] = m_unBlueLightSensorsUsedNumber;
						break;
					case SENSOR_REAL_RED_LIGHT:
						m_unNumberOfSensorInputs[i] = m_unRedLightSensorsUsedNumber;
						break;
					case SENSOR_BATTERY:
						m_unNumberOfSensorInputs[i] = (*j)->GetNumberOfInputs();
						break;
					case SENSOR_BLUE_BATTERY:
						m_unNumberOfSensorInputs[i] = (*j)->GetNumberOfInputs();
						break;
					case SENSOR_RED_BATTERY:
						m_unNumberOfSensorInputs[i] = (*j)->GetNumberOfInputs();
						break;
					case SENSOR_GROUND_MEMORY:
						m_unNumberOfSensorInputs[i] = (*j)->GetNumberOfInputs();
						break;
					case SENSOR_GROUND:
						m_unNumberOfSensorInputs[i] = m_unGroundSensorsUsedNumber;
						break;
				}
			}
		}       

		m_unNumberOfLayerInputs[i] = 0;
		for ( int j = 0 ; j < m_unNumberOfLayers ; j++ )
		{
			if(m_mAdjacencyMatrix[j][i] == 1)
			{
				m_unNumberOfLayerInputs[i] += m_unNumberOfLayerOutputs[j];
			}
		}

		/* DEBUG */
		//printf("%d %d %d %d %d\n",m_unNumberOfSensorInputs[i], m_unNumberOfLayerInputs[i], m_unNumberOfLayerOutputs[i],SIGMOID_ACTIVATION, i); 
		/* DEBUG */

		CCTRNNLayerController* pcLayer;
		pcLayer = new CCTRNNLayerController ( 	"layer", pc_epuck,
																			m_unNumberOfSensorInputs[i], 
																			m_unNumberOfLayerInputs[i], 
																			m_unNumberOfLayerOutputs[i], 
																			m_unActivationFunction[i], 
																			i, 
																			m_fLowerBounds,
																			m_fUpperBounds);
		m_vecLayers.push_back(pcLayer);

		/* Get Maximum Output Layers */
		//if ( m_unNumberOfLayerOutputs[i] > unMaximumNumberOutput )
		//unMaximumNumberOutput = m_unNumberOfLayerOutputs[i];

		/* Get Maximum Input Layers */
		//if ( m_unNumberOfLayerInputs[i] > unMaximumNumberInput )
		//unMaximumNumberInput = m_unNumberOfLayerInputs[i];
	}

	m_fOutputMatrix = new double*[m_unNumberOfLayers];
	m_fOutputMatrixOld = new double*[m_unNumberOfLayers];
	m_pfWeightMatrix = new double* [m_unNumberOfLayers];
	m_unNumberOfParameters = new unsigned int [m_unNumberOfLayers];

	for ( int i = 0 ; i < m_unNumberOfLayers ; i++ )
	{
		/* The outputs is as big as the NumberOfNeurons */
		m_fOutputMatrix[i] = new double[m_unNumberOfLayerOutputs[i]];
		m_fOutputMatrixOld[i] = new double[m_unNumberOfLayerOutputs[i]];

		/* For each Neuron in each Leayer */
		for ( int j = 0 ; j < m_unNumberOfLayerOutputs[i] ; j++ )
		{
			/* Init to 0.0 */
			m_fOutputMatrix[i][j] = 0.0;
			m_fOutputMatrixOld[i][j] = 0.0;
		}

		/* If it is the IDENTITY function, there is no weights */
		if ( m_unActivationFunction[i] == IDENTITY_ACTIVATION)
			m_unNumberOfParameters[i] = 0;
		/* If NOT IDENTITY */
		else
    {
      /* Parameters is the Number of NEURONS * ( Number of Layer INPUTS + BIAS + TAU + BETA ) + SensorInputs
       *Number of Layer INPUTS includes recurrency (SELF) */
      m_unNumberOfParameters[i] = (m_unNumberOfLayerInputs[i] + 3 ) * m_unNumberOfLayerOutputs[i] + m_unNumberOfSensorInputs[i];
    }
		
		/* Each layer weight matrix is as big as the number of paramenters */
		m_pfWeightMatrix[i] = new double[m_unNumberOfParameters[i]];
	}

	
	/* Create semi-random chromosome file */
	/* This is only used if the experiment has been executed with -l */
	if (m_bLearningFlag == true )
	{
		FILE* fileLearning = fopen("learningFiles/initFile","a");
		//printf( " LEARNING\n");
		
		for ( int i = 0 ; i < m_unNumberOfLayers ; i++)
		{
			//printf("LAYER: %d: %d %d\n", i, m_unNumberOfLayerInputs[i], m_unNumberOfLayerOutputs[i]);
			int nVar = m_unNumberOfLayerInputs[i] / m_unNumberOfLayerOutputs[i];
			
			if ( m_unActivationFunction[i] != IDENTITY_ACTIVATION )
			{
				for ( int j = 0 ; j < m_unNumberOfLayerOutputs[i] ; j++ )
				{
					/* print bias */
					if ( m_unLearningLayerFlag[i] != 0 )
						fprintf(fileLearning, "%2f ", 1.5 );
					else
						fprintf(fileLearning, "%2f ", 0.8 );
					
					for ( int k = 0 ; k < m_unNumberOfLayerInputs[i] ; k++ )
					{
						if ( m_unLearningLayerFlag[i] != 0 )
						{
							fprintf(fileLearning,"%2f ", Random::nextDouble());
						}
						else
						{
							if ( ( k >= ( nVar * j ) ) && ( k < (nVar * ( j + 1 ) ) ))
							{
								fprintf(fileLearning,"%2f ", 0.0);
							}
							else 
							{
								fprintf(fileLearning, "%2f ", 0.0);
							}
						}	
					}
				}
			}
		}
		
		fclose(fileLearning);

		exit(0);
	}
}

/******************************************************************************/
/******************************************************************************/

CCTRNNDistributedController::~CCTRNNDistributedController()
{
	if (m_pfInputs)
		free(m_pfInputs);

	delete [] m_unNumberOfLayerInputs;
	delete [] m_unNumberOfSensorInputs;
	delete [] m_unNumberOfLayerOutputs;
	delete [] m_unLayerSensorType;
	delete [] m_unActivationFunction;
	delete [] m_unNumberOfParameters;
	delete [] m_unLearningLayerFlag;
	delete [] m_unLearningDiagonalFlag;
	delete [] m_unEvoDevoLayerFlag;
	
	for ( int i = 0 ; i < m_unNumberOfLayers ; i++ )
	{
		delete [] m_mAdjacencyMatrix[i];
		delete [] m_fOutputMatrix[i];
		delete [] m_fOutputMatrixOld[i];
		delete [] m_pfWeightMatrix[i];
	} 
	
	/* Morphology */
	delete [] m_unProximitySensorsUsedValue;
	delete [] m_unContactSensorsUsedValue;
	delete [] m_unLightSensorsUsedValue;
	delete [] m_unBlueLightSensorsUsedValue;
	delete [] m_unRedLightSensorsUsedValue;
	delete [] m_unGroundSensorsUsedValue;

		
}


/******************************************************************************/
/******************************************************************************/

void CCTRNNDistributedController::SimulationStep(unsigned n_step_number, double f_time, double f_step_interval)
{
  m_fTime = f_time;
  /* Set Leds to BLACK */
  m_pcEpuck->SetAllColoredLeds(	LED_COLOR_BLACK);

  /* For every Layer Do */
  unsigned int k = 0;
  for (TLayerIterator i = m_vecLayers.begin() ; i != m_vecLayers.end(); i++, k++)
  {
    double* pfSensorInputs = NULL;
    /* Check which sensor is connected to each layer */
    TSensorVector vecSensors = GetEpuck()->GetSensors();
    for (TSensorIterator j = vecSensors.begin(); j != vecSensors.end(); j++)
    {
      /* If this is my sensor */
      if ( (*j)->GetType() == m_unLayerSensorType[k])
      {
        /* Get Readings */
        double *pfTotalSensorInputs = NULL;
        pfTotalSensorInputs = (*j)->GetComputedSensorReadings();

        /* Put only the used sensors */
        int realIndex = 0;
        switch( (*j)->GetType())
        {
          case SENSOR_CONTACT:
            pfSensorInputs = new double[m_unContactSensorsUsedNumber];
            for ( int i = 0 ; i < (*j)->GetNumberOfInputs() ; i++)
            {
              if ( m_unContactSensorsUsedValue[i] == 1 )
              {
                pfSensorInputs[realIndex] = pfTotalSensorInputs[i]; 
                realIndex++;
              }
            }

            break;

          case SENSOR_PROXIMITY:
            pfSensorInputs = new double[m_unProximitySensorsUsedNumber];
            for ( int i = 0 ; i < (*j)->GetNumberOfInputs() ; i++)
            {
              if ( m_unProximitySensorsUsedValue[i] == 1 )
              {
                pfSensorInputs[realIndex] = pfTotalSensorInputs[i]; 
                realIndex++;
              }
            }
            break;

          case SENSOR_REAL_LIGHT:
            pfSensorInputs = new double[m_unLightSensorsUsedNumber];
            for ( int i = 0 ; i < (*j)->GetNumberOfInputs() ; i++)
            {
              if ( m_unLightSensorsUsedValue[i] == 1 )
              {
                pfSensorInputs[realIndex] = pfTotalSensorInputs[i]; 
                realIndex++;
              }
            }
            break;

          case SENSOR_REAL_BLUE_LIGHT:
            pfSensorInputs = new double[m_unBlueLightSensorsUsedNumber];
            for ( int i = 0 ; i < (*j)->GetNumberOfInputs() ; i++)
            {
              if ( m_unBlueLightSensorsUsedValue[i] == 1 )
              {
                pfSensorInputs[realIndex] = pfTotalSensorInputs[i]; 
                realIndex++;
              }
            }
            break;

          case SENSOR_REAL_RED_LIGHT:
            pfSensorInputs = new double[m_unRedLightSensorsUsedNumber];
            for ( int i = 0 ; i < (*j)->GetNumberOfInputs() ; i++)
            {
              if ( m_unRedLightSensorsUsedValue[i] == 1 )
              {
                pfSensorInputs[realIndex] = pfTotalSensorInputs[i]; 
                realIndex++;
              }
            }
            break;

          case SENSOR_BATTERY:
            pfSensorInputs = new double[1];
            pfSensorInputs[0] = pfTotalSensorInputs[0]; 
            break;
          case SENSOR_BLUE_BATTERY:
            pfSensorInputs = new double[1];
            pfSensorInputs[0] = pfTotalSensorInputs[0]; 
            break;
          case SENSOR_RED_BATTERY:
            pfSensorInputs = new double[1];
            pfSensorInputs[0] = pfTotalSensorInputs[0]; 
            break;
          case SENSOR_GROUND_MEMORY:
            pfSensorInputs = new double[1];
            pfSensorInputs[0] = pfTotalSensorInputs[0]; 
            break;
          case SENSOR_GROUND:
            pfSensorInputs = new double[m_unGroundSensorsUsedNumber];
            for ( int i = 0 ; i < (*j)->GetNumberOfInputs() ; i++)
            {
              if ( m_unGroundSensorsUsedValue[i] == 1 )
              {
                pfSensorInputs[realIndex] = pfTotalSensorInputs[i]; 
                realIndex++;
              }
            }
            break;
        }
      }
    }

    double* pfLayerInputs = NULL;

    if ( m_unNumberOfLayerInputs[k] == 0 )
    {
      pfLayerInputs = NULL;
    }
    else
    {
      /* Take memory for the layers inputs */
      pfLayerInputs = new double[m_unNumberOfLayerInputs[k]];
      /* Get the values of each layer that enters in the next layer */
      int nIndex = 0;
      for ( int j = 0 ; j < m_unNumberOfLayers ; j++ )
      {
        if ( m_mAdjacencyMatrix[j][k] == 1 )
        {
          for ( int m = 0 ; m < m_unNumberOfLayerOutputs[j] ; m++ , nIndex++ )
          {
            //pfLayerInputs[nIndex] = m_fOutputMatrix[j][m];
            pfLayerInputs[nIndex] = m_fOutputMatrixOld[j][m];
          }
        }
      }
    }
    /* DEBUG */
    if ( k == 0)
    {
      if ( !m_bEvolutionaryFlag)
      {
        printf("---------------\n");
        printf("---------------\n");
        printf("COLLISIONS: %2d\n",CCollisionManager::GetInstance()->GetTotalNumberOfCollisions());
        printf("---------------\n");
        printf("---------------\n");
      }

      int nContact = 0;
      CContactSensor *m_seContact = (CContactSensor*) m_pcEpuck->GetSensor(SENSOR_CONTACT);
      double* contact = m_seContact->GetSensorReading(m_pcEpuck);
      for ( int j = 0 ; j < m_seContact->GetNumberOfInputs() ; j++)
      {
        if(contact[j] > 0.0) 
          nContact=1;
      }

      if (nContact == 1 )
      {
        /* Set Leds to GREEN */
        m_pcEpuck->SetAllColoredLeds(	LED_COLOR_GREEN);
        if ( !m_bEvolutionaryFlag && m_nWriteToFile)
        {
          FILE* fileContact = fopen("outputFiles/robotContacts", "a");
          fprintf(fileContact,"%2.4f\n", m_fTime);
          fclose(fileContact);
          //printf("%2.4f CONTACT\n",f_time);	
        }
      }
    }
    /* DEBUG */

    /* DEBUG */
    if (!m_bEvolutionaryFlag)
    {
      printf("LAYER: %d\n",k);
      printf("---------------\n");
      printf("SENSORS %d: ",m_unNumberOfSensorInputs[k]);

      for ( int j = 0 ; j < m_unNumberOfSensorInputs[k] ; j++)
      {
        printf(" %2f",pfSensorInputs[j]);
      }

      printf("\n");

      //unsigned int unNumberOfWeights = ( m_unNumberOfLayerInputs[k] + 1 ) * m_unNumberOfLayerOutputs[k];
      printf("WEIGHTS %d: ",m_unNumberOfParameters[k]);
      for ( int j = 0 ; j < m_unNumberOfParameters[k] ; j++ )
      {
        printf(" %2f",m_pfWeightMatrix[k][j]);
      }
      printf("\n");
    }
    /* DEBUG */



    /* Execute Layer */
    m_fOutputMatrix[k] = (*i)->ComputeOutputs(pfLayerInputs, pfSensorInputs, m_pfWeightMatrix[k]);
    if ( !m_bEvolutionaryFlag)
    {
      if ( m_unLearningLayerFlag[k] != 0 )
      {
        LearningFunction(pfLayerInputs, pfSensorInputs, k);
      }
    }
    else
    {
      if ( m_unEvoDevoLayerFlag[k] != 0)
      {
        LearningFunction(pfLayerInputs, pfSensorInputs, k);
      }
    }

    /* DEBUG */
    if (!m_bEvolutionaryFlag)
    {
      printf("OUTPUT %d: ",m_unNumberOfLayerOutputs[k]);
      for ( int j = 0 ; j < m_unNumberOfLayerOutputs[k] ; j++)
      {
        printf(" %2f",m_fOutputMatrix[k][j]);
      }
      printf("\n");
      printf("---------------\n");
      printf("---------------\n");
    }
    /* DEBUG */


    delete [] pfLayerInputs;

    if ( m_nWriteToFile ) 
    {
      /* INIT: WRITING TO FILES */
      if (!m_bEvolutionaryFlag){
        /*Write a file for each layer with the input, weights and output */
        char fileName[100];
        sprintf(fileName, "outputFiles/layer%dOutput", k);
        FILE* fileOutput = fopen(fileName, "a");

        /* Print TIME */
        fprintf(fileOutput,"%2.4f ", m_fTime);
        /* Print INPUTS */
        for ( int j = 0 ; j < m_unNumberOfSensorInputs[k] ; j++)
        {
          fprintf(fileOutput,"%2.4f ",pfSensorInputs[j]);
        }
        /* Print WEIGHTS */
        for ( int j = 0 ; j < m_unNumberOfParameters[k] ; j++ )
        {
          fprintf(fileOutput,"%2.4f ",m_pfWeightMatrix[k][j]);
        }
        /* Print OUTPUTS */
        for ( int j = 0 ; j < m_unNumberOfLayerOutputs[k] ; j++)
        {
          fprintf(fileOutput,"%2.4f ",m_fOutputMatrix[k][j]);
        }
        fprintf(fileOutput,"\n");
        fclose(fileOutput);
      }
      /* INIT: WRITING TO FILES */
    }

    delete [] pfSensorInputs;
  }

  /* Once all layers took the oldOutput value, Update new to old outputMatrix */
  for ( int i = 0 ; i < m_unNumberOfLayers ; i++ )
    for ( int j = 0 ; j < m_unNumberOfLayerOutputs[i] ; j++ )
      m_fOutputMatrixOld[i][j] = m_fOutputMatrix[i][j];

  // Apply the outputs to all the actuators:
  TActuatorVector vecActuators = GetEpuck()->GetActuators();
  for (TActuatorIterator k = vecActuators.begin(); k != vecActuators.end(); k++)
  {
    unsigned int unThisActuatorNumberOfOutputs = (*k)->GetNumberOfOutputs();

    for (int l = 0; l < unThisActuatorNumberOfOutputs; l++)
    {
      if ( m_fOutputMatrix[(m_unNumberOfLayers -1 )][l] > 1.0 ) 
        m_fOutputMatrix[(m_unNumberOfLayers -1 )][l] = 1.0;
      if ( m_fOutputMatrix[(m_unNumberOfLayers -1 )][l] < 0.0)
        m_fOutputMatrix[(m_unNumberOfLayers -1 )][l] = 0.0;
      (*k)->SetOutput(l, m_fOutputMatrix[(m_unNumberOfLayers -1 )][l]);
    }              
  }

  /* DEBUG */
  if (!m_bEvolutionaryFlag)
  {
    printf("MOTORS: ");
    printf("%2.4f %2.4f \n", m_fOutputMatrix[(m_unNumberOfLayers -1 )][0], m_fOutputMatrix[(m_unNumberOfLayers -1 )][1]);
    printf("\n");
    printf("---------------\n");
    printf("---------------\n");
  }
  /* DEBUG */

  if ( m_nWriteToFile && !m_bEvolutionaryFlag) 
  {
    /* INIT: WRITE TO FILES */
    /* Write robot position and orientation */
    FILE* filePosition = fopen("outputFiles/robotPosition", "a");
    fprintf(filePosition,"%2.4f %2.4f %2.4f %2.4f\n", m_fTime, m_pcEpuck->GetPosition().x, m_pcEpuck->GetPosition().y, m_pcEpuck->GetRotation());
    fclose(filePosition);

    //printf("%2.4f %2.4f %2.4f %2.4f\n", m_fTime, m_pcEpuck->GetPosition().x, m_pcEpuck->GetPosition().y, m_pcEpuck->GetRotation());

    /* Write robot wheels speed */
    FILE* fileWheels = fopen("outputFiles/robotWheels", "a");
    fprintf(fileWheels,"%2.4f %2.4f %2.4f \n", m_fTime, m_fOutputMatrix[(m_unNumberOfLayers -1 )][0], m_fOutputMatrix[(m_unNumberOfLayers -1 )][1]);
    fclose(fileWheels);
    /* END WRITE TO FILES */
  }

  /* DEBUG COMPARISON AGUTI */
  if (!m_bEvolutionaryFlag)
  {
    /* Calc Collisions */
    m_unDebugCollisions = CCollisionManager::GetInstance()->GetTotalNumberOfCollisions();	

    /* Calc Max Linear Speed */
    double f_tempLinearSpeed = ( ( (2000 * m_fOutputMatrix[(m_unNumberOfLayers -1 )][0] ) - 1000 ) + ( (2000 * m_fOutputMatrix[(m_unNumberOfLayers -1 )][1] ) - 1000 ) ) / 2.0;
    if ( f_tempLinearSpeed >= m_fDebugMaxLinearSpeed )
      m_fDebugMaxLinearSpeed = f_tempLinearSpeed;

    /* Calc Max Distance */
    double f_deltaDist = sqrt (pow ( ( m_pcEpuck->GetPosition().x - m_vDebugOldPos.x ), 2 ) + pow ( ( m_pcEpuck->GetPosition().y - m_vDebugOldPos.y ) , 2) );
    m_fDebugMaxDist += f_deltaDist;
    m_vDebugOldPos.x = m_pcEpuck->GetPosition().x;
    m_vDebugOldPos.y = m_pcEpuck->GetPosition().y;

    printf("---------------\n");
    printf("---------------\n");
    printf ( "MAX SPEED: %2f -- MAX DIST: %2f -- COLL: %2d\n", m_fDebugMaxLinearSpeed, m_fDebugMaxDist, m_unDebugCollisions); 
    printf("---------------\n");
    printf("---------------\n");
  }
  /* END DEBUG COMPARISON AGUTI */

}

/******************************************************************************/
/******************************************************************************/

unsigned int CCTRNNDistributedController::GetNumberOfSensorInputs()
{
	unsigned int systemSensorInputs = 0;
	for ( int i = 0 ; i < m_unNumberOfLayers ; i++)
	{
		systemSensorInputs += m_unNumberOfSensorInputs[i];
	}
	
    return systemSensorInputs;
}

/******************************************************************************/
/******************************************************************************/

unsigned int CCTRNNDistributedController::GetNumberOfActuatorOutputs()
{
    return m_unNumberOfActuatorOutputs;
}

/******************************************************************************/
/******************************************************************************/

double* CCTRNNDistributedController::LoadWeights(const char* pch_filename)
{
	printf("Controller: %s, loading weights from: %s\n",GetName(),pch_filename);
	int nErrorCode;
	ifstream in;
	in.open( pch_filename, ios::in );
	if( !in ) {
		printf("Cannot open file containing neural network weights: %s", pch_filename);
		fflush(stdout);
	}

	int length = 0;
	if( !(in >> length) ) {
		printf("Cannot read file containing neural network weights: %s\n", pch_filename);
		fflush(stdout);
	}

	double* weights = new double[length];

	for( int i = 0; i < length; i++ ) {
		if( !(in >> weights[i] ) ) {
			printf("Cannot read weight %d from file: %s.\n", i, pch_filename);
			fflush(stdout);
		}
	}

	return weights;
	//delete weights;
}

/******************************************************************************/
/******************************************************************************/

void CCTRNNDistributedController::SaveState(const char* pch_filename){

}

/******************************************************************************/
/******************************************************************************/

void CCTRNNDistributedController::LoadState(const char* pch_filename){
	
}
/******************************************************************************/
/******************************************************************************/
void CCTRNNDistributedController::SetWeights(unsigned int un_number_of_weights, double* pf_weights) 
{
	//if (un_number_of_weights != m_unRequiredNumberOfWeights)
	//{
		//printf("CCTRNNLayerController::SetWeights ERROR. Numero incorrecto de pesos\n");   
		//exit(1);                       
	//}      

	// Set the weights
	//memcpy(m_pfWeights, pf_weights, m_unRequiredNumberOfWeights * sizeof(double));
	//

	/* DEBUG */
	//printf("PESOS LEIDOS: ");
	//for ( int i = 0 ; i < 26 ; i++)
	//{
		//printf("%2f ",pf_weights[i]);
	//}
	//printf("\n");
	/* DEBUG */
	
	//ofstream file("weightsFile");
	int nIndex = 0;
	for ( int i = 0 ; i < m_unNumberOfLayers ; i++)
	{
		//file << i << " " << m_unNumberOfLayerInputs[i] << " " << m_unNumberOfLayerOutputs[i] << endl;
		//unsigned int unNumberOfWeights = ( m_unNumberOfLayerInputs[i] + 1 ) * m_unNumberOfLayerOutputs[i];
		//for ( int j = 0 ; j < unNumberOfWeights ; j++ , nIndex++)
		for ( int j = 0 ; j < m_unNumberOfParameters[i] ; j++ , nIndex++)
		{
			m_pfWeightMatrix[i][j] = pf_weights[nIndex];
			//m_pfWeightMatrix[i][j] = 0.0;
			//m_pfWeightMatrix[i][j] = pf_weights[nIndex] * (m_fUpperBounds - m_fLowerBounds) + m_fLowerBounds;
			//printf("SIZE: %d\n",sizeof(pf_weights));

		}
	}


	//file << nIndex << " " ;
	
	//for ( int i = 0 ; i < nIndex ; i++ )
	//{
		//file << 0.5 << " ";
	//}
	//file << endl;


		//printf("\n");
		//printf("\n");
	//for ( int i = 0 ; i < m_unNumberOfLayers ; i++)
	//{
		//for ( int j = 0 ; j < m_unNumberOfLayers ; j++ )
		//{
			//printf (" %d",m_mAdjacencyMatrix[i][j]);
		//}
		//printf("\n");
	//}
	
	//Scale the weights in given range                      
	//if(!((m_fUpperBounds==1.0)&&(m_fLowerBounds==0.0))){
	//for(int i=0;i<un_number_of_weights;i++){                 
	//m_pfWeights[i]=m_pfWeights[i]*(m_fUpperBounds-m_fLowerBounds)+m_fLowerBounds;
	//}
	//} 
}

/******************************************************************************/
/******************************************************************************/

void CCTRNNDistributedController::LearningFunction ( double* f_layer_inputs, double* f_sensor_inputs, unsigned int un_layer)
{


	//for ( int i = 0 ; i < m_unNumberOfParameters[un_layer] ; i++)
	//{
	//if ( i % ( m_unNumberOfLayersInputs[un_layer] + 1) != 0 )
	//{
	//m_pfWeightMatrix[un_layer][i] += fNu * m_fOutputMatrix[un_layer][i] * f_inputs[i] 
	//}
	//}
	//printf("---------------------ETA: %2f, EP: %2f\n", m_fEta, m_fEpsilon);
	
	double fOutputAverage = 0.0;
	for ( int i= 0 ; i < m_unNumberOfLayerOutputs[un_layer] ; i++)
	{
		fOutputAverage += m_fOutputMatrix[un_layer][i];
	}
		fOutputAverage /= m_unNumberOfLayerOutputs[un_layer];
 
	int nIndexWeights = 0;
	for ( int i= 0 ; i < m_unNumberOfLayerOutputs[un_layer] ; i++)
	{
		/* We jup the bias */
		nIndexWeights++;
		for ( int j = 0 ; j < m_unNumberOfLayerInputs[un_layer] ; j++)
		{

			/* If sensorial learning */
			if ( m_unLearningLayerFlag[un_layer] == 1 )
			{
				/* If in the diagonal, make for any leearning */
				if ( i == j )
				{
					//m_pfWeightMatrix[un_layer][nIndexWeights] += m_fEta * m_fOutputMatrix[un_layer][j] * f_sensor_inputs[i] * f_layer_inputs[i]
					m_pfWeightMatrix[un_layer][nIndexWeights] += m_fEta * m_fOutputMatrix[un_layer][i] * f_sensor_inputs[i]; 
				}
				/* If not in the diagonal */
				else {
					/* Check for DiagonalFlag */
					if ( m_unLearningDiagonalFlag[un_layer] == 0 )
					{
						//m_pfWeightMatrix[un_layer][nIndexWeights] += m_fEta * m_fOutputMatrix[un_layer][j] * f_sensor_inputs[i] * f_layer_inputs[i]
						m_pfWeightMatrix[un_layer][nIndexWeights] += m_fEta * m_fOutputMatrix[un_layer][i] * f_sensor_inputs[i]; 
					}
				}
			}
			/* If hebbian learning */
			else if ( m_unLearningLayerFlag[un_layer] == 2 )
			{
 						if (f_layer_inputs[j]<0.25) f_layer_inputs[j]=0.0;
//						if (f_layer_inputs[j]>0.55) f_layer_inputs[j]=1.0;
						if (f_sensor_inputs[i]==1.0) f_layer_inputs[j]=1.0;

						/* If in the diagonal, make for any leearning */
						if ( i == j )
						{
							m_pfWeightMatrix[un_layer][nIndexWeights] += m_fEta * m_fOutputMatrix[un_layer][i] * f_layer_inputs[j] -
								m_fEpsilon * m_pfWeightMatrix[un_layer][nIndexWeights] * fOutputAverage;
						}
						/* If not in the diagonal */
						else {
							/* Check for DiagonalFlag */
							if ( m_unLearningDiagonalFlag[un_layer] == 0 )
							{
								m_pfWeightMatrix[un_layer][nIndexWeights] += m_fEta * m_fOutputMatrix[un_layer][i] * f_layer_inputs[j] -
									m_fEpsilon * m_pfWeightMatrix[un_layer][nIndexWeights] * fOutputAverage;
							}
						}
			}
			//else if ( m_unLearningLayerFlag[un_layer] == 3 )
			//{
				//fOutputAverage *= 4.0/m_unNumberOfLayerOutputs[un_layer];
				//if ((j==0) || (j==1) || (j==6) || (j==7))
				//{
					//if ((i==0) || (i==1) || (i==6) || (i==7))
					//{

						//if (f_layer_inputs[j]<0.25) f_layer_inputs[j]=0.0;
						////						if (f_layer_inputs[j]>0.55) f_layer_inputs[j]=1.0;
						//if (f_sensor_inputs[i]==1.0) f_layer_inputs[j]=1.0;

						///* If in the diagonal, make for any leearning */
						//if ( i == j )
						//{
							//m_pfWeightMatrix[un_layer][nIndexWeights] += m_fEta * m_fOutputMatrix[un_layer][i] * f_layer_inputs[j] -
								//m_fEpsilon * m_pfWeightMatrix[un_layer][nIndexWeights] * fOutputAverage;
						//}
						///* If not in the diagonal */
						//else {
							///* Check for DiagonalFlag */
							//if ( m_unLearningDiagonalFlag[un_layer] == 0 )
							//{
								//m_pfWeightMatrix[un_layer][nIndexWeights] += m_fEta * m_fOutputMatrix[un_layer][i] * f_layer_inputs[j] -
									//m_fEpsilon * m_pfWeightMatrix[un_layer][nIndexWeights] * fOutputAverage;
							//}
						//}
					//}
					//else
					//{
						//m_pfWeightMatrix[un_layer][nIndexWeights]=0.0;
					//}
				//}
				//else
				//{
					//m_pfWeightMatrix[un_layer][nIndexWeights]=0.0;
				//}
			//}
			nIndexWeights++;
		}
	}
}

/******************************************************************************/
/******************************************************************************/

void CCTRNNDistributedController::SetUpperBounds(float fUB){
	m_fUpperBounds=fUB;
}

/******************************************************************************/
/******************************************************************************/

void CCTRNNDistributedController::SetLowerBounds(float fLB){
	m_fLowerBounds=fLB;
}

/******************************************************************************/
/******************************************************************************/
void CCTRNNDistributedController::Reset( void )
{
  /* Init to 0.0 all OutputMatrix's */
	for ( int i = 0 ; i < m_unNumberOfLayers ; i++ )
	{
		/* For each Neuron in each Leayer */
		for ( int j = 0 ; j < m_unNumberOfLayerOutputs[i] ; j++ )
		{
			/* Init to 0.0 */
			m_fOutputMatrix[i][j] = 0.0;
			m_fOutputMatrixOld[i][j] = 0.0;
		}
	}
  
  /* Reset every layer */
  for (TLayerIterator i = m_vecLayers.begin() ; i != m_vecLayers.end(); i++)
  {
    (*i)->Reset();
  }
}


