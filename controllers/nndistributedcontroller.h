#ifndef NNDISTRIBUTEDCONTROLLER_H_
#define NNDISTRIBUTEDCONTROLLER_H_

/******************************************************************************/
/******************************************************************************/

#include <math.h>
#include <vector>
#include <list>

#include <iostream>
#include <sys/types.h>
#include <fstream>
#include "random.h"

using namespace std;

class CNNDistributedController;

/******************************************************************************/
/******************************************************************************/

#include "controller.h"
#include "layercontroller.h"
#include "contactsensor.h"
#include "collisionmanager.h"
/******************************************************************************/
/******************************************************************************/

typedef vector<CLayerController*>            TLayerVector;
typedef vector<CLayerController*>::iterator  TLayerIterator;

/******************************************************************************/
/******************************************************************************/

class CNNDistributedController : public CController
{
	public:
		CNNDistributedController(const char* pch_name, CEpuck* pc_epuck,
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
				double f_nu, double f_epsilon,
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
				unsigned int* un_red_light_value);
		~CNNDistributedController();
		
		virtual void SimulationStep(unsigned n_step_number, double f_time, double f_step_interval);
		
		double* LoadWeights(const char* pch_filename);

		void Reset();

		void SaveState(const char* pch_filename);
		void LoadState(const char* pch_filename);
		void SetWeights(unsigned int un_number_of_weights, double* pf_weights); 

		void SetUpperBounds(float fUB);
		void SetLowerBounds(float fUB);
protected:

		CEpuck* m_pcEpuck;
		double m_fTime;
		int m_nWriteToFile;

    unsigned int GetNumberOfSensorInputs();
    unsigned int GetNumberOfActuatorOutputs();

    unsigned int m_unNumberOfActuatorOutputs;

    double*      m_pfInputs;
		
		unsigned int 		m_unNumberOfLayers;
		unsigned int* 	m_unNumberOfLayerInputs;
		unsigned int* 	m_unNumberOfSensorInputs;
		unsigned int*		m_unNumberOfLayerOutputs;
		unsigned int*		m_unLayerSensorType;
		unsigned int*		m_unActivationFunction;
		unsigned int**	m_mAdjacencyMatrix;
		double ** 			m_fOutputMatrix;
		unsigned int*		m_unLearningLayerFlag;
		unsigned int*		m_unLearningDiagonalFlag;
		unsigned int*		m_unEvoDevoLayerFlag;
		
		double ** 			m_pfWeightMatrix;
		unsigned int*		m_unNumberOfParameters;

		double m_fUpperBounds;
		double m_fLowerBounds;
		
		TLayerVector m_vecLayers;


		bool m_bEvolutionaryFlag;

		/* Learning */
		void LearningFunction ( double* f_layer_inputs, double* f_sensor_inputs, unsigned int un_layer);
		bool m_bLearningFlag;
		double m_fEta;
		double m_fEpsilon;

		/* Morphology */
		unsigned int  m_unProximitySensorsUsedNumber;
		unsigned int* m_unProximitySensorsUsedValue;
		unsigned int  m_unContactSensorsUsedNumber;
		unsigned int* m_unContactSensorsUsedValue;
		unsigned int  m_unLightSensorsUsedNumber;
		unsigned int* m_unLightSensorsUsedValue;
		unsigned int  m_unGroundSensorsUsedNumber;
		unsigned int* m_unGroundSensorsUsedValue;
		unsigned int  m_unBlueLightSensorsUsedNumber;
		unsigned int* m_unBlueLightSensorsUsedValue;
		unsigned int  m_unRedLightSensorsUsedNumber;
		unsigned int* m_unRedLightSensorsUsedValue;

		/* DEBUG AGUTI */
		unsigned int m_unDebugCollisions;
		float m_fDebugMaxLinearSpeed;
		float m_fDebugMaxDist;
		dVector2 m_vDebugOldPos;
};

/******************************************************************************/
/******************************************************************************/

#endif
