#ifndef CTRNNEURONEXP_H
#define CTRNNEURONEXP_H

/******************************************************************************/
/******************************************************************************/

#include "experiment.h"

/******************************************************************************/
/******************************************************************************/

class CCTRNNExp : public CExperiment
{
public:
    CCTRNNExp ( const char* pch_name , const char* paramsFile,
										 unsigned int un_chromosome_length, unsigned int un_fitness_function,
										 double f_evaluation_time, double f_upper_bounds, double f_lower_bounds, 
										 bool b_evolutionary_flag, bool b_learning_flag);
		~CCTRNNExp ( void );
protected:
    // Overwritten from the superclasses:
    CArena* CreateArena();
		void AddActuators(CEpuck* pc_epuck);
    void AddSensors(CEpuck* pc_epuck);
    void SetController(CEpuck* pc_epuck);
    void CreateAndAddEpucks(CSimulator* pc_simulator);
   	void Reset(); 
   	void RandomPositionAndOrientation(); 
private:

		bool m_bEvolutionaryFlag;
		bool m_bLearningFlag;
		
		/* VARIABLES*/
		/* Extra */
		int m_nRobotsNumber;
		int m_nWriteToFile;
		dVector2* m_pcvRobotPositions;
		double* m_fRobotOrientations;
		int m_nRunTime;

		/* Environment */
		int m_nLightObjectNumber;
		dVector2 *m_pcvLightObjects;
		int m_nBlueLightObjectNumber;
		dVector2 *m_pcvBlueLightObjects;
		int m_nRedLightObjectNumber;
		dVector2 *m_pcvRedLightObjects;
		int m_nNumberOfGroundArea;
		dVector2* m_vGroundAreaCenter;
		double* m_fGroundAreaExternalRadius;
		double * m_fGroundAreaInternalRadius;
		double * m_fGroundAreaColor;

		/* Sensors */
		float 	m_fLightSensorRange;
		float 	m_fBlueLightSensorRange;
		float 	m_fRedLightSensorRange;
		
		double 	m_fBatterySensorRange;
		double 	m_fBatteryChargeCoef;
		double 	m_fBatteryDischargeCoef;
		
		double m_fBlueBatterySensorRange;
		double m_fBlueBatteryChargeCoef;
		double m_fBlueBatteryDischargeCoef;
		
		double m_fRedBatterySensorRange;
		double m_fRedBatteryChargeCoef;
		double m_fRedBatteryDischargeCoef;

		/* Morphology */
		unsigned int 	m_unProximitySensorsUsedNumber;
		unsigned int* m_unProximitySensorsUsedValue;
		unsigned int 	m_unContactSensorsUsedNumber;
		unsigned int* m_unContactSensorsUsedValue;
		unsigned int 	m_unLightSensorsUsedNumber;
		unsigned int* m_unLightSensorsUsedValue;
		unsigned int 	m_unBlueLightSensorsUsedNumber;
		unsigned int* m_unBlueLightSensorsUsedValue;
		unsigned int 	m_unRedLightSensorsUsedNumber;
		unsigned int* m_unRedLightSensorsUsedValue;
		unsigned int 	m_unGroundSensorsUsedNumber;
		unsigned int* m_unGroundSensorsUsedValue;
		
		/* Genetic */	
		float m_fUpperBounds,m_fLowerBounds;
		float m_fInitAreaX, m_fInitAreaY;
    
		/* Neuronal */
		unsigned int 		m_unNumberOfLayers;
		unsigned int* 	m_unLayersOutputs; 
		unsigned int* 	m_unLayerSensorType;
		unsigned int* 	m_unActivationFunction;
		unsigned int** 	m_mAdjacencyMatrix;
		
		/* Learning */
		double m_fEta;
		double m_fEpsilon;
		unsigned int* 	m_unLearningLayerFlag;
		unsigned int* 	m_unEvoDevoLayerFlag;
		unsigned int* 	m_unLearningDiagonalFlag;

};

/******************************************************************************/
/******************************************************************************/

#endif
