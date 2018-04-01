#ifndef IRI1CONTROLLER_H_
#define IRI1CONTROLLER_H_

/******************************************************************************/
/******************************************************************************/

#include "controller.h"

/******************************************************************************/
/******************************************************************************/

class CIri1Controller : public CController
{
public:

    CIri1Controller (const char* pch_name, CEpuck* pc_epuck, int n_wrtie_to_file);
    ~CIri1Controller();
    void SimulationStep(unsigned n_step_number, double f_time, double f_step_interval);

private:
    /* ROBOT */
    CEpuck* m_pcEpuck;
   
     /* SENSORS */
    CWheelsActuator* m_acWheels;
    CEpuckProximitySensor* m_seProx;
    CRealLightSensor* m_seLight;
    CRealBlueLightSensor* m_seBlueLight;
    CRealRedLightSensor* m_seRedLight;
    CContactSensor* m_seContact;
    CGroundSensor* m_seGround;
    CGroundMemorySensor* m_seGroundMemory;
    CBatterySensor* m_seBattery;
    CRedBatterySensor* m_seRedBattery;
    CEncoderSensor* m_seEncoder;

    /* Global Variables */
    double 		m_fLeftSpeed;
    double 		m_fRightSpeed;
    double**	m_fActivationTable;
    int 			m_nWriteToFile;
    double 		m_fTime;
    double    fBattToForageInhibitor;
    double    fMoneyToForageInhibitor;
    double    fDeliverToSwitchOffInhibitor;
    double    fLastRandomMovement;  // Ensure convergence
   
    /* Odometry */
    float     m_fOrientation; 
    dVector2  m_vPosition;
    int       m_nState;
    dVector2  *m_vPositionsPlanning;
    int       m_nPathPlanningStops;
    int       m_nRobotActualGridX;
    int       m_nRobotActualGridY;

    /*
     * Do nothing --> 0
     * Go to get object --> 1
     * Go to deliver object --> 2
     * Go to switch off blue light --> 3
     */
    int       m_nDeliverStatus;
    
    int       m_nNestFound;
    int       m_nNestGridX;
    int       m_nNestGridY;
    
    int       m_nPreyFound;
    vector<dVector2> m_vecPrey;
    vector<dVector2> m_vecPreyNotChecked;
    int       m_nActualPreyGridX;
    int       m_nActualPreyGridY;
    bool      m_bIsInPrey;

    int       m_nPathPlanningDone;
    int       m_nStepTime;
    int       m_nPacketsAttended;
    /* Functions */

    void ExecuteBehaviors   ( void );
    void Coordinator        ( void );

    void CalcPositionAndOrientation ( double *f_encoder );
    string  pathFind                ( const int &xStart, const int &yStart, const int &xFinish, const int &yFinish );
  
    void PrintMap ( int *print_map  );
    void SaveStats ( int *print_map  );
    /* Behaviors */
    void ObstacleAvoidance     ( unsigned int un_priority );
    void Navigate              ( unsigned int un_priority );
    void GoLoad                ( unsigned int un_priority );

    void GoGetMoney            ( unsigned int un_priority );
    void GoBlueLight           ( unsigned int un_priority );

    void ComputeActualCell     ( unsigned int un_priority );
    void PathPlanning          ( unsigned int un_priority );
    void GoGetOrDeliverObject  ( unsigned int un_priority );

    /* Helpers */
    bool IsPreyAlreadyChecked(dVector2* position);
    bool SeemsToBeInTheRightPrey();
};

#endif
