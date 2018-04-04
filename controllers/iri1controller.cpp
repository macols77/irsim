/******************* INCLUDES ******************/
/***********************************************/

/******************** General ******************/
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <sys/time.h>
#include <iostream>

#include <iomanip>
#include <queue>
#include <string>
#include <math.h>
#include <ctime>
#include <cstdlib>
#include <cstdio>

/******************** Simulator ****************/
/******************** Sensors ******************/
#include "epuckproximitysensor.h"
#include "contactsensor.h"
#include "reallightsensor.h"
#include "realbluelightsensor.h"
#include "realredlightsensor.h"
#include "groundsensor.h"
#include "groundmemorysensor.h"
#include "batterysensor.h"
#include "redbatterysensor.h"
#include "encodersensor.h"

/******************** Actuators ****************/
#include "wheelsactuator.h"

/******************** Controller **************/
#include "iri1controller.h"


/******************************************************************************/
/******************************************************************************/

extern gsl_rng *rng;
extern long int rngSeed;

const int mapGridX = 20;
const int mapGridY = 20;
double mapLengthX = 3.0;
double mapLengthY = 3.0;
int robotStartGridX = 10;
int robotStartGridY = 10;

const int n = mapGridX; // horizontal size of the map
const int m = mapGridY; // vertical size size of the map
static int map[n][m];
static int onlineMap[n][m];
static int closed_nodes_map[n][m]; // map of closed (tried-out) nodes
static int open_nodes_map[n][m]; // map of open (not-yet-tried) nodes
static int dir_map[n][m]; // map of directions
const int dir = 8; // number of possible directions to go at any position
//if dir==4
//static int dx[dir]={1, 0, -1, 0};
//static int dy[dir]={0, 1, 0, -1};
// if dir==8
static int dx[dir] = {1, 1, 0, -1, -1, -1, 0, 1};
static int dy[dir] = {0, 1, 1, 1, 0, -1, -1, -1};

#define ERROR_DIRECTION 0.05
#define ERROR_POSITION  0.02
/******************************************************************************/
/******************************************************************************/

using namespace std;
/******************************************************************************/
/******************************************************************************/

#define BEHAVIORS    7

#define AVOID_PRIORITY           0
#define RELOAD_PRIORITY          1
#define FORAGE_PRIORITY          2
#define NAVIGATE_PRIORITY        3
#define GO_GOAL_PRIORITY         4
#define GO_MONEY_PRIORITY        5
#define GO_SWITCH_OFF_BLUE_LIGHT 6

/* Threshold to avoid obstacles */
#define PROXIMITY_THRESHOLD 0.4
/* Threshold to define the battery discharged */
#define BATTERY_THRESHOLD 0.5
/* Threshold to define the red battery discharged */
#define REDBATTERY_THRESHOLD 0.3
/* Threshold to reduce the speed of the robot */
#define NAVIGATE_LIGHT_THRESHOLD 0.9

#define SPEED 500

#define NO_OBSTACLE 0
#define OBSTACLE    1
#define START       2
#define PATH        3
#define END         4
#define NEST        5
#define PREY        6

#define MAX_GROUND_RADIUS 2.2

#define BLUE_OBJECT 1

#define RANDOM_NAVIGATION 0.5

/******************************************************************************/
/******************************************************************************/

class node {
    // current position
    int xPos;
    int yPos;
    // total distance already travelled to reach the node
    int level;
    // priority=level+remaining distance estimate
    int priority;  // smaller: higher priority

public:
    node(int xp, int yp, int d, int p) {
        xPos = xp;
        yPos = yp;
        level = d;
        priority = p;
    }

    int getxPos() const { return xPos; }

    int getyPos() const { return yPos; }

    int getLevel() const { return level; }

    int getPriority() const { return priority; }

    void updatePriority(const int &xDest, const int &yDest) {
        priority = level + estimate(xDest, yDest) * 10; //A*
    }

    // give better priority to going strait instead of diagonally
    void nextLevel(const int &i) // i: direction
    {
        level += (dir == 8 ? (i % 2 == 0 ? 10 : 14) : 10);
    }

    // Estimation function for the remaining distance to the goal.
    const int &estimate(const int &xDest, const int &yDest) const {
        static int xd, yd, d;
        xd = xDest - xPos;
        yd = yDest - yPos;

        // Euclidian Distance
        d = static_cast<int>(sqrt(xd * xd + yd * yd));

        // Manhattan distance
        //d=abs(xd)+abs(yd);

        // Chebyshev distance
        //d=max(abs(xd), abs(yd));

        return (d);
    }
};

/******************************************************************************/
/******************************************************************************/

// Determine priority (in the priority queue)
bool operator<(const node &a, const node &b) {
    return a.getPriority() > b.getPriority();
}


/******************************************************************************/
/******************************************************************************/
CIri1Controller::CIri1Controller(const char *pch_name, CEpuck *pc_epuck, int n_write_to_file) : CController(pch_name,
                                                                                                            pc_epuck) {
    m_nWriteToFile = n_write_to_file;

    /* Set epuck */
    m_pcEpuck = pc_epuck;
    /* Set Wheels */
    m_acWheels = (CWheelsActuator *) m_pcEpuck->GetActuator(ACTUATOR_WHEELS);
    /* Set Prox Sensor */
    m_seProx = (CEpuckProximitySensor *) m_pcEpuck->GetSensor(SENSOR_PROXIMITY);
    /* Set light Sensor */
    m_seLight = (CRealLightSensor *) m_pcEpuck->GetSensor(SENSOR_REAL_LIGHT);
    /* Set Blue light Sensor */
    m_seBlueLight = (CRealBlueLightSensor *) m_pcEpuck->GetSensor(SENSOR_REAL_BLUE_LIGHT);
    /* Set Red light Sensor */
    m_seRedLight = (CRealRedLightSensor *) m_pcEpuck->GetSensor(SENSOR_REAL_RED_LIGHT);
    /* Set contact Sensor */
    m_seContact = (CContactSensor *) m_pcEpuck->GetSensor(SENSOR_CONTACT);
    /* Set ground Sensor */
    m_seGround = (CGroundSensor *) m_pcEpuck->GetSensor(SENSOR_GROUND);
    /* Set ground memory Sensor */
    m_seGroundMemory = (CGroundMemorySensor *) m_pcEpuck->GetSensor(SENSOR_GROUND_MEMORY);
    /* Set battery Sensor */
    m_seBattery = (CBatterySensor *) m_pcEpuck->GetSensor(SENSOR_BATTERY);
    /* Set red battery Sensor */
    m_seRedBattery = (CRedBatterySensor *) m_pcEpuck->GetSensor(SENSOR_RED_BATTERY);
    /* Set encoder Sensor */
    m_seEncoder = (CEncoderSensor *) m_pcEpuck->GetSensor(SENSOR_ENCODER);
    m_seEncoder->InitEncoderSensor(m_pcEpuck);


    /* Initialize Motor Variables */
    m_fLeftSpeed = 0.0;
    m_fRightSpeed = 0.0;

    /* Initialize Inhibitors */
    fBattToForageInhibitor = 1.0;
    fMoneyToForageInhibitor = 1.0;
    fDeliverToSwitchOffInhibitor = 1.0;

    /* Initialize Activation Table */
    m_fActivationTable = new double *[BEHAVIORS];
    for (int i = 0; i < BEHAVIORS; i++) {
        m_fActivationTable[i] = new double[3];
    }

    /* Odometry */
    m_nState = 0;
    m_nPathPlanningStops = 0;
    m_fOrientation = 0.0;
    m_vPosition.x = 0.0;
    m_vPosition.y = 0.0;
    m_bIsInPrey = false;

    /* Set Actual Position to robot Start Grid */
    m_nRobotActualGridX = robotStartGridX;
    m_nRobotActualGridY = robotStartGridY;

    /* Init onlineMap */
    for (int y = 0; y < m; y++)
        for (int x = 0; x < n; x++)
            onlineMap[x][y] = OBSTACLE;

    /* DEBUG */
    PrintMap(&onlineMap[0][0]);
    /* DEBUG */

    /* Initialize status of foraging */
    m_nDeliverStatus = 0;

    /* Initialize Nest/Prey variables */
    m_nNestGridX = 0;
    m_nNestGridY = 0;
    m_nActualPreyGridX = 0;
    m_nActualPreyGridY = 0;
    m_nPreyFound = 0;
    m_nNestFound = 0;
    m_vecPrey.clear();
    m_vecPreyNotChecked.clear();

    /* Initialize PathPlanning Flag*/
    m_nPathPlanningDone = 0;

    m_nStepTime = 0;
    m_nPacketsAttended = 0;
    fLastRandomMovement = 0.0;
}

/******************************************************************************/
/******************************************************************************/

CIri1Controller::~CIri1Controller() {
    for (int i = 0; i < BEHAVIORS; i++) {
        delete[] m_fActivationTable;
    }
}


/******************************************************************************/
/******************************************************************************/

void CIri1Controller::SimulationStep(unsigned n_step_number, double f_time, double f_step_interval) {
    /* Move time to global variable, so it can be used by the bahaviors to write to files*/
    m_fTime = f_time;

    /* Execute the levels of competence */
    ExecuteBehaviors();

    /* Execute Coordinator */
    Coordinator();

    /* Set Speed to wheels */
    m_acWheels->SetSpeed(m_fLeftSpeed, m_fRightSpeed);

    if (m_nWriteToFile) {
        /* INIT: WRITE TO FILES */
        /* Write robot position and orientation */
        FILE *filePosition = fopen("outputFiles/robotPosition", "a");
        fprintf(filePosition, "%2.4f %2.4f %2.4f %2.4f\n", m_fTime, m_pcEpuck->GetPosition().x,
                m_pcEpuck->GetPosition().y, m_pcEpuck->GetRotation());
        fclose(filePosition);

        /* Write robot wheels speed */
        FILE *fileWheels = fopen("outputFiles/robotWheels", "a");
        fprintf(fileWheels, "%2.4f %2.4f %2.4f \n", m_fTime, m_fLeftSpeed, m_fRightSpeed);
        fclose(fileWheels);
        /* END WRITE TO FILES */
    }

    if (m_nStepTime % 100 == 0) {
        SaveStats(&onlineMap[0][0]);
    }

    m_nStepTime++;
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::ExecuteBehaviors(void) {
    for (int i = 0; i < BEHAVIORS; i++) {
        m_fActivationTable[i][2] = 0.0;
    }

    /* Release Inhibitors */
    fBattToForageInhibitor = 1.0;
    fMoneyToForageInhibitor = 1.0;
    fDeliverToSwitchOffInhibitor = 1.0;

    /* Set Leds to BLACK */
    m_pcEpuck->SetAllColoredLeds(LED_COLOR_BLACK);

    /* Execute Behaviors */
    ObstacleAvoidance(AVOID_PRIORITY);
    GoLoad(RELOAD_PRIORITY);

    GoGetMoney(GO_MONEY_PRIORITY);

    ComputeActualCell(GO_GOAL_PRIORITY);
    PathPlanning(GO_GOAL_PRIORITY);
    GoGetOrDeliverObject(GO_GOAL_PRIORITY);
    GoBlueLight(GO_SWITCH_OFF_BLUE_LIGHT);

    Navigate(NAVIGATE_PRIORITY);
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::Coordinator(void) {
    /* Create counter for behaviors */
    int nBehavior;
    /* Create angle of movement */
    double fAngle = 0.0;
    /* Create vector of movement */
    dVector2 vAngle;
    vAngle.x = 0.0;
    vAngle.y = 0.0;

    /* For every Behavior */
    for (nBehavior = 0; nBehavior < BEHAVIORS; nBehavior++) {
        /* If behavior is active */
        if (m_fActivationTable[nBehavior][2] == 1.0) {
            /* DEBUG */
            printf("Behavior %d: %2f\n", nBehavior, m_fActivationTable[nBehavior][0]);
            /* DEBUG */
            vAngle.x += m_fActivationTable[nBehavior][1] * cos(m_fActivationTable[nBehavior][0]);
            vAngle.y += m_fActivationTable[nBehavior][1] * sin(m_fActivationTable[nBehavior][0]);
        }
    }

    /* Calc angle of movement */
    fAngle = atan2(vAngle.y, vAngle.x);
    /* DEBUG */
    printf("fAngle: %2f\n", fAngle);
    printf("\n");
    /* DEBUG */

    if (fAngle > 0) {
        m_fLeftSpeed = SPEED * (1 - fmin(fAngle, ERROR_DIRECTION) / ERROR_DIRECTION);
        m_fRightSpeed = SPEED;
    } else {
        m_fLeftSpeed = SPEED;
        m_fRightSpeed = SPEED * (1 - fmin(-fAngle, ERROR_DIRECTION) / ERROR_DIRECTION);
    }

    if (m_nWriteToFile) {
        /* INIT: WRITE TO FILES */
        /* Write coordinator ouputs */
        FILE *fileOutput = fopen("outputFiles/coordinatorOutput", "a");
        fprintf(fileOutput, "%2.4f %d %2.4f %2.4f \n", m_fTime, nBehavior, m_fLeftSpeed, m_fRightSpeed);
        fclose(fileOutput);
        /* END WRITE TO FILES */
    }
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::ObstacleAvoidance(unsigned int un_priority) {
    /* Leer Sensores de Proximidad */
    double *prox = m_seProx->GetSensorReading(m_pcEpuck);

    double fMaxProx = 0.0;
    const double *proxDirections = m_seProx->GetSensorDirections();

    dVector2 vRepelent;
    vRepelent.x = 0.0;
    vRepelent.y = 0.0;

    /* Calc vector Sum */
    for (int i = 0; i < m_seProx->GetNumberOfInputs(); i++) {
        vRepelent.x += prox[i] * cos(proxDirections[i]);
        vRepelent.y += prox[i] * sin(proxDirections[i]);

        if (prox[i] > fMaxProx)
            fMaxProx = prox[i];
    }

    /* Calc pointing angle */
    float fRepelent = atan2(vRepelent.y, vRepelent.x);
    /* Create repelent angle */
    fRepelent -= M_PI;
    /* Normalize angle */
    while (fRepelent > M_PI) fRepelent -= 2 * M_PI;
    while (fRepelent < -M_PI) fRepelent += 2 * M_PI;

    m_fActivationTable[un_priority][0] = fRepelent;
    m_fActivationTable[un_priority][1] = 1.0; //fMaxProx;

    /* If above a threshold */
    if (fMaxProx > PROXIMITY_THRESHOLD) {
        /* Set Leds to GREEN */
        m_pcEpuck->SetAllColoredLeds(LED_COLOR_GREEN);
        /* Mark Behavior as active */
        m_fActivationTable[un_priority][2] = 1.0;
    }

    if (m_nWriteToFile) {
        /* INIT WRITE TO FILE */
        /* Write level of competence ouputs */
        FILE *fileOutput = fopen("outputFiles/avoidOutput", "a");
        fprintf(fileOutput, "%2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f ", m_fTime, prox[0],
                prox[1], prox[2], prox[3], prox[4], prox[5], prox[6], prox[7], fMaxProx, fRepelent);
        fprintf(fileOutput, "%2.4f %2.4f %2.4f\n", m_fActivationTable[un_priority][2],
                m_fActivationTable[un_priority][0], m_fActivationTable[un_priority][1]);
        fclose(fileOutput);
        /* END WRITE TO FILE */
    }

}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::Navigate(unsigned int un_priority) {

    double randomizeMovement = (double) rand() / RAND_MAX;

    if (randomizeMovement < RANDOM_NAVIGATION
        && !m_fActivationTable[GO_GOAL_PRIORITY][2] == 1.0
        && !m_fActivationTable[GO_SWITCH_OFF_BLUE_LIGHT][2] == 1.0
        && fMoneyToForageInhibitor == 1.0
        && fBattToForageInhibitor == 1.0) {

        float angle = m_fActivationTable[un_priority][0];

        if (fLastRandomMovement == 1.0) {
            angle += M_PI;
            fLastRandomMovement = 0.0;
        } else {
            angle -= M_PI;
            fLastRandomMovement = 1.0;
        }

        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;

        m_fActivationTable[un_priority][0] = angle;
        m_fActivationTable[un_priority][1] = 0.5;

        printf("RANDOMIZE NAVIGATE \n");
    } else {
        m_fActivationTable[un_priority][0] = 0.0;
        m_fActivationTable[un_priority][1] = 0.5;
    }
    m_fActivationTable[un_priority][2] = 1.0;

    printf("Navigate: %1.4f angle %1.4f distance\n", m_fActivationTable[un_priority][0],
           m_fActivationTable[un_priority][1]);

    if (m_nWriteToFile) {
        FILE *fileOutput = fopen("outputFiles/navigateOutput", "a");
        fprintf(fileOutput, "%2.4f %2.4f %2.4f %2.4f \n", m_fTime, m_fActivationTable[un_priority][2],
                m_fActivationTable[un_priority][0], m_fActivationTable[un_priority][1]);
        fclose(fileOutput);
    }

}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::GoLoad(unsigned int un_priority) {
    /* Leer Battery Sensores */
    double *battery = m_seBattery->GetSensorReading(m_pcEpuck);

    /* Leer Sensores de Luz */
    double *light = m_seLight->GetSensorReading(m_pcEpuck);

    double fMaxLight = 0.0;
    const double *lightDirections = m_seLight->GetSensorDirections();

    /* We call vRepelent to go similar to Obstacle Avoidance, although it is an aproaching vector */
    dVector2 vRepelent;
    vRepelent.x = 0.0;
    vRepelent.y = 0.0;

    /* Calc vector Sum */
    for (int i = 0; i < m_seProx->GetNumberOfInputs(); i++) {
        vRepelent.x += light[i] * cos(lightDirections[i]);
        vRepelent.y += light[i] * sin(lightDirections[i]);

        if (light[i] > fMaxLight)
            fMaxLight = light[i];
    }

    /* Calc pointing angle */
    float fRepelent = atan2(vRepelent.y, vRepelent.x);

    /* Normalize angle */
    while (fRepelent > M_PI) fRepelent -= 2 * M_PI;
    while (fRepelent < -M_PI) fRepelent += 2 * M_PI;

    m_fActivationTable[un_priority][0] = fRepelent;
    m_fActivationTable[un_priority][1] = fMaxLight;

    /* If battery below a BATTERY_THRESHOLD */
    if (battery[0] < BATTERY_THRESHOLD) {
        /* Inibit Forage */
        fBattToForageInhibitor = 0.0;
        /* Set Leds to RED */
        m_pcEpuck->SetAllColoredLeds(LED_COLOR_RED);

        /* Mark behavior as active */
        m_fActivationTable[un_priority][2] = 1.0;
    }


    if (m_nWriteToFile) {
        /* INIT WRITE TO FILE */
        FILE *fileOutput = fopen("outputFiles/batteryOutput", "a");
        fprintf(fileOutput, "%2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f ", m_fTime, battery[0],
                light[0], light[1], light[2], light[3], light[4], light[5], light[6], light[7]);
        fprintf(fileOutput, "%2.4f %2.4f %2.4f\n", m_fActivationTable[un_priority][2],
                m_fActivationTable[un_priority][0], m_fActivationTable[un_priority][1]);
        fclose(fileOutput);
        /* END WRITE TO FILE */
    }
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::GoGetMoney(unsigned int un_priority) {
    /* Leer Sensores de Luz roja */
    double *redLight = m_seRedLight->GetSensorReading(m_pcEpuck);

    /* Leer Sensor de bateria roja (dinero) */
    double *redBattery = m_seRedBattery->GetSensorReading(m_pcEpuck);

    /* We call vRepelent to go similar to Obstacle Avoidance, although it is an aproaching vector */
    dVector2 vRepelent;
    vRepelent.x = 0.0;
    vRepelent.y = 0.0;

    double fMaxLight = 0.0;
    const double *lightDirections = m_seRedLight->GetSensorDirections();

    for (int i = 0; i < m_seRedLight->GetNumberOfInputs(); i++) {
        vRepelent.x += redLight[i] * cos(lightDirections[i]);
        vRepelent.y += redLight[i] * sin(lightDirections[i]);

        if (redLight[i] > fMaxLight)
            fMaxLight = redLight[i];
    }

    /* Calc pointing angle */
    float fAngle = atan2(vRepelent.y, vRepelent.x);

    /* Normalize angle */
    while (fAngle > M_PI) fAngle -= 2 * M_PI;
    while (fAngle < -M_PI) fAngle += 2 * M_PI;

    m_fActivationTable[un_priority][0] = fAngle;
    m_fActivationTable[un_priority][1] = fMaxLight;

    printf("Red light is %2.4f\n", fMaxLight);
    printf("Red battery is  %2.4f\n", redBattery[0]);

    if (redBattery[0] < REDBATTERY_THRESHOLD && fBattToForageInhibitor == 1.0) {
        /*if (fBattToForageInhibitor * fMaxLight > 0.8) {
          m_seRedLight->SwitchNearestLight(0);
        } else if (fBattToForageInhibitor * fMaxLight > 0.0) {

        }*/

        /* Set Leds to GREEN */
        m_pcEpuck->SetAllColoredLeds(LED_COLOR_GREEN);
        /* Mark Behavior as active */
        m_fActivationTable[un_priority][2] = 1.0;

        // Inhibite object movement.
        fMoneyToForageInhibitor = 0.0;

    }
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::GoBlueLight(unsigned int un_priority) {
    /* Leer Sensores de Luz roja */
    double *blueLight = m_seBlueLight->GetSensorReading(m_pcEpuck);


    /* We call vRepelent to go similar to Obstacle Avoidance, although it is an aproaching vector */
    dVector2 vRepelent;
    vRepelent.x = 0.0;
    vRepelent.y = 0.0;

    double fMaxLight = 0.0;
    const double *lightDirections = m_seBlueLight->GetSensorDirections();

    for (int i = 0; i < m_seBlueLight->GetNumberOfInputs(); i++) {
        vRepelent.x += blueLight[i] * cos(lightDirections[i]);
        vRepelent.y += blueLight[i] * sin(lightDirections[i]);

        if (blueLight[i] > fMaxLight)
            fMaxLight = blueLight[i];
    }

    /* Calc pointing angle */
    float fAngle = atan2(vRepelent.y, vRepelent.x);

    /* Normalize angle */
    while (fAngle > M_PI) fAngle -= 2 * M_PI;
    while (fAngle < -M_PI) fAngle += 2 * M_PI;

    m_fActivationTable[un_priority][0] = fAngle;
    m_fActivationTable[un_priority][1] = 1 - fMaxLight;

    // We've just arrived to the light, so turn off it.
    if (fBattToForageInhibitor * fMoneyToForageInhibitor * fDeliverToSwitchOffInhibitor * fMaxLight > 0.8) {
        m_seBlueLight->SwitchNearestLight(0);
        m_fActivationTable[un_priority][2] = 0.0;
        // we find the light so reset status
            m_nDeliverStatus = 0;
        m_nPacketsAttended++;
    } else if (fBattToForageInhibitor * fMoneyToForageInhibitor * fDeliverToSwitchOffInhibitor * fMaxLight > 0.1) {
        /* Set Leds to GREEN */
        m_pcEpuck->SetAllColoredLeds(LED_COLOR_BLUE);
        /* Mark Behavior as active */
        m_fActivationTable[un_priority][2] = 1.0;
        }
    }
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::ComputeActualCell(unsigned int un_priority) {
    printf("Init compute cell\n");

    /* Leer Encoder */
    double *encoder = m_seEncoder->GetSensorReading(m_pcEpuck);
    /* Leer Sensores Memory */
    double *ground = m_seGround->GetSensorReading(m_pcEpuck);

    printf("Deliver status %d\n", m_nDeliverStatus);

    if (m_nDeliverStatus == 0) {

        /* Leer Sensores de Luz azul */
        double *blueLight = m_seBlueLight->GetSensorReading(m_pcEpuck);

        for (int i = 0; i < m_seBlueLight->GetNumberOfInputs(); i++) {
            // Activate deliver behaviour
            if (blueLight[i] > 0.0) {
                m_nDeliverStatus = 1;
                printf("Updating deliver status to 1\n");
                m_vecPreyNotChecked.clear();
                m_vecPreyNotChecked = m_vecPrey;

                if (m_vecPreyNotChecked.size() > 0) {
                    m_nActualPreyGridX = m_vecPreyNotChecked.front().x;
                    m_nActualPreyGridY = m_vecPreyNotChecked.front().y;
                    m_nPathPlanningDone = 0;
                    /* Restart PathPlanning state */
                    m_nState = 0;
                    printf("Setting prey to go [%2.2f, %2.2f] with pathplanning: %d\n", m_nActualPreyGridX,
                           m_nActualPreyGridY, m_nPathPlanningDone);
                }

                break;
            }
        }
    }
    
    // Wait for turning off the light
    if (m_nDeliverStatus == 1 || m_nDeliverStatus == 2) {
        fDeliverToSwitchOffInhibitor = 0.0;
    }

    CalcPositionAndOrientation(encoder);

    /* DEBUG */
    //printf("POS: X: %2f, %2f\r", m_vPosition.x, m_vPosition.y );
    /* DEBUG */

    /* Calc increment of position, correlating grid and metrics */
    double fXmov = mapLengthX / ((double) mapGridX);
    double fYmov = mapLengthY / ((double) mapGridY);

    /* Compute X grid */
    double tmp = m_vPosition.x;
    tmp += robotStartGridX * fXmov + 0.5 * fXmov;
    m_nRobotActualGridX = (int) (tmp / fXmov);

    /* Compute Y grid */
    tmp = -m_vPosition.y;
    tmp += robotStartGridY * fYmov + 0.5 * fYmov;
    m_nRobotActualGridY = (int) (tmp / fYmov);


    /* DEBUG */
    //printf("GRID: X: %d, %d\n", m_nRobotActualGridX, m_nRobotActualGridY);
    /* DEBUG */

    /* Update no-obstacles on map */
    if (onlineMap[m_nRobotActualGridX][m_nRobotActualGridY] != NEST &&
        onlineMap[m_nRobotActualGridX][m_nRobotActualGridY] != PREY)
        onlineMap[m_nRobotActualGridX][m_nRobotActualGridY] = NO_OBSTACLE;


    // we've found a nest
    if (ground[0] == 0) {
        /* Mark nest on map */
        onlineMap[m_nRobotActualGridX][m_nRobotActualGridY] = NEST;

        /* Flag that nest was found */
        m_nNestFound = 1;

        /* Update nest grid */
        m_nNestGridX = m_nRobotActualGridX;
        m_nNestGridY = m_nRobotActualGridY;

        /* DEBUG */
        printf("NEST HAS BEEN FOUND!\n");
        PrintMap(&onlineMap[0][0]);
        /* DEBUG */

        // update deliver status
        if (m_nDeliverStatus == 2) {
            m_nDeliverStatus = 3;
        }

    } else if (ground[0] == 0.5) {

        /* Mark prey on map */
        onlineMap[m_nRobotActualGridX][m_nRobotActualGridY] = PREY;

        dVector2 position;
        position.x = m_nRobotActualGridX;
        position.y = m_nRobotActualGridY;

        if (!IsPreyAlreadyChecked(&position)) {
            printf("------------------------ADDING PREY\n");
            m_nPreyFound = 1;
            m_vecPrey.push_back(position);
        }

        // update deliver status
        if (m_nDeliverStatus == 1 && !m_bIsInPrey &&
            (m_nPathPlanningDone == 0 || (m_nPathPlanningDone == 1 && SeemsToBeInTheRightPrey()))) {
            int objectInGround = m_seGround->GetObjectFromNearestGround();
            // if this prey has the right object, take it
            if (objectInGround == BLUE_OBJECT) {
                m_nDeliverStatus = 2;
                m_seGround->TakeObjectFromNearestGround();

                printf("Ground actually has the object\n");
                // Reset path planning to go now to the nest.
                m_nPathPlanningDone = 0;
                /* Restart PathPlanning state */
                m_nState = 0;
            } else {
                // Go to next prey if exists
                if (m_vecPreyNotChecked.size() > 0) {

                    m_vecPreyNotChecked.erase(m_vecPreyNotChecked.begin());
                    printf("removed, there are left %d\n", m_vecPreyNotChecked.size());

                    vector<dVector2>::iterator it = m_vecPreyNotChecked.begin();
                    while (it != m_vecPreyNotChecked.end()) {
                        printf("Prey position [%2.2f,%2.2f]\n", (it)->x, (it)->y);
                        it++;
                    }

                    m_nActualPreyGridX = m_vecPreyNotChecked.front().x;
                    m_nActualPreyGridY = m_vecPreyNotChecked.front().y;
                    /* Asumme Path Planning is done */
                    m_nPathPlanningDone = 0;
                    /* Restart PathPlanning state */
                    m_nState = 0;
                } else {
                    printf("Prey doesn't have the object and there are no more objects\n");
                    // We didn't find the correct prey so first try to find it.
                    m_nPreyFound = 0;
                }
            }
        }

        /* DEBUG */
        printf("PREY HAS BEEN FOUND!\n");
        PrintMap(&onlineMap[0][0]);
        /* DEBUG */

        // If ground == 0.5 then we're in prey.
        m_bIsInPrey = true;
    } else {
        m_bIsInPrey = false;
    }
}


bool CIri1Controller::IsPreyAlreadyChecked(dVector2 *preyPosition) {
    vector<dVector2>::iterator it = m_vecPrey.begin();
    printf("Size of vecPrey %d", m_vecPrey.size());

    while (it != m_vecPrey.end()) {

        /* Check the distance to the prey position */
        double distance = sqrt(pow((preyPosition->x - (it)->x), 2) + pow((preyPosition->y - (it)->y), 2));

        printf("Robot position [%2.2f,%2.2f], it [%2.2f,%2.2f]\n", preyPosition->x, preyPosition->y, (it)->x, (it)->y);
        printf("distance %2.2f, diameter %2.2f\n", distance, 2 * MAX_GROUND_RADIUS);

        /* If distance is higher than the ground's diameter */
        if (distance <= 2 * MAX_GROUND_RADIUS) {
            return true;
        }
        it++;
    }

    return false;
}

bool CIri1Controller::SeemsToBeInTheRightPrey() {
    double distance = sqrt(
            pow((m_nRobotActualGridX - m_nActualPreyGridX), 2) + pow((m_nRobotActualGridY - m_nActualPreyGridY), 2));
    return distance <= 2 * MAX_GROUND_RADIUS;
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::PathPlanning(unsigned int un_priority) {
    printf("Init PathPlanning\n");

    /* Clear Map */
    for (int y = 0; y < m; y++)
        for (int x = 0; x < n; x++)
            map[x][y] = NO_OBSTACLE;


    if (m_nNestFound == 1 && m_nPreyFound == 1 && m_nPathPlanningDone == 0 && (m_nDeliverStatus == 1 || m_nDeliverStatus == 2)) {
        printf("Doing path planning\n");
        m_nPathPlanningStops = 0;
        m_fActivationTable[un_priority][2] = 1;

        /* Obtain start and end desired position */
        int xA, yA, xB, yB;
        if (m_nDeliverStatus == 2) {
            xA = m_nRobotActualGridX;
            yA = m_nRobotActualGridY;
            xB = m_nNestGridX;
            yB = m_nNestGridY;
        } else /*if (m_nDeliverStatus == 1)*/
        {
            xA = m_nRobotActualGridX;
            yA = m_nRobotActualGridY;
            xB = m_nActualPreyGridX;
            yB = m_nActualPreyGridY;
        }

        /* DEBUG */
        printf("START: %d, %d - END: %d, %d\n", xA, yA, xB, yB);
        /* DEBUG */

        /* Obtain Map */
        for (int y = 0; y < m; y++)
            for (int x = 0; x < n; x++)
                if (onlineMap[x][y] != NO_OBSTACLE && onlineMap[x][y] != NEST && onlineMap[x][y] != PREY)
                    map[x][y] = OBSTACLE;


        /* Obtain optimal path */
        string route = pathFind(xA, yA, xB, yB);
        /* DEBUG */
        if (route == "") cout << "An empty route generated!" << endl;
        cout << "Route:" << route << endl;
        printf("route Length: %d\n", route.length());
        /* DEBUG */

        /* Obtain number of changing directions */
        for (int i = 1; i < route.length(); i++)
            if (route[i - 1] != route[i])
                m_nPathPlanningStops++;

        /* Add last movement */
        m_nPathPlanningStops++;
        /* DEBUG */
        printf("STOPS: %d\n", m_nPathPlanningStops);
        /* DEBUG */

        /* Define vector of desired positions. One for each changing direction */
        m_vPositionsPlanning = new dVector2[m_nPathPlanningStops];

        /* Calc increment of position, correlating grid and metrics */
        double fXmov = mapLengthX / mapGridX;
        double fYmov = mapLengthY / mapGridY;

        /* Get actual position */
        dVector2 actualPos;
        //actualPos.x = robotStartGridX * fXmov;
        actualPos.x = m_nRobotActualGridX * fXmov;
        //actualPos.y = robotStartGridY * fYmov;
        actualPos.y = m_nRobotActualGridY * fYmov;

        /* Fill vector of desired positions */
        int stop = 0;
        int counter = 0;
        /* Check the route and obtain the positions*/
        for (int i = 1; i < route.length(); i++) {
            /* For every position in route, increment countr */
            counter++;
            /* If a direction changed */
            if ((route[i - 1] != route[i])) {
                /* Obtain the direction char */
                char c;
                c = route.at(i - 1);

                /* Calc the new stop according to actual position and increment based on the grid */
                m_vPositionsPlanning[stop].x = actualPos.x + counter * fXmov * dx[atoi(&c)];
                m_vPositionsPlanning[stop].y = actualPos.y + counter * fYmov * dy[atoi(&c)];

                /* Update position for next stop */
                actualPos.x = m_vPositionsPlanning[stop].x;
                actualPos.y = m_vPositionsPlanning[stop].y;

                /* Increment stop */
                stop++;
                /* reset counter */
                counter = 0;
            }

            /* If we are in the last update, calc last movement */
            if (i == (route.length() - 1)) {
                /* Increment counter */
                counter++;
                /* Obtain the direction char */
                char c;
                c = route.at(i);

                /* DEBUG */
                //printf("COUNTER: %d, CHAR: %c\n", counter, c);
                /* END DEBUG */

                /* Calc the new stop according to actual position and increment based on the grid */
                m_vPositionsPlanning[stop].x =
                        actualPos.x + counter * fXmov * dx[atoi(&c)];// - robotStartGridX * fXmov;
                m_vPositionsPlanning[stop].y =
                        actualPos.y + counter * fYmov * dy[atoi(&c)];// - robotStartGridY * fYmov;

                /* Update position for next stop */
                actualPos.x = m_vPositionsPlanning[stop].x;
                actualPos.y = m_vPositionsPlanning[stop].y;

                /* Increment stop */
                stop++;
                /* reset counter */
                counter = 0;
            }

        }

        /* DEBUG */
        if (route.length() > 0) {
            int j;
            char c;
            int x = xA;
            int y = yA;
            map[x][y] = START;
            for (int i = 0; i < route.length(); i++) {
                c = route.at(i);
                j = atoi(&c);
                x = x + dx[j];
                y = y + dy[j];
                map[x][y] = PATH;
            }
            map[x][y] = END;

            PrintMap(&onlineMap[0][0]);
            printf("\n\n");
            PrintMap(&map[0][0]);
        }
        /* END DEBUG */

        /* DEBUG */
        //printf("Start: %2f, %2f\n", m_nRobotActualGridX * fXmov, m_nRobotActualGridY * fXmov);
        //for (int i = 0 ; i < m_nPathPlanningStops ; i++)
        //printf("MOV %d: %2f, %2f\n", i, m_vPositionsPlanning[i].x, m_vPositionsPlanning[i].y);
        /* END DEBUG */

        /* Convert to simulator coordinates */
        for (int i = 0; i < m_nPathPlanningStops; i++) {
            m_vPositionsPlanning[i].x -= (mapGridX * fXmov) / 2;
            m_vPositionsPlanning[i].y -= (mapGridY * fYmov) / 2;
            m_vPositionsPlanning[i].y = -m_vPositionsPlanning[i].y;
        }
        /* DEBUG */
        //for (int i = 0 ; i < m_nPathPlanningStops ; i++)
        //printf("MOV %d: %2f, %2f\n", i, m_vPositionsPlanning[i].x, m_vPositionsPlanning[i].y);
        /* END DEBUG */


        /* Convert to robot coordinates. FAKE!!.
         * Notice we are only working with initial orientation = 0.0 */
        for (int i = 0; i < m_nPathPlanningStops; i++) {
            /* Traslation */
            m_vPositionsPlanning[i].x += ((robotStartGridX * fXmov) - (mapGridX * fXmov) / 2);
            m_vPositionsPlanning[i].y += ((robotStartGridY * fXmov) - (mapGridY * fYmov) / 2);
            /* Rotation */
            //double compass = m_pcEpuck->GetRotation();
            //m_vPositionsPlanning[i].x = m_vPositionsPlanning[i].x * cos (compass) - m_vPositionsPlanning[i].y  * sin(compass);
            //m_vPositionsPlanning[i].y = m_vPositionsPlanning[i].x * sin (compass) + m_vPositionsPlanning[i].y  * cos(compass);
        }
        /* DEBUG */
        //for (int i = 0 ; i < m_nPathPlanningStops ; i++)
        //printf("MOV %d: %2f, %2f\n", i, m_vPositionsPlanning[i].x, m_vPositionsPlanning[i].y);
        /* END DEBUG */

        m_nPathPlanningDone = 1;
    }
}


/******************************************************************************/
/******************************************************************************/

void CIri1Controller::GoGetOrDeliverObject(unsigned int un_priority) {
    if (((m_nNestFound * fBattToForageInhibitor * fMoneyToForageInhibitor) == 1) &&
        ((m_nPreyFound * fBattToForageInhibitor * fMoneyToForageInhibitor) == 1) &&
        (m_nDeliverStatus > 0 && m_nDeliverStatus < 3)) {

        /* If something not found at the end of planning, reset plans */
        if (m_nState >= m_nPathPlanningStops) {
            m_nNestFound = 0;
            m_nPreyFound = 0;
            m_nPathPlanningDone = 0;
            m_nState = 0;
            m_vecPrey.clear();
            m_vecPreyNotChecked.clear();
            return;
        }

        /* DEBUG */
        printf("PlanningX: %2f, Actual: %2f\n", m_vPositionsPlanning[m_nState].x, m_vPosition.x);
        printf("PlanningY: %2f, Actual: %2f\n", m_vPositionsPlanning[m_nState].y, m_vPosition.y);
        /* DEBUG */

        double fX = (m_vPositionsPlanning[m_nState].x - m_vPosition.x);
        double fY = (m_vPositionsPlanning[m_nState].y - m_vPosition.y);
        double fGoalDirection = 0;

        /* If on Goal, return 1 */
        if ((fabs(fX) <= ERROR_POSITION) && (fabs(fY) <= ERROR_POSITION))
            m_nState++;

        fGoalDirection = atan2(fY, fX);

        /* Translate fGoalDirection into local coordinates */
        fGoalDirection -= m_fOrientation;
        /* Normalize Direction */
        while (fGoalDirection > M_PI) fGoalDirection -= 2 * M_PI;
        while (fGoalDirection < -M_PI) fGoalDirection += 2 * M_PI;

        m_fActivationTable[un_priority][0] = fGoalDirection;
        m_fActivationTable[un_priority][1] = 1;
        m_fActivationTable[un_priority][2] = 1;
    }
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::CalcPositionAndOrientation(double *f_encoder) {
    /* DEBUG */
    //printf("Encoder: %2f, %2f\n", f_encoder[0], f_encoder[1]);
    /* DEBUG */

    /* Remake kinematic equations */
    double fIncU = (f_encoder[0] + f_encoder[1]) / 2;
    double fIncTetha = (f_encoder[1] - f_encoder[0]) / CEpuck::WHEELS_DISTANCE;

    /* Substitute arc by chord, take care of 0 division */
    if (fIncTetha != 0.0)
        fIncU = ((f_encoder[0] / fIncTetha) + (CEpuck::WHEELS_DISTANCE / 2)) * 2.0 * sin(fIncTetha / 2.0);

    /* Update new Position */
    m_vPosition.x += fIncU * cos(m_fOrientation + fIncTetha / 2);
    m_vPosition.y += fIncU * sin(m_fOrientation + fIncTetha / 2);

    /* Update new Orientation */
    m_fOrientation += fIncTetha;

    /* Normalize Orientation */
    while (m_fOrientation < 0) m_fOrientation += 2 * M_PI;
    while (m_fOrientation > 2 * M_PI) m_fOrientation -= 2 * M_PI;
}

/******************************************************************************/
/******************************************************************************/

// A-star algorithm.
// The route returned is a string of direction digits.
string CIri1Controller::pathFind(const int &xStart, const int &yStart,
                                 const int &xFinish, const int &yFinish) {
    static priority_queue <node> pq[2]; // list of open (not-yet-tried) nodes
    static int pqi; // pq index
    static node *n0;
    static node *m0;
    static int i, j, x, y, xdx, ydy;
    static char c;
    pqi = 0;

    // reset the node maps
    for (y = 0; y < m; y++) {
        for (x = 0; x < n; x++) {
            closed_nodes_map[x][y] = 0;
            open_nodes_map[x][y] = 0;
        }
    }

    // create the start node and push into list of open nodes
    n0 = new node(xStart, yStart, 0, 0);
    n0->updatePriority(xFinish, yFinish);
    pq[pqi].push(*n0);
    //open_nodes_map[x][y]=n0->getPriority(); // mark it on the open nodes map

    // A* search
    while (!pq[pqi].empty()) {
        // get the current node w/ the highest priority
        // from the list of open nodes
        n0 = new node(pq[pqi].top().getxPos(), pq[pqi].top().getyPos(),
                      pq[pqi].top().getLevel(), pq[pqi].top().getPriority());

        x = n0->getxPos();
        y = n0->getyPos();

        pq[pqi].pop(); // remove the node from the open list
        open_nodes_map[x][y] = 0;
        // mark it on the closed nodes map
        closed_nodes_map[x][y] = 1;

        // quit searching when the goal state is reached
        //if((*n0).estimate(xFinish, yFinish) == 0)
        if (x == xFinish && y == yFinish) {
            // generate the path from finish to start
            // by following the directions
            string path = "";
            while (!(x == xStart && y == yStart)) {
                j = dir_map[x][y];
                c = '0' + (j + dir / 2) % dir;
                path = c + path;
                x += dx[j];
                y += dy[j];
            }

            // garbage collection
            delete n0;
            // empty the leftover nodes
            while (!pq[pqi].empty()) pq[pqi].pop();
            return path;
        }

        // generate moves (child nodes) in all possible directions
        for (i = 0; i < dir; i++) {
            xdx = x + dx[i];
            ydy = y + dy[i];

            if (!(xdx < 0 || xdx > n - 1 || ydy < 0 || ydy > m - 1 || map[xdx][ydy] == 1
                  || closed_nodes_map[xdx][ydy] == 1)) {
                // generate a child node
                m0 = new node(xdx, ydy, n0->getLevel(),
                              n0->getPriority());
                m0->nextLevel(i);
                m0->updatePriority(xFinish, yFinish);

                // if it is not in the open list then add into that
                if (open_nodes_map[xdx][ydy] == 0) {
                    open_nodes_map[xdx][ydy] = m0->getPriority();
                    pq[pqi].push(*m0);
                    // mark its parent node direction
                    dir_map[xdx][ydy] = (i + dir / 2) % dir;
                } else if (open_nodes_map[xdx][ydy] > m0->getPriority()) {
                    // update the priority info
                    open_nodes_map[xdx][ydy] = m0->getPriority();
                    // update the parent direction info
                    dir_map[xdx][ydy] = (i + dir / 2) % dir;

                    // replace the node
                    // by emptying one pq to the other one
                    // except the node to be replaced will be ignored
                    // and the new node will be pushed in instead
                    while (!(pq[pqi].top().getxPos() == xdx &&
                             pq[pqi].top().getyPos() == ydy)) {
                        pq[1 - pqi].push(pq[pqi].top());
                        pq[pqi].pop();
                    }
                    pq[pqi].pop(); // remove the wanted node

                    // empty the larger size pq to the smaller one
                    if (pq[pqi].size() > pq[1 - pqi].size()) pqi = 1 - pqi;
                    while (!pq[pqi].empty()) {
                        pq[1 - pqi].push(pq[pqi].top());
                        pq[pqi].pop();
                    }
                    pqi = 1 - pqi;
                    pq[pqi].push(*m0); // add the better node instead
                } else delete m0; // garbage collection
            }
        }
        delete n0; // garbage collection
    }
    return ""; // no route found
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::PrintMap(int *print_map) {

    /* DEBUG */
    for (int x = 0; x < n; x++) {
        for (int y = 0; y < m; y++) {
            if (print_map[y * n + x] == 0)
                cout << ".";
            else if (print_map[y * n + x] == 1)
                cout << "O"; //obstacle
            else if (print_map[y * n + x] == 2)
                cout << "S"; //start
            else if (print_map[y * n + x] == 3)
                cout << "R"; //route
            else if (print_map[y * n + x] == 4)
                cout << "F"; //finish
            else if (print_map[y * n + x] == 5)
                cout << "N"; //finish
            else if (print_map[y * n + x] == 6)
                cout << "P"; //finish
        }
        cout << endl;
    }
    /* DEBUG */
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::SaveStats(int *print_map) {

    int clearDiscovered = 0,
            preyDiscovered = 0,
            nestDiscovered = 0,
            totalDiscovered = 0,
            objects = 0;

    for ( int x = 0 ; x < n ; x++ )
    {
        for ( int y = 0 ; y < m ; y++ )
        {
            if ( print_map[y*n + x] == 0 ) {
                clearDiscovered++;
            } else if(print_map[y*n+x]==5) {
                nestDiscovered++;
            } else if(print_map[y*n+x]==6) {
                preyDiscovered++;
            }
        }
    }
    totalDiscovered = nestDiscovered + preyDiscovered + clearDiscovered;
    objects = n * m - totalDiscovered;
    FILE* fileOutput = fopen("outputFiles/randomMovementOutput", "a");
    fprintf(fileOutput, "%1.4f %u %u %u %u %u %u %u\n", RANDOM_NAVIGATION, m_nStepTime, totalDiscovered, objects,
            clearDiscovered, preyDiscovered, nestDiscovered, m_nPacketsAttended);
    fclose(fileOutput);
}