#include <math.h>

#include "loadfitnessfunction.h"
#include "collisionmanager.h"

/******************************************************************************/
/******************************************************************************/

CLoadFitnessFunction::CLoadFitnessFunction(const char *pch_name,
                                           CSimulator *pc_simulator,
                                           unsigned int un_collisions_allowed_per_epuck)
        :
        CFitnessFunction(pch_name, pc_simulator) {

    /* Check number of robots */
    m_pcSimulator = pc_simulator;
    TEpuckVector *pvecEpucks = m_pcSimulator->GetEpucks();

    if (pvecEpucks->size() == 0) {
        printf("No Robot, so fitness function can not be computed.\n Exiting...\n");
        fflush(stdout);
        exit(0);
    } else if (pvecEpucks->size() > 1) {
        printf("More than 1 robot, and fitness is not prepared for it.\n Exiting...\n");
    }

    m_pcEpuck = (*pvecEpucks)[0];

    m_unNumberOfSteps = 0;
    m_fComputedFitness = 0.0;

    m_unCollisionsNumber = 0;
}

/******************************************************************************/
/******************************************************************************/

CLoadFitnessFunction::~CLoadFitnessFunction() {
}
/******************************************************************************/
/******************************************************************************/

double CLoadFitnessFunction::GetFitness() {

    //printf("COLL: %2d\n",CCollisionManager::GetInstance()->GetTotalNumberOfCollisions());
    //fflush(stdout);
    //int coll = (CCollisionManager::GetInstance()->GetTotalNumberOfCollisions());

//    double fit = ((m_fComputedFitness / (double) m_unNumberOfSteps) *
//                  (1 - ((double) (fmin(m_unCollisionsNumber, 10.0) / 10.0))));
    double fit = (m_fComputedFitness / (double) m_unNumberOfSteps);
    if (fit < 0.0) fit = 0.0;

    return fit;
}

/******************************************************************************/
/******************************************************************************/
void CLoadFitnessFunction::SimulationStep(unsigned int n_simulation_step, double f_time, double f_step_interval) {

    /* Maximize movement as in avoidfitnessfunction */
    double leftSpeed = 0.0;
    double rightSpeed = 0.0;
    m_pcEpuck->GetWheelSpeed(&leftSpeed, &rightSpeed);
    leftSpeed = 0.5 + (leftSpeed / (2.0 * m_pcEpuck->GetMaxWheelSpeed()));
    rightSpeed = 0.5 + (rightSpeed / (2.0 * m_pcEpuck->GetMaxWheelSpeed()));

    /* Eval maximum speed partial fitness */
    double maxSpeedEval = (fabs(leftSpeed - 0.5) + fabs(rightSpeed - 0.5));

    /* Eval same directio partial fitness */
    double sameDirectionEval = 1 - sqrt(fabs(leftSpeed - rightSpeed));

    /* Eval minimum sensor reading partial fitness */
    double maxProxSensorEval = 0.0;
    double maxLightSensorEval = 0.0;
    double maxBlueLightSensorEval = 0.0;
    double maxRedLightSensorEval = 0.0;
    double maxContactSensorEval = 0.0;
    double lightRange = 0.0;
    double light0 = 0.0;
    double light7 = 0.0;

    TSensorVector vecSensors = m_pcEpuck->GetSensors();

    double *groundMemory;
    double *ground;
    double *battery;
    double *blueBattery;
    double *redBattery;
    unsigned int unThisSensorsNumberOfInputs;
    double *pfThisSensorInputs;

    for (TSensorIterator i = vecSensors.begin(); i != vecSensors.end(); i++) {

        switch ((*i)->GetType()) {
            case SENSOR_BATTERY:
                battery = (*i)->GetComputedSensorReadings();
                break;

            case SENSOR_BLUE_BATTERY:
                blueBattery = (*i)->GetComputedSensorReadings();
                break;

            case SENSOR_RED_BATTERY:
                redBattery = (*i)->GetComputedSensorReadings();
                break;

            case SENSOR_GROUND_MEMORY:
                groundMemory = (*i)->GetComputedSensorReadings();
                break;

            case SENSOR_GROUND:
                ground = (*i)->GetComputedSensorReadings();
                break;

            case SENSOR_LIGHT:
                unThisSensorsNumberOfInputs = (*i)->GetNumberOfInputs();
                pfThisSensorInputs = (*i)->GetComputedSensorReadings();

                for (int j = 0; j < unThisSensorsNumberOfInputs; j++) {
                    if (pfThisSensorInputs[j] > maxLightSensorEval) {
                        maxLightSensorEval = pfThisSensorInputs[j];
                    }
                    if (j == 0)
                        light0 = pfThisSensorInputs[j];
                    else if (j == 7)
                        light7 = pfThisSensorInputs[j];
                }
                break;

            case SENSOR_BLUE_LIGHT:
                unThisSensorsNumberOfInputs = (*i)->GetNumberOfInputs();
                pfThisSensorInputs = (*i)->GetComputedSensorReadings();

                for (int j = 0; j < unThisSensorsNumberOfInputs; j++) {
                    if (pfThisSensorInputs[j] > maxBlueLightSensorEval) {
                        maxBlueLightSensorEval = pfThisSensorInputs[j];
                    }
                }
                break;

            case SENSOR_RED_LIGHT:
                unThisSensorsNumberOfInputs = (*i)->GetNumberOfInputs();
                pfThisSensorInputs = (*i)->GetComputedSensorReadings();

                for (int j = 0; j < unThisSensorsNumberOfInputs; j++) {
                    if (pfThisSensorInputs[j] > maxRedLightSensorEval) {
                        maxRedLightSensorEval = pfThisSensorInputs[j];
                    }
                }
                break;

            case SENSOR_PROXIMITY:
                unThisSensorsNumberOfInputs = (*i)->GetNumberOfInputs();
                pfThisSensorInputs = (*i)->GetComputedSensorReadings();

                for (int j = 0; j < unThisSensorsNumberOfInputs; j++) {
                    if (pfThisSensorInputs[j] > maxProxSensorEval) {
                        maxProxSensorEval = pfThisSensorInputs[j];
                    }
                }
                break;

            case SENSOR_CONTACT:
                unThisSensorsNumberOfInputs = (*i)->GetNumberOfInputs();
                pfThisSensorInputs = (*i)->GetComputedSensorReadings();

                for (int j = 0; j < unThisSensorsNumberOfInputs; j++) {
                    if (pfThisSensorInputs[j] > maxContactSensorEval) {
                        maxContactSensorEval = pfThisSensorInputs[j];
                    }
                }
                break;
        }
    }

//    double coef1 = 1.0/4.0;
//    double coef2 = 3.0/4.0;
//
//    double coef3 = 1 - coef1 - coef2;
//
//    //double fitness = 0.0;
//    //fitness = coef1 * ( maxSpeedEval * sameDirectionEval * (1 - maxProxSensorEval) * (leftSpeed * rightSpeed ) ) + coef2 * ( battery[0] ) + coef3 * ( maxLightSensorEval );
////    fitness = coef1 * maxSpeedEval * sameDirectionEval * (1 - maxProxSensorEval) +
////              coef2 * battery[0] * (light0 + light7);
//
//    //printf("BATTERY: %2.4f\n", battery[0]);
//
//    /* Bat Exp 1 */
//    double fitness = maxSpeedEval * sameDirectionEval * (leftSpeed * rightSpeed);
//
//    if (battery[0] > 0.7 )
//    fitness *= (1 - maxProxSensorEval);
//    else
//    fitness *= (maxLightSensorEval);

    double th = 0.7;
//    double sigmoid = ((10.0 * (battery[0] - th)) / (1.0 + 10.0 * abs(battery[0] - 0.7)) + 1.0) * 0.57;
    double fitness = 0.0;
    double sigmoid = 1 / (1 + exp(-(battery[0] - 0.7) * 50));
    double coef1 = 0.25;
    double coef2 = 0.75;
//    fitness = sigmoid * (maxSpeedEval * sameDirectionEval * (1 - maxProxSensorEval) * (leftSpeed * rightSpeed)) +
//              (1 - sigmoid) *
//              (maxSpeedEval * sameDirectionEval * (1 - maxProxSensorEval) * (leftSpeed * rightSpeed) * maxLightSensorEval);
    fitness = sigmoid * (maxSpeedEval * sameDirectionEval * (1 - maxProxSensorEval) * (leftSpeed * rightSpeed)) +
              (1 - sigmoid) *
              (coef1 * (maxSpeedEval * sameDirectionEval * (1 - maxProxSensorEval) * (leftSpeed * rightSpeed))) +
              (coef2 * (maxLightSensorEval));
//    fitness = sigmoid * (maxSpeedEval * sameDirectionEval * (1 - maxProxSensorEval) * (leftSpeed * rightSpeed)) +
//              (1 - sigmoid) *
//              (coef1 * (maxSpeedEval * sameDirectionEval * (1 - maxProxSensorEval) * (leftSpeed * rightSpeed))) +
//              (coef2 * (maxLightSensorEval));
    /* END Bat Exp 1 */


    m_unNumberOfSteps++;
    m_fComputedFitness += fitness;

    /* Get Collisions */
    int nContact = 0;
    CContactSensor *m_seContact = (CContactSensor *) m_pcEpuck->GetSensor(SENSOR_CONTACT);
    double *contact = m_seContact->GetSensorReading(m_pcEpuck);
    for (int j = 0; j < m_seContact->GetNumberOfInputs(); j++) {
        if (contact[j] > 0.0)
            nContact = 1;
    }

    if (nContact == 1)
        m_unCollisionsNumber++;
}

/******************************************************************************/
/******************************************************************************/
