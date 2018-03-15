#include "testwheel.h"

/******************************************************************************/
/******************************************************************************/

CTestWheel::CTestWheel(const char* pch_name, CEpuck* pc_epuck) :
    CActuator(pch_name, pc_epuck, 2)
{
	printf("Test wheels actuator instantiated\n");
	fflush(stdout);
}

/******************************************************************************/
/******************************************************************************/

unsigned int CTestWheel::GetType()
{
    return ACTUATOR_WHEELS_TEST;
}

/******************************************************************************/
/******************************************************************************/

void CTestWheel::SetSpeed(double left, double right)
{
	
  if( left <= MAX_REAL_SPEED && left >= MIN_REAL_SPEED
      && right <= MAX_REAL_SPEED && right >= MIN_REAL_SPEED )
  {
    double realLeftSpeed  = left * 0.00012987;
    double realRightSpeed = right * 0.00012987;
    CEpuck* pcEpuck = GetEpuck();
    pcEpuck->SetWheelSpeed(realLeftSpeed, realRightSpeed);
  }
  else
    printf("ERROR : You are going out of bounds for the wheels speed\n");
}

/******************************************************************************/
/******************************************************************************/

void CTestWheel::SetOutput(unsigned int un_input_index, double f_value)
{
    CEpuck* pcEpuck = GetEpuck();
    //double fNewSpeed = 2.0f * (f_value - 0.50000000) * pcEpuck->GetMaxWheelSpeed();
	//HEre i compute the wheel speed (range -1000 , +1000)
//	printf("Wheel %d, network value: %f\n",un_input_index,f_value);
	double fNewSpeed = 2.0f * (f_value - 0.50000000) * MAX_REAL_SPEED;
	//printf("Wheel %d, actuated value: %f\n",un_input_index,fNewSpeed);
	//fflush(stdout);
	//sleep(1);
	/**/
	fNewSpeed*=CEpuck::MAX_EPUCK_SPEED/MAX_REAL_SPEED;
	
	//Add some noise
	float fNoise=Random::nextDouble(-0.015*CEpuck::MAX_EPUCK_SPEED,0.015*CEpuck::MAX_EPUCK_SPEED);
	fNewSpeed+=fNoise;
	/*
		printf("Wheel %d, speed value: %f\n",un_input_index,fNewSpeed);
	fflush(stdout);
	sleep(1);
	/**/
	if(fNewSpeed<-CEpuck::MAX_EPUCK_SPEED){
		fNewSpeed=-CEpuck::MAX_EPUCK_SPEED;
	}else if(fNewSpeed>CEpuck::MAX_EPUCK_SPEED){
		fNewSpeed=CEpuck::MAX_EPUCK_SPEED;
	}
	
    double fLeftSpeed;
    double fRightSpeed;

    pcEpuck->GetWheelTargetSpeed(&fLeftSpeed, &fRightSpeed);
	
    if (un_input_index == 0)
        fLeftSpeed = fNewSpeed;
    else
        fRightSpeed = fNewSpeed;
    pcEpuck->SetWheelSpeed(fLeftSpeed, fRightSpeed);
}

/******************************************************************************/
/******************************************************************************/
