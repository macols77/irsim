#include "wheelsactuator.h"

/******************************************************************************/
/******************************************************************************/

CWheelsActuator::CWheelsActuator(const char* pch_name, CEpuck* pc_epuck) :
    CActuator(pch_name, pc_epuck, 2)
{

}

/******************************************************************************/
/******************************************************************************/

unsigned int CWheelsActuator::GetType()
{
    return ACTUATOR_WHEELS;
}

/******************************************************************************/
/******************************************************************************/

void CWheelsActuator::SetSpeed(double left, double right)
{

	CEpuck* pcEpuck = GetEpuck();

  /* Saturation */
  if (left > MAX_REAL_SPEED)
    left = MAX_REAL_SPEED;
  if ( left < MIN_REAL_SPEED )
    left = MIN_REAL_SPEED;
  
  if ( right > MAX_REAL_SPEED)
    right = MAX_REAL_SPEED;
  if ( right < MIN_REAL_SPEED )
    right = MIN_REAL_SPEED;


  //if( left <= MAX_REAL_SPEED && left >= MIN_REAL_SPEED
  //&& right <= MAX_REAL_SPEED && right >= MIN_REAL_SPEED )
  //{
  
		//double realLeftSpeed  = left * 0.0001288053;
		//double realRightSpeed = right * 0.0001288053
		
		double realLeftSpeed  = ( left * pcEpuck->GetMaxWheelSpeed() ) / MAX_REAL_SPEED;
		double realRightSpeed  = ( right * pcEpuck->GetMaxWheelSpeed() ) / MAX_REAL_SPEED;
    pcEpuck->SetWheelSpeed(realLeftSpeed, realRightSpeed);

    //}
    //else
    //printf("ERROR : You are going out of bounds for the wheels speed\n");
}

/******************************************************************************/
/******************************************************************************/

void CWheelsActuator::SetOutput(unsigned int un_input_index, double f_value)
{
	/* Saturation */
	if (f_value > 1.0 ) 
		f_value = 1.0;
	if (f_value < 0.0 ) 
		f_value = 0.0;

	CEpuck* pcEpuck = GetEpuck();
	double fNewSpeed = 2.0f * (f_value - 0.50000000) * pcEpuck->GetMaxWheelSpeed();

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
