#include "redbatterysensor.h"

/******************************************************************************/
/******************************************************************************/

CRedBatterySensor::CRedBatterySensor(const char* pch_name, double f_range, double f_charge_coef, double f_discharge_coef) :
    CSensor(pch_name, NUM_RED_BATTERY_SENSORS )
{
	m_fBatteryLevel = 1.0;
	SetInput(0,m_fBatteryLevel);

	m_fRange = f_range;
	m_fChargeCoef = f_charge_coef;
	m_fDischargeCoef = f_discharge_coef;
}

/******************************************************************************/
/******************************************************************************/

CRedBatterySensor::~CRedBatterySensor()
{
}

/******************************************************************************/
/******************************************************************************/

double* CRedBatterySensor::ComputeSensorReadings(CEpuck* pc_epuck, CSimulator* pc_simulator){
	
	/* Get Position of the epuck */
	dVector2 vPosition;
	vPosition.x = (pc_epuck->GetPosition()).x; 
	vPosition.y = (pc_epuck->GetPosition()).y;
	double fOrientation;
	fOrientation = (pc_epuck->GetRotation());
	
	/* Get Arena */
	CArena* pcArena = pc_simulator->GetArena();

	dVector2 vLightPosition;
	double fDistance;
	/* If there is lights charge battery */
	if ( pcArena->GetNearestRedLight(vPosition, m_fRange , &vLightPosition, &fDistance))
	{
			m_fBatteryLevel += m_fChargeCoef * ( 1 - pow ( m_fBatteryLevel , 2 ));
	}
	else
	{
		/* discharge battery */
		m_fBatteryLevel -= m_fDischargeCoef;
	}

	/* Limit signal */
	if ( m_fBatteryLevel < 0.0 ) m_fBatteryLevel = 0.0;
	if ( m_fBatteryLevel > 1.0 ) m_fBatteryLevel = 1.0;

	SetInput (0,m_fBatteryLevel);
}

/******************************************************************************/
/******************************************************************************/

double* CRedBatterySensor::GetSensorReading( CEpuck *p_pcEpuck){
	return GetInputs();
}

/******************************************************************************/
/******************************************************************************/

double CRedBatterySensor::GetBatteryLevel( void ){
	return m_fBatteryLevel;
}
/******************************************************************************/
/******************************************************************************/

unsigned int CRedBatterySensor::GetType(){
	return SENSOR_RED_BATTERY;
}

/******************************************************************************/
/******************************************************************************/

void CRedBatterySensor::Reset()
{
	m_fBatteryLevel = 1.0;
	SetInput(0,m_fBatteryLevel);
}
