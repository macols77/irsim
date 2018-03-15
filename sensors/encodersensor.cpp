#include "encodersensor.h"

/******************************************************************************/
/******************************************************************************/

CEncoderSensor::CEncoderSensor(const char* pch_name,CArena* pc_arena, float encoder_error, float initX, float initY) :
    CSensor(pch_name, NUM_ENCODER_SENSORS )
{
	m_pcArena=pc_arena;
	m_fEncoderError = encoder_error;
  m_vLastPosition.x = initX;
  m_vLastPosition.y = initY;

}

/******************************************************************************/
/******************************************************************************/

CEncoderSensor::~CEncoderSensor()
{
}

/******************************************************************************/
/******************************************************************************/

double* CEncoderSensor::ComputeSensorReadings(CEpuck* p_pcEpuck, CSimulator* p_pcSimulator){
	///* Get Actual Position of the epuck */
	//dVector2 vPosition;
	//vPosition.x = (p_pcEpuck->GetPosition()).x; 
	//vPosition.y = (p_pcEpuck->GetPosition()).y;

	///* Get increment of the Position */
	//dVector2 vIncPosition;
	//vIncPosition.x = vPosition.x - m_vLastPosition.x;
	//vIncPosition.y = vPosition.y - m_vLastPosition.y;

  ////printf("INITX: %2f, INITY: %2f, ACTX: %2f, ACTY: %2f\n", m_vLastPosition.x, m_vLastPosition.y, vPosition.x, vPosition.y);
  ////printf("INCX: %2f, INCY: %2f\n", vIncPosition.x, vIncPosition.y);

	///* Get actual orientation of the robot */
	//float fOrientation = p_pcEpuck->GetRotation();
	///* Get increment of the orientation */
	//float fIncOrientation = fOrientation - m_fLastOrientation;

	///* Normalize Robot Orientation Increment */
	//while(fIncOrientation > M_PI) fIncOrientation -= 2 * M_PI;
	//while(fIncOrientation < - M_PI) fIncOrientation += 2 * M_PI;

	///* Define Encoder increments, and Angular Position Increments */
	//float fIncSLeft, fIncSRight;
	////float fIncAngularPosLeft, fIncAngularPosRight;

	///* Brief explanation on kinematics movement */
	///* 
	 //* The IncTheta ( Angular movement ) of the robot is given by: 
	 //*   IncTheta = ( IncSr - IncSl ) / b, 
	 //*   
	 //*   where IncSr is the increment of the Right wheel encoder in standard units
	 //*   where IncSl is the increment of the Left wheel encoder in standard units
	 //*   where b is the distance between the 2 ideal points of connection of the wheels and the floor.
	 //*   In this case is the Cepuck::WHEELS_DISTANCE, defined in footbot_entity.h in
	 //*   simulator/swarmanoid_space/entities.
	 //*
	 //* The IncS ( Linear movement ) of the robot is given by:
	 //*  IncS = ( IncSr + IncSl ) / 2
	 //*
   //*
   //* Then
   //* IncSr = (2 * IncS + b * IncTheta) /2
   //* IncSl = (2 * IncS - b * IncTheta) /2
	 //*
	 //*  Xi = Xi-1 + IncS * cos(Thetai-1 + IncTheta/2);
	 //*  Yi = Yi-1 + IncS * sin(Thetai-1 + IncTheta/2);
	 //*  Thetai = Thetai-1 + IncTheta;
	 //*
	 //*  This ecuations should be manage in a inverse way to obtain the encoder movement of the robot
	 //*
	 //*  Special cases must be manage when Robot Turning on himself on when Robot moving in a straight line.
	 //*
	 //*  */

	/////* If Robot turning over himself each wheel goes in a sense with the same increment 
	 ////*      * So IncSR = - IncSl --> IncS = 0 and IncTheta = 2 * IncSl / b */
  ////if(DoubleEq(vIncPosition.x,0.0) && DoubleEq(vIncPosition.y,0.0))
    //////if(vIncPosition.x <= 0.00000001  && vIncPosition.y <= 0.00000001)
  ////{
    //////printf("A\n");
		/////* Inc x = 0 && Inc y = 0 --> Robot turns over his body */

		/////* Calc linear movement of each wheel */
		////fIncSLeft = -fIncOrientation * CEpuck::WHEELS_DISTANCE / 2;
		////fIncSRight = -fIncSLeft;

		/////* Calc angular movement of each wheel */
		//////fIncAngularPosLeft = fIncSLeft / CFootBotEntity::FOOTBOT_WHEEL_RADIUS;
		//////fIncAngularPosRight = fIncSRight / CFootBotEntity::FOOTBOT_WHEEL_RADIUS;
	////}
	
	/////* If Robot going straight a head IncTheta = 0*/
	////else if(DoubleEq(fOrientation,m_fLastOrientation)){
    //////printf("B\n");
		/////* fIncU = IncS */
		////double fIncU = 0;
		/////* IncX = IncS * cos (Thetai-1 + 0) */
		/////* IncY = IncS * sin (Thetai-1 + 0) */
		/////* Calc IncS */
		////if(DoubleEq(vPosition.x,0)) fIncU = vIncPosition.y/sin(fOrientation);
		////else fIncU = vIncPosition.x/cos(fOrientation);

		/////* Calc IncSl and IncSr. 
		 ////* Because IncTheta = 0, IncSr = IncSl 
		 ////* IncS = ( IncSr + IncSl ) / 2 = IncSr = IncSl */
		////fIncSLeft = fIncU;
		////fIncSRight = fIncU;

		/////* Calc angular movement of each wheel */
		//////fIncAngularPosLeft = fIncSLeft / CFootBotEntity::FOOTBOT_WHEEL_RADIUS;
		//////fIncAngularPosRight = fIncAngularPosLeft;

	////}
	/////* If doing an arc as movement 
	 ////* NOTE: Take into account that in this case robot movement is approximated by a strait line. 
	 ////* It will be the cord of the arc made, that it's enough if Inct --> 0, so if big time steps
	 ////* approximation will no be valid, but there is not other way to do it*/
	////else{
    ////////printf("C\n");
    //////printf("INCX: %2f, INCY: %2f\n", vIncPosition.x, vIncPosition.y);
		///////* fIncU = IncS */
		//////double fIncU = 0;
		///////* IncX = IncS * cos (Thetai-1 + IncTheta/2) */
		///////* IncY = IncS * sin (Thetai-1 + IncTheta/2) */
		///////* Calc IncS */
		//////if(DoubleEq(vPosition.x,0)) 
      //////fIncU = vIncPosition.y/sin(fOrientation) + fIncOrientation / 2;
		//////else 
      //////fIncU = vIncPosition.x/cos(fOrientation) + fIncOrientation /2;

		///////* Calc IncSl and IncSr */
		///////* IncS = (IncSr + IncSl) / 2 --> 2 * IncS = IncSr + IncSl
		 //////* IncTheta = (IncSr - IncSl ) / b --> b * IncTheta = IncSr - IncSl
		 //////* then
		 //////* IncSr = IncS + b/2 * IncTheta
		 //////* IncSl = IncS - b/2 * IncTheta */
		//////fIncSLeft = fIncU - CEpuck::WHEELS_DISTANCE * fIncOrientation / 2;
		//////fIncSRight = fIncU + CEpuck::WHEELS_DISTANCE * fIncOrientation / 2;
	
    /////* IncSr - IncSl = b * IncTheta
     ////* IncSr + IncSl = 2 * IncS
     ////* then
     ////* IncSr = (2 * IncS + b * IncTheta)/2
     ////* IncSl = (2 * IncS - b * IncTheta)/2 */
    ////double fIncS = sqrt(vIncPosition.x * vIncPosition.x + vIncPosition.y * vIncPosition.y);

    ////fIncSLeft = (2 * fIncS - CEpuck::WHEELS_DISTANCE * fIncOrientation ) / 2;
		////fIncSRight = (2 * fIncS + CEpuck::WHEELS_DISTANCE * fIncOrientation) / 2;

		/////* Calc Angular movement of each whell */
		//////fIncAngularPosLeft = fIncSLeft / CFootBotEntity::FOOTBOT_WHEEL_RADIUS;
		//////fIncAngularPosRight = fIncSRight / CFootBotEntity::FOOTBOT_WHEEL_RADIUS;
	////}
    
  //double fIncS = sqrt(vIncPosition.x * vIncPosition.x + vIncPosition.y * vIncPosition.y);

  ////Chord is r * crd (tetha) = r * 2 * sin (tetha/2)
  //if (fIncOrientation != 0.0)
    //fIncS = ( fIncS * fIncOrientation ) / (2 * sin (fIncOrientation/2));

  ////printf("fIncS: %2f\n", fIncS);
  //fIncSLeft = (2 * fIncS - CEpuck::WHEELS_DISTANCE * fIncOrientation ) / 2;
  //fIncSRight = (2 * fIncS + CEpuck::WHEELS_DISTANCE * fIncOrientation) / 2;

	///* If Param noise on and there is movement on the wheels, add noise */
	////if(DoubleEq(m_fNoise,1) && (!DoubleEq(fIncAngularPosLeft,0) || !DoubleEq(fIncAngularPosRight,0) ))
	////AddNoise(&fIncAngularPosLeft,&fIncAngularPosRight);

	///* Update encoder readings */
	///* NOTE: Now encoders are returning values between 0-2*PI, it should be 
	 //*      * changed according to the Robot Api */
	////m_fEncoderReadings[0] += fIncAngularPosLeft;
	////m_fEncoderReadings[1] += fIncAngularPosRight;

	///* Normalize encoder Readings */
	////m_fEncoderReadings[0] = NormalizeAngle( m_fEncoderReadings[0] );
	////m_fEncoderReadings[1] = NormalizeAngle( m_fEncoderReadings[1] );

	///* Update Position and Orientation for next calculation */
	//m_fLastOrientation = fOrientation;
	//m_vLastPosition = vPosition;

	/* Output data */
  double fLeftSpeed;
  double fRightSpeed;

  p_pcEpuck->GetWheelSpeed(&fLeftSpeed, &fRightSpeed);

  /* Because sim is 100ms scale */
  fLeftSpeed /= 10.0;
  fRightSpeed /= 10.0;
  
  /* Go for error */
  SetInput(0,fLeftSpeed + Random::nextDouble(-fLeftSpeed*m_fEncoderError,fLeftSpeed*m_fEncoderError));
  SetInput(1,fRightSpeed + Random::nextDouble(-fRightSpeed*m_fEncoderError,fRightSpeed*m_fEncoderError));

  //*f_enc_left = fIncSLeft+Random::nextDouble(-fIncSLeft*m_fEncoderError,fIncSLeft*m_fEncoderError);
  //SetInput(0,fIncSLeft+Random::nextDouble(-fIncSLeft*m_fEncoderError,fIncSLeft*m_fEncoderError));
  //*f_enc_right = fIncSRight+Random::nextDouble(-fIncSRight*m_fEncoderError,fIncSRight*m_fEncoderError);
  //SetInput(1,fIncSRight+Random::nextDouble(-fIncSRight*m_fEncoderError,fIncSRight*m_fEncoderError));
	
  //return GetInputs();
}


/******************************************************************************/
/******************************************************************************/

unsigned int CEncoderSensor::GetType(){
	return SENSOR_ENCODER;
}

/******************************************************************************/
/******************************************************************************/

//void  CEncoderSensor::GetSensorReading( CEpuck *p_pcEpuck, float *f_enc_left, float *f_enc_right){
double*  CEncoderSensor::GetSensorReading( CEpuck *p_pcEpuck)
{
	///* Get Actual Position of the epuck */
	//dVector2 vPosition;
	//vPosition.x = (p_pcEpuck->GetPosition()).x; 
	//vPosition.y = (p_pcEpuck->GetPosition()).y;

	///* Get increment of the Position */
	//dVector2 vIncPosition;
	//vIncPosition.x = vPosition.x - m_vLastPosition.x;
	//vIncPosition.y = vPosition.y - m_vLastPosition.y;

	///* Get actual orientation of the robot */
	//float fOrientation = p_pcEpuck->GetRotation();
	///* Get increment of the orientation */
	//float fIncOrientation = fOrientation - m_fLastOrientation;

	///* Normalize Robot Orientation Increment */
	//while(fIncOrientation > M_PI) fIncOrientation -= 2 * M_PI;
	//while(fIncOrientation < - M_PI) fIncOrientation += 2 * M_PI;

	///* Define Encoder increments, and Angular Position Increments */
	//float fIncSLeft, fIncSRight;
	////float fIncAngularPosLeft, fIncAngularPosRight;

	///* Brief explanation on kinematics movement */
	///* 
	 //* The IncTheta ( Angular movement ) of the robot is given by: 
	 //*   IncTheta = ( IncSr - IncSl ) / b, 
	 //*   
	 //*   where IncSr is the increment of the Right wheel encoder in standard units
	 //*   where IncSl is the increment of the Left wheel encoder in standard units
	 //*   where b is the distance between the 2 ideal points of connection of the wheels and the floor.
	 //*   In this case is the Cepuck::WHEELS_DISTANCE, defined in footbot_entity.h in
	 //*   simulator/swarmanoid_space/entities.
	 //*
	 //* The IncS ( Linear movement ) of the robot is given by:
	 //*  IncS = ( IncSr + IncSl ) / 2
	 //*
	 //*
	 //*  Xi = Xi-1 + IncS * cos(Thetai-1 + IncTheta/2);
	 //*  Yi = Yi-1 + IncS * sin(Thetai-1 + IncTheta/2);
	 //*  Thetai = Thetai-1 + IncTheta;
	 //*
	 //*  This ecuations should be manage in a inverse way to obtain the encoder movement of the robot
	 //*
	 //*  Special cases must be manage when Robot Turning on himself on when Robot moving in a straight line.
	 //*
	 //*  */

	///* If Robot turning over himself each wheel goes in a sense with the same increment 
	 //*      * So IncSR = - IncSl --> IncS = 0 and IncTheta = 2 * IncSl / b */
	//if(DoubleEq(vIncPosition.x,0) && DoubleEq(vIncPosition.y,0)){
		///* Inc x = 0 && Inc y = 0 --> Robot turns over his body */

		///* Calc linear movement of each wheel */
		//fIncSLeft = -fIncOrientation * CEpuck::WHEELS_DISTANCE / 2;
		//fIncSRight = -fIncSLeft;

		///* Calc angular movement of each wheel */
		////fIncAngularPosLeft = fIncSLeft / CFootBotEntity::FOOTBOT_WHEEL_RADIUS;
		////fIncAngularPosRight = fIncSRight / CFootBotEntity::FOOTBOT_WHEEL_RADIUS;
	//}
	
	///* If Robot going straight a head IncTheta = 0*/
	//else if(DoubleEq(fOrientation,m_fLastOrientation)){
		///* fIncU = IncS */
		//double fIncU = 0;
		///* IncX = IncS * cos (Thetai-1 + 0) */
		///* IncY = IncS * sin (Thetai-1 + 0) */
		///* Calc IncS */
		//if(DoubleEq(vPosition.x,0)) fIncU = vIncPosition.y/sin(fOrientation);
		//else fIncU = vIncPosition.x/cos(fOrientation);

		///* Calc IncSl and IncSr. 
		 //* Because IncTheta = 0, IncSr = IncSl 
		 //* IncS = ( IncSr + IncSl ) / 2 = IncSr = IncSl */
		//fIncSLeft = fIncU;
		//fIncSRight = fIncU;

		///* Calc angular movement of each wheel */
		////fIncAngularPosLeft = fIncSLeft / CFootBotEntity::FOOTBOT_WHEEL_RADIUS;
		////fIncAngularPosRight = fIncAngularPosLeft;

	//}
	///* If doing an arc as movement 
	 //* NOTE: Take into account that in this case robot movement is approximated by a strait line. 
	 //* It will be the cord of the arc made, that it's enough if Inct --> 0, so if big time steps
	 //* approximation will no be valid, but there is not other way to do it*/
	//else{
		///* fIncU = IncS */
		//double fIncU = 0;
		///* IncX = IncS * cos (Thetai-1 + IncTheta/2) */
		///* IncY = IncS * sin (Thetai-1 + IncTheta/2) */
		///* Calc IncS */
		//if(DoubleEq(vPosition.x,0)) fIncU = vIncPosition.y/sin(fOrientation) + fIncOrientation / 2;
		//else fIncU = vIncPosition.x/cos(fOrientation) + fIncOrientation /2;

		///* Calc IncSl and IncSr */
		///* IncS = (IncSr + IncSl) / 2 --> 2 * IncS = IncSr + IncSl
		 //* IncTheta = (IncSr - IncSl ) / b --> b * IncTheta = IncSr - IncSl
		 //* then
		 //* IncSr = IncS + b/2 * IncTheta
		 //* IncSl = IncS - b/2 * IncTheta */
		//fIncSLeft = fIncU - CEpuck::WHEELS_DISTANCE * fIncOrientation / 2;
		//fIncSRight = fIncU + CEpuck::WHEELS_DISTANCE * fIncOrientation / 2;

		///* Calc Angular movement of each whell */
		////fIncAngularPosLeft = fIncSLeft / CFootBotEntity::FOOTBOT_WHEEL_RADIUS;
		////fIncAngularPosRight = fIncSRight / CFootBotEntity::FOOTBOT_WHEEL_RADIUS;
	//}

	///* If Param noise on and there is movement on the wheels, add noise */
	////if(DoubleEq(m_fNoise,1) && (!DoubleEq(fIncAngularPosLeft,0) || !DoubleEq(fIncAngularPosRight,0) ))
	////AddNoise(&fIncAngularPosLeft,&fIncAngularPosRight);

	///* Update encoder readings */
	///* NOTE: Now encoders are returning values between 0-2*PI, it should be 
	 //*      * changed according to the Robot Api */
	////m_fEncoderReadings[0] += fIncAngularPosLeft;
	////m_fEncoderReadings[1] += fIncAngularPosRight;

	///* Normalize encoder Readings */
	////m_fEncoderReadings[0] = NormalizeAngle( m_fEncoderReadings[0] );
	////m_fEncoderReadings[1] = NormalizeAngle( m_fEncoderReadings[1] );

	///* Update Position and Orientation for next calculation */
	//m_fLastOrientation = fOrientation;
	//m_vLastPosition = vPosition;

	///* Output data */
	//*f_enc_left = fIncSLeft+Random::nextDouble(-fIncSLeft*m_fEncoderError,fIncSLeft*m_fEncoderError);
	//*f_enc_right = fIncSRight+Random::nextDouble(-fIncSRight*m_fEncoderError,fIncSRight*m_fEncoderError);

  return GetInputs();
}

void CEncoderSensor::InitEncoderSensor (CEpuck *p_pcEpuck){
	/* Get Position of the epuck and stored on the Initial variables*/
	m_vInitialPosition.x = (p_pcEpuck->GetPosition()).x; 
	m_vInitialPosition.y = (p_pcEpuck->GetPosition()).y;
	m_fInitialOrientation = (p_pcEpuck->GetRotation());

	/* Updated the LastKnownPosition and orientation */
	m_vLastPosition.x = m_vInitialPosition.x; 
	m_vLastPosition.y = m_vInitialPosition.y;
	m_fLastOrientation = m_fInitialOrientation;
}
