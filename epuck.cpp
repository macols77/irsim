#include "epuck.h"
#include "render.h"

/******************************************************************************/
/******************************************************************************/

#define INFINITE 100000

double CEpuck::CHASSIS_RADIUS     = 0.035;
//double CEpuck::CHASSIS_RADIUS     = 0.0000001;
double CEpuck::CHASSIS_RADIUS_FALSE     = 0.0001;

double CEpuck::MAX_ANGULAR_WHEEL_SPEED = M_PI * 2;

double CEpuck::EPUCK_MASS               = 1.5;
double CEpuck::MAX_EPUCK_SPEED          = 0.1288053; //m/s

double CEpuck::MAX_EPUCK_ACCELLERATION  = 2.0;  // m/s

float CEpuck::DEFAULT_COLOR_RED       = 0;
float CEpuck::DEFAULT_COLOR_GREEN     = 1;
float CEpuck::DEFAULT_COLOR_BLUE      = 1;

float CEpuck::WHEELS_DISTANCE = 0.053; 

double CEpuck::GRIPPER_LENGTH = 4 * 2*CPuck::CHASSIS_RADIUS;
double CEpuck::GRIPPER_ANGLE = 55*M_PI/180;

/******************************************************************************/
/******************************************************************************/

CEpuck::CEpuck(const char* pch_name, double f_xpos, double f_ypos, double f_rotation) : 
    CGeometry(pch_name, f_xpos, f_ypos, f_rotation, EPUCK_MASS), m_fLeftWheelSpeed(0), 
    m_fRightWheelSpeed(0), m_fMaxWheelSpeedFactor(1), m_fInitTime(0), m_fChassisRotation(f_rotation), m_fColorRed(DEFAULT_COLOR_RED), 
    m_fColorGreen(DEFAULT_COLOR_GREEN), m_fColorBlue(DEFAULT_COLOR_BLUE), 
    m_pcController(NULL), m_fInitialRotation(f_rotation), m_fLeftWheelTargetSpeed(0), m_fRightWheelTargetSpeed(0),
    m_bHasGripper(false), m_pcGrippedPuck(NULL)    
{
	SetAllColoredLeds(LED_COLOR_BLACK);
	m_unCollisionCount=0;
	//TO BE REMOVED
	m_bBuggyEpuck=false;
	m_unClass=0; //Default class, all epucks are the same	
	SetLabel(pch_name);

	/* RANDB BOARD */
	ResetNestPosition();
	ResetPreyPosition();

	/* COM SENSOR */
	m_nComData = 0;
}

/******************************************************************************/
/******************************************************************************/

CEpuck::~CEpuck(){	
}

/******************************************************************************/
/******************************************************************************/

void CEpuck::Draw(CRender* pc_render)
{
    pc_render->DrawEpuck(this, m_fColorRed, m_fColorGreen, m_fColorBlue);
    CSimObject::Draw(pc_render);
}

/******************************************************************************/
/******************************************************************************/

void CEpuck::GetWheelSpeed(double* left, double* right)
{
    (*left)  = m_fLeftWheelSpeed;
    (*right) = m_fRightWheelSpeed;
}

/******************************************************************************/
/******************************************************************************/

void CEpuck::GetWheelTargetSpeed(double* left, double* right)
{
    (*left)  = m_fLeftWheelTargetSpeed;
    (*right) = m_fRightWheelTargetSpeed;
}

/******************************************************************************/
/******************************************************************************/

void CEpuck::SetWheelSpeed(double left, double right)
{
    m_fLeftWheelTargetSpeed  = left;
    m_fRightWheelTargetSpeed = right;
}

/******************************************************************************/
/******************************************************************************/

void CEpuck::Accellerate(double left, double right)
{ 
    m_fLeftWheelTargetSpeed  += left;
    m_fRightWheelTargetSpeed += right;
}

/******************************************************************************/
/******************************************************************************/

void CEpuck::KeepSpeedWithinBounds()
{
    double fMaxSpeed = MAX_EPUCK_SPEED*m_fMaxWheelSpeedFactor;
    if (m_fLeftWheelSpeed < -fMaxSpeed)  m_fLeftWheelSpeed  = -fMaxSpeed;
    if (m_fLeftWheelSpeed >  fMaxSpeed)  m_fLeftWheelSpeed  =  fMaxSpeed;
    if (m_fRightWheelSpeed < -fMaxSpeed) m_fRightWheelSpeed = -fMaxSpeed;
    if (m_fRightWheelSpeed >  fMaxSpeed) m_fRightWheelSpeed =  fMaxSpeed;
}

/******************************************************************************/
/******************************************************************************/

double CEpuck::GetMovementHeading(){
	if(m_fRightWheelSpeed<0 && m_fLeftWheelSpeed <0){
		double aux = GetRotation() + M_PI;
		while ( aux < 0.0 ) aux += 2 * M_PI;
		while ( aux > 2 * M_PI ) aux -= 2 * M_PI;
		return aux;
	}
	return GetRotation();
}

/******************************************************************************/
/******************************************************************************/

void CEpuck::SetCollisions(unsigned int num){
//	printf("Setting collisions: %d for epuck: %s\n",num,GetName());
//	sleep(1);
	m_unCollisionCount=num;
}

/******************************************************************************/
/******************************************************************************/

void CEpuck::AddCollision(){
	m_unCollisionCount++;	
}

/******************************************************************************/
/******************************************************************************/

unsigned int CEpuck::GetCollisions(){
	return m_unCollisionCount;	
}

/******************************************************************************/
/******************************************************************************/

void CEpuck::SimulationStep(unsigned int n_simulation_step, 
                           double f_time, 
                           double f_step_interval) 
{

    if( f_time < m_fInitTime ) return;
	//if(m_bBuggyEpuck)
	//printf("EPUCK STEP %d\n",n_simulation_step);
    CSimObject::SimulationStep(n_simulation_step, f_time, f_step_interval);
	m_fLeftWheelSpeed=m_fLeftWheelTargetSpeed;
	m_fRightWheelSpeed=m_fRightWheelTargetSpeed;

  double R           = 0.0;
  double omega       = 0.0;
  float fNewXpos= GetPosition().x;
  float fNewYpos= GetPosition().y; 
	
	//float tmpOldX=fNewXpos;
	//float tmpOldY=fNewYpos;
	
	//printf("########################################################\n");
	//printf("Epuck_name %s Position: %f --%f\n",GetName(),fNewXpos,fNewYpos);
	//printf("Epuck_name %s Heading: %f\n",GetName(),GetRotation()*180/M_PI);
	//printf("Epuck_name %s Speeds: %f --%f\n",GetName(),m_fLeftWheelSpeed,m_fRightWheelSpeed);
	//sleep(1);
	//fflush(stdout);
	/**/
  float fNewRotation = GetRotation();
	
  bool bTurningOnSpot=false;
  if(fabs(fabs(m_fLeftWheelSpeed)-fabs(m_fRightWheelSpeed))<0.00001){
	  if((m_fLeftWheelSpeed>0 && m_fRightWheelSpeed<0) || (m_fLeftWheelSpeed<0 && m_fRightWheelSpeed>0)){
	  bTurningOnSpot=true;
	  }		  
  }
/*
	printf("Speeds: %f -- %f\n",m_fLeftWheelSpeed,m_fRightWheelSpeed);
  fflush(stdout);
 /* */
  
  
  
  if((fabs(fabs(m_fLeftWheelSpeed)-fabs(m_fRightWheelSpeed))>0.00001) || bTurningOnSpot){ //(m_fLeftWheelSpeed==-m_fRightWheelSpeed)
	  R=(WHEELS_DISTANCE*(m_fRightWheelSpeed+m_fLeftWheelSpeed))/((m_fRightWheelSpeed-m_fLeftWheelSpeed)*2.0);
	  omega=(m_fRightWheelSpeed-m_fLeftWheelSpeed)/WHEELS_DISTANCE;
	/*R=(wheel_distance*(linearVelocity[1]+linearVelocity[0]))/
      ((linearVelocity[1]-linearVelocity[0])*2.0);*/
	  
	 //printf("Rotation radius: %f\n",R); 
	  double ICC_x = fNewXpos - R * sin(fNewRotation);
      double ICC_y = fNewYpos + R * cos(fNewRotation);
	  //printf("ICC_x,ICC_y: %f -- %f\n",ICC_x,ICC_y);
	  
	  fNewXpos=((fNewXpos-ICC_x)*cos(omega*f_step_interval)-(fNewYpos-ICC_y)*sin(omega*f_step_interval)+ICC_x);
	  fNewYpos=((fNewXpos-ICC_x)*sin(omega*f_step_interval)+(fNewYpos-ICC_y)*cos(omega*f_step_interval)+ICC_y);
	  //printf("fNewXpos,fNewYpos: %f--%f\n",fNewXpos,fNewYpos);
	 // 	sleep(1);
	  fNewRotation +=omega*f_step_interval;
		while (fNewRotation < 0.0 ) fNewRotation += 2 * M_PI;
		while (fNewRotation > 2 * M_PI ) fNewRotation -= 2 * M_PI;

	  
  }else{
	fNewXpos +=m_fLeftWheelSpeed*f_step_interval*cos(fNewRotation);
	fNewYpos +=m_fRightWheelSpeed*f_step_interval*sin(fNewRotation);
  }
	/* 
  
void Robot::updateRobotPos( void ) {

 double R           = 0.0;
  double omega       = 0.0;
  
  if ( linearVelocity[0] != linearVelocity[1] ){
    R=(wheel_distance*(linearVelocity[1]+linearVelocity[0]))/
      ((linearVelocity[1]-linearVelocity[0])*2.0);
    
    omega = (linearVelocity[1]-linearVelocity[0])/wheel_distance;  
    double ICC_x = position[0] - R * sin(orientation);
    double ICC_y = position[1] + R * cos(orientation);
    
    position[0]=((position[0]-ICC_x)*cos(omega*(double)( TIMESTEP))
		 -(position[1]-ICC_y)*sin(omega*(double)( TIMESTEP))+ICC_x);
    position[1]=((position[0]-ICC_x)*sin(omega*(double)( TIMESTEP))
		 +(position[1]-ICC_y)*cos(omega*(double)( TIMESTEP))+ICC_y);
    orientation += (omega * (double)( TIMESTEP));
    if ( orientation >= TWOPI ) orientation -= TWOPI;
    if ( orientation < 0.0 ) orientation += TWOPI;
  }
  else{
    position[0]  += linearVelocity[1] * (double)(TIMESTEP) * cos(orientation);
    position[1]  += linearVelocity[0] * (double)(TIMESTEP) * sin(orientation);
  }

  // Add noise to position and orientation
  position[0]  += dran1()*2.0*positionNoise    - positionNoise;
  position[1]  += dran1()*2.0*positionNoise    - positionNoise;
  orientation  += dran1()*2.0*orientationNoise - orientationNoise;
  if ( orientation > TWOPI ) orientation -= TWOPI;
  if ( orientation < 0.0 ) orientation += TWOPI;

}
*/

	  SetRotation(fNewRotation);
	  SetPosition(fNewXpos,fNewYpos);
	  //printf("NEW position: %f --%f\n",fNewXpos,fNewYpos);
	  //printf("########################################################\n");
	  //sleep(1);
      //SetPosition(fNewX, fNewY);
	  
//	printf("EPUCK: SimulationStep, heading: %f\n",fNewRotation*180/M_PI);
//	fflush(stdout);
	//getchar();
}

/******************************************************************************/
/******************************************************************************/

double CEpuck::GetChassisRotation()
{
    return m_fChassisRotation;
}

/******************************************************************************/
/******************************************************************************/

void CEpuck::SetColor(float f_red, float f_green, float f_blue)
{
    m_fColorRed     = f_red;
    m_fColorGreen   = f_green;
    m_fColorBlue    = f_blue;
}

/******************************************************************************/
/******************************************************************************/

void CEpuck::AddSensor(CSensor* pc_sensor)
{
    m_vecSensors.push_back(pc_sensor);
    AddChild(pc_sensor);
}

/******************************************************************************/
/******************************************************************************/

void CEpuck::RemoveSensor(CSensor* pc_sensor)
{
    vector<CSensor*>::iterator i = m_vecSensors.begin();

    while (i != m_vecSensors.end() && (*i) != pc_sensor)
        i++;

    if (i == m_vecSensors.end())
    {
        printf("%s tried to remove a non-existing sensor %s", GetName(), pc_sensor->GetName());
        fflush(stdout);
    } else {
        m_vecSensors.erase(i);
    }

    RemoveChild(pc_sensor);
}

/******************************************************************************/
/******************************************************************************/

void CEpuck::RemoveAllSensors( void )
{
  vector<CSensor*>::iterator i = m_vecSensors.begin();
  while (i != m_vecSensors.end() ) {
    RemoveChild(*i);    
    m_vecSensors.erase(i);
  }
}

/******************************************************************************/
/******************************************************************************/

TSensorVector CEpuck::GetSensors()
{
    return m_vecSensors;
}

/******************************************************************************/
/******************************************************************************/

void CEpuck::Sense(CSimulator* pc_parent_simulator)
{
    TSensorIterator i;
    for (i = m_vecSensors.begin(); i != m_vecSensors.end(); i++)
    {
        (*i)->ComputeSensorReadings(this, pc_parent_simulator);
    }
}

/******************************************************************************/
/******************************************************************************/

double CEpuck::GetMaxWheelSpeed()
{
    return MAX_EPUCK_SPEED*m_fMaxWheelSpeedFactor;
}

/******************************************************************************/
/******************************************************************************/

void CEpuck::SetMaxWheelSpeedFactor( double f_max_speed )
{
    m_fMaxWheelSpeedFactor = f_max_speed;
}

/******************************************************************************/
/******************************************************************************/

void  CEpuck::SetInitTime( double f_init_time ) {
  m_fInitTime = f_init_time;
}

/******************************************************************************/
/******************************************************************************/

void CEpuck::AddActuator(CActuator* pc_actuator)
{
    m_vecActuators.push_back(pc_actuator);
    AddChild(pc_actuator);
}

void CEpuck::RemoveActuator(CActuator* pc_actuator)
{
    vector<CActuator*>::iterator i = m_vecActuators.begin();

    while (i != m_vecActuators.end() && (*i) != pc_actuator)
        i++;

    if (i == m_vecActuators.end())
    {
        printf("%s tried to remove a non-existing sensor %s", GetName(), pc_actuator->GetName());
        fflush(stdout);
    } else {
        m_vecActuators.erase(i);
    }

    RemoveChild(pc_actuator);
}


/******************************************************************************/
/******************************************************************************/

TActuatorVector CEpuck::GetActuators()
{
    return m_vecActuators;
}

/******************************************************************************/
/******************************************************************************/

void CEpuck::SetController(CController* pc_controller)
{
    m_pcController = pc_controller;
    // By adding the controller as a child, it automatically gets
    // updated every simulation step - so this is all that needs to
    // be done:
	
	//printf("Epuck: %s, setting controller: %s\n",GetName(),pc_controller->GetName());
    AddChild(pc_controller);
}

/******************************************************************************/
/******************************************************************************/

void CEpuck::RemoveAndDeleteController()
{
    if (m_pcController != NULL)
    {
        RemoveChild(m_pcController);
        delete m_pcController;
        m_pcController = NULL;
    }
}

/******************************************************************************/
/******************************************************************************/

CController* CEpuck::GetController() const
{
    return m_pcController;
}

/******************************************************************************/
/******************************************************************************/

double CEpuck::GetInitialRotation()
{
    return m_fInitialRotation;
}

/******************************************************************************/
/******************************************************************************/

void CEpuck::SetInitialRotation(double f_initial_rotation)
{
    m_fInitialRotation = f_initial_rotation;
}

/******************************************************************************/
/******************************************************************************/

CSensor* CEpuck::GetSensor(unsigned int un_type)
{
    int i    = 0;
    CSensor* pcSensor = NULL;

    while (i < m_vecSensors.size() && pcSensor == NULL) 
    {
        if (m_vecSensors[i]->GetType() == un_type)
        {
            pcSensor = m_vecSensors[i];
        } 
        else 
        {
            i++;
        }
    }

    return pcSensor;
}

/******************************************************************************/
/******************************************************************************/

void CEpuck::SetChassisRotation(double f_angle)
{
    m_fChassisRotation = f_angle;
}

/******************************************************************************/
/******************************************************************************/

CActuator* CEpuck::GetActuator(unsigned int un_type)
{
    int i    = 0;
    CActuator* pcActuator = NULL;
    while (i < m_vecActuators.size() && pcActuator == NULL) 
    {
        if (m_vecActuators[i]->GetType() == un_type)
        {
            pcActuator = m_vecActuators[i];
        } 
        else 
        {
            i++;
        }
    }

    return pcActuator;    
}

/******************************************************************************/
/******************************************************************************/

unsigned int CEpuck::GetAllColoredLeds(){
    return m_punColoredLeds[0];
}

/******************************************************************************/
/******************************************************************************/

void CEpuck::SetAllColoredLeds(unsigned int un_color)
{
    for (int i = 0; i < 8; i++)
        m_punColoredLeds[i] = un_color ;
    
    if(un_color == LED_COLOR_BLACK) 		  SetColor(0,0,0);
    else if(un_color == LED_COLOR_RED) 	  SetColor(1,0,0);
    else if(un_color == LED_COLOR_GREEN)   SetColor(0,1,0);
    else if(un_color == LED_COLOR_BLUE) 	  SetColor(0,0,1);
    else if(un_color == LED_COLOR_YELLOW ) SetColor(1,1,0);
    else if(un_color == LED_COLOR_WHITE ) SetColor(1,1,1);
}

/******************************************************************************/
/******************************************************************************/

void CEpuck::SetColoredLed(unsigned int un_index, unsigned int un_color)
{
    m_punColoredLeds[un_index] = un_color;
}

/******************************************************************************/
/******************************************************************************/

unsigned int CEpuck::GetColoredLed(unsigned int un_index)
{
    return m_punColoredLeds[un_index];
}

/******************************************************************************/
/******************************************************************************/

void CEpuck::SetControllerType( unsigned int un_controller )
{
  m_unControllerType = un_controller;
}

/******************************************************************************/
/******************************************************************************/

unsigned int CEpuck::GetControllerType()
{
  return m_unControllerType;
}

/******************************************************************************/
/******************************************************************************/

bool CEpuck::SomeColoredLedIsOn()
{
    unsigned int unIndex = 0;
    while (unIndex < 8)
    {
        if (m_punColoredLeds[unIndex] != LED_COLOR_BLACK)
            return true;
        else
            unIndex++;
    }
    return false;
}

/******************************************************************************/
/******************************************************************************/

void CEpuck::SetClass(int c){
	m_unClass=c;
	
	if(c==0){
		m_fColorRed=DEFAULT_COLOR_RED;
		m_fColorBlue=DEFAULT_COLOR_BLUE;
		m_fColorGreen=DEFAULT_COLOR_GREEN;
	}else if(c==1){
		m_fColorRed=1.0;
		m_fColorBlue=0.0;
		m_fColorGreen=0.0;		
	}else if(c==2){
		m_fColorRed=0.0;
		m_fColorBlue=1.0;
		m_fColorGreen=0.0;				
	}else{
		m_fColorRed=0.0;
		m_fColorBlue=0.0;
		m_fColorGreen=1.0;				
	}
}

/******************************************************************************/
/******************************************************************************/

int CEpuck::GetClass(){
	return m_unClass;
}

/******************************************************************************/
/******************************************************************************/

void CEpuck::SetGripperPresence(bool b_gripper_presence)
{
  m_bHasGripper = b_gripper_presence;
}

/******************************************************************************/
/******************************************************************************/

bool CEpuck::GetGripperPresence()
{
  return m_bHasGripper;
}

/******************************************************************************/
/******************************************************************************/

void CEpuck::SetGrippedPuck(CPuck* pc_gripped_puck)
{
    if(m_bHasGripper)
        m_pcGrippedPuck = pc_gripped_puck;
    else
    {
        printf("You try to assign a gripped puck to an epuck without gripper !\n");
        exit(1);
    }
}

/******************************************************************************/
/******************************************************************************/

CPuck* CEpuck::GetGrippedPuck()
{
  return m_pcGrippedPuck;
}

/******************************************************************************/
/******************************************************************************/

void CEpuck::SetGripperLength(double f_gripper_length)
{
  GRIPPER_LENGTH = f_gripper_length;
}

/******************************************************************************/
/******************************************************************************/

double CEpuck::GetGripperLength()
{
  return GRIPPER_LENGTH;
}

/******************************************************************************/
/******************************************************************************/

/*
double CEpuck::GetProbablityOfSuccessfulGrip()
{
}

/******************************************************************************/
/******************************************************************************/

void CEpuck::RemoveAllActuators(){
  vector<CActuator*>::iterator i = m_vecActuators.begin();
  while (i != m_vecActuators.end() ) {
    RemoveChild(*i);    
    m_vecActuators.erase(i);
  }
}

/******************************************************************************/
/******************************************************************************/

void CEpuck::SetGripperAngle(double f_gripper_angle)
{
  GRIPPER_ANGLE = f_gripper_angle;
}

/******************************************************************************/
/******************************************************************************/

double CEpuck::GetGripperAngle()
{
  return GRIPPER_ANGLE;
}

/******************************************************************************/
/******************************************************************************/

void CEpuck::SetLabel(const char *label){
	memcpy(m_pcLabel,label,9);
}

/******************************************************************************/
/******************************************************************************/
void CEpuck::GetLabel( char * label ){
	memcpy(label, m_pcLabel,9);
}

/******************************************************************************/
/******************************************************************************/

/* RANDB BOARD */
void CEpuck::SetNestPosition ( dVector2 nest_pos_estimated, dVector2 robot_pos_estimated, double robot_orien_estimated, long int confidence_level)
{
	m_uNestKnown = true;
	m_lNestConfidenceLevel = confidence_level;
	/* Calc Rotation of the relative axis of the robot */
	double fThetaAxis = GetRotation() - robot_orien_estimated;

	/* Calc the position of the Robot oriented to the inertial system */
	dVector2 vPosEstimatedOrien;
	vPosEstimatedOrien.x = robot_pos_estimated.x * cos ( fThetaAxis) - robot_pos_estimated.y * sin ( fThetaAxis);
	vPosEstimatedOrien.y = robot_pos_estimated.x * sin ( fThetaAxis) + robot_pos_estimated.y * cos ( fThetaAxis);

	/* Calc the x,y position of the relative axis */
	dVector2 vAxisPos;
	dVector2 vRealRobotPos = GetPosition();
	vAxisPos.x = vRealRobotPos.x - vPosEstimatedOrien.x;
	vAxisPos.y = vRealRobotPos.y - vPosEstimatedOrien.y;

	/* Transform the Nest Position to the correct orientation */
	dVector2 vNestPosEstimatedOrien;
	vNestPosEstimatedOrien.x = nest_pos_estimated.x * cos (fThetaAxis) - nest_pos_estimated.y * sin (fThetaAxis);
	vNestPosEstimatedOrien.y = nest_pos_estimated.x * sin (fThetaAxis) + nest_pos_estimated.y * cos (fThetaAxis);

	/* Get the intertial coordinates of the NEST */
	m_NestPosition.x = vAxisPos.x + vNestPosEstimatedOrien.x;
	m_NestPosition.y = vAxisPos.y + vNestPosEstimatedOrien.y;

	/* DEBUG */
	//printf("NEST: X: %2f, Y: %2f\n",m_NestPosition.x,m_NestPosition.y);
	/* DEBUG */
}

/******************************************************************************/
/******************************************************************************/

void CEpuck::SetPreyPosition ( dVector2 prey_pos_estimated, dVector2 robot_pos_estimated, double robot_orien_estimated, long int confidence_level)
{
	m_uPreyKnown = true;
	m_lPreyConfidenceLevel = confidence_level;

	/* Calc Rotation of the relative axis of the robot */
	double fThetaAxis = GetRotation() - robot_orien_estimated;

	/* Calc the position of the Robot oriented to the inertial system */
	dVector2 vPosEstimatedOrien;
	vPosEstimatedOrien.x = robot_pos_estimated.x * cos ( fThetaAxis) - robot_pos_estimated.y * sin ( fThetaAxis);
	vPosEstimatedOrien.y = robot_pos_estimated.x * sin ( fThetaAxis) + robot_pos_estimated.y * cos ( fThetaAxis);

	/* Calc the x,y position of the relative axis */
	dVector2 vAxisPos;
	dVector2 vRealRobotPos = GetPosition();
	vAxisPos.x = vRealRobotPos.x - vPosEstimatedOrien.x;
	vAxisPos.y = vRealRobotPos.y - vPosEstimatedOrien.y;

	/* Transform the Prey Position to the correct orientation */
	dVector2 vPreyPosEstimatedOrien;
	vPreyPosEstimatedOrien.x = prey_pos_estimated.x * cos (fThetaAxis) - prey_pos_estimated.y * sin (fThetaAxis);
	vPreyPosEstimatedOrien.y = prey_pos_estimated.x * sin (fThetaAxis) + prey_pos_estimated.y * cos (fThetaAxis);

	/* Get the intertial coordinates of the NEST */
	m_PreyPosition.x = vAxisPos.x + vPreyPosEstimatedOrien.x;
	m_PreyPosition.y = vAxisPos.y + vPreyPosEstimatedOrien.y;

	/* DEBUG */
	//printf("PREY: X: %2f, Y: %2f\n",m_PreyPosition.x , m_PreyPosition.y);
	/* DEBUG */

}

/******************************************************************************/
/******************************************************************************/

void CEpuck::ResetNestPosition ( void ){
	m_uNestKnown = false;
}

/******************************************************************************/
/******************************************************************************/

void CEpuck::ResetPreyPosition ( void )
{
	m_uPreyKnown = false;
}

/******************************************************************************/
/******************************************************************************/
int CEpuck::GetNestPosition ( dVector2* nest_pos_estimated , long int* nest_confidence_level)
{
	if (m_uNestKnown)
	{
		*nest_confidence_level = m_lNestConfidenceLevel;
		*nest_pos_estimated = m_NestPosition;
		return true;
	}
	else return false;
}


/******************************************************************************/
/******************************************************************************/

int CEpuck::GetPreyPosition ( dVector2* prey_pos_estimated , long int* prey_confidence_level)
{
	if (m_uPreyKnown)
	{
		*prey_confidence_level = m_lPreyConfidenceLevel;
		*prey_pos_estimated = m_PreyPosition;

		return true;
	}
	else return false;
}

/******************************************************************************/
/******************************************************************************/

void CEpuck::SetNestProjection ( int state)
{
	m_uShowNestProjection = state;
}

/******************************************************************************/
/******************************************************************************/

void CEpuck::SetPreyProjection ( int state)
{
	m_uShowPreyProjection = state;
}

/******************************************************************************/
/******************************************************************************/

int CEpuck::GetNestProjection ( void )
{
	return m_uShowNestProjection;
}

/******************************************************************************/
/******************************************************************************/

int CEpuck::GetPreyProjection ( void )
{
	return m_uShowPreyProjection;
}

/******************************************************************************/
/******************************************************************************/
void CEpuck::SetComData ( int n_data )
{
	m_nComData = n_data;
}
/******************************************************************************/
/******************************************************************************/
int CEpuck::GetComData ( void )
{
	return m_nComData;
}
