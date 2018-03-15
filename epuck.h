#ifndef EPUCK_H
#define EPUCK_H

/******************************************************************************/
/******************************************************************************/

#include <math.h>
#include <vector>
#include <list>

using namespace std;

/******************************************************************************/
/******************************************************************************/

class CEpuck;

typedef vector<CEpuck*>              TEpuckVector;
typedef vector<CEpuck*>::iterator    TEpuckIterator;

/******************************************************************************/
/******************************************************************************/

#include "puck.h"
#include "geometry.h"
#include "sensor.h"
#include "simulator.h"
#include "actuator.h"
#include "controller.h"

#define MAX_REAL_SPEED 1000
#define MIN_REAL_SPEED -1000

/******************************************************************************/
/******************************************************************************/

class CEpuck : public CGeometry
{
public:
    CEpuck(const char* pch_name, double f_posx, double f_posy, double f_rotation);
	~CEpuck();
    virtual void Draw(CRender* pc_render);
    

	//TO BE REMOVED	
	bool m_bBuggyEpuck;
	bool GetBuggy(){return m_bBuggyEpuck;}
	void SetBuggy(bool b){m_bBuggyEpuck=b;}

	
	//Collision stuff
	void SetCollisions(unsigned int);
	void AddCollision();
	unsigned int GetCollisions();

	// Set/get the speed of the wheels (e.g. how fast the wheels are turning):
	virtual void GetWheelSpeed(double* left, double* right); 
	virtual void GetWheelTargetSpeed(double* left, double* right); 
	virtual void SetWheelSpeed(double left, double right); 

	// Change the relative speed of the wheels.
	virtual void Accellerate(double left, double right);   
	virtual double GetMaxWheelSpeed();
	virtual void SetMaxWheelSpeedFactor( double f_max_speed );

	virtual void SetInitTime( double f_init_time );
	void SimulationStep(unsigned int n_simulation_step, 
			double f_time, 
			double f_step_interval);


	virtual double GetChassisRotation();

	virtual void SetColor(float f_red, float f_green, float f_blue);

	virtual void AddActuator(CActuator* pc_actuator);
	virtual void RemoveActuator(CActuator* pc_actuator);
	virtual TActuatorVector GetActuators();
	virtual CActuator* GetActuator(unsigned int un_type);

	// This function returns copy of the stl::vector of sensors.
	// Although it might have been faster to return a reference
	// to the vector, this one returns a copy. Reason: Less chance
	// of screw-ups and copying a couple of pointers do not 
	// take very long.
	virtual TSensorVector GetSensors();
	virtual void AddSensor(CSensor* pc_sensor);
	virtual void RemoveSensor(CSensor* pc_sensor);
	virtual void RemoveAllSensors( void );
	virtual CSensor* GetSensor(unsigned int un_type);
	virtual void Sense(CSimulator* pc_parent_simulator);
	// Remember only to add a controller AFTER all sensors and 
	// actuators have been added!
	virtual void SetController(CController* pc_controller);
	virtual CController* GetController() const;
	void SetControllerType( unsigned int un_controller );
	unsigned int GetControllerType();
	virtual void RemoveAndDeleteController();

	virtual void RemoveAllActuators();

	// Get the initial rotation of the epuck:
	virtual double GetInitialRotation();
	virtual void   SetInitialRotation(double f_initial_rotation);
	// These SetRotation methods are instant:
	virtual void SetChassisRotation(double f_angle);

	// Get/Set color of LEDs
	virtual unsigned int GetAllColoredLeds();
	virtual void         SetAllColoredLeds(unsigned int un_color);
	virtual bool         SomeColoredLedIsOn();
	virtual unsigned int GetColoredLed(unsigned int un_index);
	virtual void         SetColoredLed(unsigned int un_index, unsigned int un_color);

	//Used to know heading of movement when epuck moves backwards
	virtual double GetMovementHeading();

	//Set and get epuck's class
	virtual void SetClass(int c);
	virtual int GetClass();

	//All about the gripper
	virtual void SetGripperPresence(bool b_gripper_presence);
	virtual bool GetGripperPresence();
	virtual void SetGrippedPuck(CPuck* pc_gripped_puck);
	virtual CPuck* GetGrippedPuck();
	virtual void SetGripperLength(double f_gripper_length);
	virtual double GetGripperLength();
	virtual void SetGripperAngle(double f_gripper_angle);
	virtual double GetGripperAngle();

	// Some constants for the geometry of the epucks:
	static double CHASSIS_RADIUS;
	static double CHASSIS_RADIUS_FALSE;

	// Constants for the physics and capabilities of an epuck:
	static double MAX_EPUCK_SPEED;
	static double MAX_EPUCK_ACCELLERATION;

	// Default color of an epuck
	static float DEFAULT_COLOR_RED;
	static float DEFAULT_COLOR_GREEN;
	static float DEFAULT_COLOR_BLUE;

	//Distance between wheels
	static float WHEELS_DISTANCE;

	void SetLabel(const char *label);
	void GetLabel( char * label );

	/* RANDB BOARD */
	void SetNestPosition ( dVector2 nest_pos_estimated, dVector2 robot_pos_estimated, double robot_orien_estimated, long int confidence_level);
	void SetPreyPosition ( dVector2 prey_pos_estimated, dVector2 robot_pos_estimated, double robot_orien_estimated, long int confidence_level);
	int GetNestPosition ( dVector2* nest_pos_estimated , long int* nest_confidence_level);
	int GetPreyPosition ( dVector2* prey_pos_estimated , long int* prey_confidence_level);
	void ResetNestPosition ( void );
	void ResetPreyPosition ( void );
	
	/* Projection */
	void SetNestProjection ( int state);
	void SetPreyProjection ( int state);
	int GetNestProjection ( void );
	int GetPreyProjection ( void );

	/* COM SENSOR */
	void 	SetComData ( int n_data);
	int 	GetComData ( void );

protected:
	void KeepSpeedWithinBounds();

protected:
	static double MAX_ANGULAR_WHEEL_SPEED;
	static double EPUCK_MASS;
	static double GRIPPER_LENGTH;
	static double GRIPPER_ANGLE;

protected:
	double             	m_fLeftWheelSpeed;
	double             	m_fRightWheelSpeed;
	double             	m_fMaxWheelSpeedFactor;    
	double    	       	m_fInitTime;

	double             	m_fLeftWheelTargetSpeed;
	double             	m_fRightWheelTargetSpeed;    
	
	double             	m_fChassisRotation;
	float              	m_fColorRed;
	float              	m_fColorGreen;
	float              	m_fColorBlue;

	TSensorVector      	m_vecSensors;
	TActuatorVector    	m_vecActuators;

	CController*       	m_pcController;

	double             	m_fInitialRotation;

	int		           		m_punColoredLeds[8];

	unsigned int       	m_unControllerType;
	unsigned int       	m_unCollisionCount;

	unsigned int	   		m_unClass; //Use to represent differences among epucks (es: different colors)

	bool               	m_bHasGripper;
	CPuck*             	m_pcGrippedPuck;

	char								m_pcLabel[9];

	/* RANDB BOARD */
	dVector2 						m_NestPosition;
	dVector2 						m_PreyPosition;
	int 								m_uNestKnown;
	int 								m_uPreyKnown;
	long int 						m_lNestConfidenceLevel;
	long int 						m_lPreyConfidenceLevel;

	/* Projection */
	int									m_uShowNestProjection;
	int									m_uShowPreyProjection;

	/* COM SENSOR */
	int 								m_nComData;
};

/******************************************************************************/
/******************************************************************************/

#endif
