#include "arena.h"


/******************************************************************************/
/******************************************************************************/

CArena::CArena(const char* pch_name) : CGeometry(pch_name, 0, 0, 0, 0),
                                       m_fSizeX(0), m_fSizeY(0), 
                                       m_unResX(0), m_unResY(0)
{
  m_pColorFunction = NULL;
}

/******************************************************************************/
/******************************************************************************/

CArena::~CArena() 
{
//     if( m_pcGroundTexture )
//         delete m_pcGroundTexture;
}

/******************************************************************************/
/******************************************************************************/

void CArena::GetSize(double* f_size_x, double* f_size_y) 
{
    (*f_size_x) = m_fSizeX;
    (*f_size_y) = m_fSizeY;
}

/******************************************************************************/
/******************************************************************************/

void CArena::GetResolution(unsigned int* un_res_x, unsigned int* un_res_y)
{
    (*un_res_x) = m_unResX;
    (*un_res_y) = m_unResY;
}

/******************************************************************************/
/******************************************************************************/

void CArena::SetSize(double f_size_x, double f_size_y)
{
    m_fSizeX = f_size_x;
    m_fSizeY = f_size_y;
    
}

/******************************************************************************/
/******************************************************************************/

void CArena::SetResolution(unsigned int un_res_x, unsigned int un_res_y)
{
    m_unResX = un_res_x;
    m_unResY = un_res_y;
}


/******************************************************************************/
/******************************************************************************/

void CArena::AddShelter(CCircleCollisionObject* pc_Shelter){
	m_vecShelters.push_back(pc_Shelter);
	AddChild(pc_Shelter);
}

/******************************************************************************/
/******************************************************************************/

bool CArena::IsUnderShelter(float f_X,float f_Y){
	vector<CCircleCollisionObject*>::iterator it=m_vecShelters.begin();
	bool found=false;
	while(it!=m_vecShelters.end() && !found){
		dVector2 shelterPos=(*it)->GetPosition();
		if( sqrt( pow( (shelterPos.x - f_X), 2 ) + pow( (shelterPos.y - f_Y), 2 )) < (*it)->GetRadius())
				found = true;
		it++;
	}
	return found;
}

/******************************************************************************/
/******************************************************************************/

dVector2 CArena::GetClosestShelterPosition(float f_X, float f_Y)
{
    vector<CCircleCollisionObject*>::iterator it = m_vecShelters.begin();
    
    double minDistance = 99999.0;
    dVector2 closestShelterPos;

    while (it != m_vecShelters.end())
    {
	dVector2 shelterPos = (*it)->GetPosition();
	double d = sqrt( (shelterPos.x - f_X) * (shelterPos.x - f_X) + (shelterPos.y - f_Y) * (shelterPos.y - f_Y) ); 
	if (d < minDistance)
	{
	    minDistance = d;
	    closestShelterPos = shelterPos;
	}
	it++;
    }

    dVector2 res;
    res.x = closestShelterPos.x - f_X;
    res.y = closestShelterPos.y - f_Y;

    return res;
}


/******************************************************************************/
/******************************************************************************/

vector<CCircleCollisionObject*> CArena::GetShelters(){
	return m_vecShelters;
}

/******************************************************************************/
/******************************************************************************/

//vector<CColoredWall*> CArena::GetColoredWalls(){
	//return m_vecWalls;	
//}

///******************************************************************************/
///******************************************************************************/

//void CArena::AddColoredWall(CColoredWall* pc_wall){
	//m_vecWalls.push_back(pc_wall);
////	AddChild(pc_wall);
//}

/******************************************************************************/
/******************************************************************************/

//vector<CLightObject*> CArena::GetLights(){
//return m_vecLights;
//}

/******************************************************************************/
/******************************************************************************/
//bool CArena::GetLightStatus(int index){
//return m_vecLights[index]->GetStatus();
//}

/******************************************************************************/
/******************************************************************************/

//void CArena::AddLight(CLightObject* pc_Light){
//m_vecLights.push_back(pc_Light);
//AddChild(pc_Light);
//}

/******************************************************************************/
/******************************************************************************/

//void CArena::SetLightStatus(bool bStat,int index){
//m_vecLights[index]->SetStatus(bStat);
//}

/******************************************************************************/
/******************************************************************************/

void CArena::AddGroundArea(CGroundArea* pc_ground_area){
	m_vecGroundArea.push_back(pc_ground_area);
	AddChild(pc_ground_area);
}

/******************************************************************************/
/******************************************************************************/

double* CArena::GetGroundAreaColor(dVector2 Pos, double f_orientation){
//davidf: sensors placed like in the real e-puck

	double fSensorsDistance = 0.03;
	double fSensorsGap = 0.01;
	vector<CGroundArea*>::iterator it=m_vecGroundArea.begin();
	double *fcolor = new double[3];
	fcolor[0]=1.0; //LEFT
	fcolor[1]=1.0; //CENTER
	fcolor[2]=1.0; //RIGHT
	
	while(it!=m_vecGroundArea.end()){
		dVector2 groundAreaPos;
		(*it)->GetCenter(&groundAreaPos);
		double radius;
		(*it)->GetExtRadius(&radius);
		//Check center ground: now at center-front
		if( sqrt( pow( (groundAreaPos.x - (Pos.x + (fSensorsDistance * cos(f_orientation)))), 2 ) + pow( (groundAreaPos.y - (Pos.y + (fSensorsDistance * sin(f_orientation)))), 2 )) < radius){
			(*it)->GetIntRadius(&radius);
		if( sqrt( pow( (groundAreaPos.x - (Pos.x + (fSensorsDistance * cos(f_orientation)))), 2 ) + pow( (groundAreaPos.y - (Pos.y + (fSensorsDistance * sin(f_orientation)))), 2 )) > radius){
				(*it)->GetColor(&fcolor[1]);
			}
		}
		//Check right ground: now at front-right
		(*it)->GetExtRadius(&radius);
		if( sqrt( pow( (groundAreaPos.x - (Pos.x + (fSensorsDistance * cos(f_orientation)+fSensorsGap * sin(f_orientation)))), 2 ) + pow( (groundAreaPos.y - (Pos.y + (fSensorsDistance * sin(f_orientation)-fSensorsGap * cos(f_orientation)))), 2 )) < radius){
			(*it)->GetIntRadius(&radius);
		if( sqrt( pow( (groundAreaPos.x - (Pos.x + (fSensorsDistance * cos(f_orientation)+fSensorsGap * sin(f_orientation)))), 2 ) + pow( (groundAreaPos.y - (Pos.y + (fSensorsDistance * sin(f_orientation)-fSensorsGap * cos(f_orientation)))), 2 )) > radius){
				(*it)->GetColor(&fcolor[2]);
			}
		}
		//Check Left ground: now at front-left
		(*it)->GetExtRadius(&radius);
		if( sqrt( pow( (groundAreaPos.x - (Pos.x + (fSensorsDistance * cos(f_orientation)-fSensorsGap * sin(f_orientation)))), 2 ) + pow( (groundAreaPos.y - (Pos.y + (fSensorsDistance * sin(f_orientation)+fSensorsGap * cos(f_orientation)))), 2 )) < radius){
			(*it)->GetIntRadius(&radius);
		if( sqrt( pow( (groundAreaPos.x - (Pos.x + (fSensorsDistance * cos(f_orientation)-fSensorsGap * sin(f_orientation)))), 2 ) + pow( (groundAreaPos.y - (Pos.y + (fSensorsDistance * sin(f_orientation)+fSensorsGap * cos(f_orientation)))), 2 )) > radius){
				(*it)->GetColor(&fcolor[0]);
			}
		}

		it++;
	}
	return fcolor;
}

/******************************************************************************/
/******************************************************************************/

char* CArena::GetGroundAreaName(dVector2 Pos, double f_orientation){
//davidf: sensors placed like in the real e-puck

	float fSensorsDistance = 0.03;
	float fSensorsGap = 0.01;
	vector<CGroundArea*>::iterator it=m_vecGroundArea.begin();

	char *s_name="none";

	while(it!=m_vecGroundArea.end()){

		dVector2 groundAreaPos;
		(*it)->GetCenter(&groundAreaPos);
		double radius;
		(*it)->GetExtRadius(&radius);
		//Check center ground: now at center-front
		if( sqrt( pow( (groundAreaPos.x - (Pos.x + (fSensorsDistance * cos(f_orientation)))), 2 ) + pow( (groundAreaPos.y - (Pos.y + (fSensorsDistance * sin(f_orientation)))), 2 )) < radius){
			(*it)->GetIntRadius(&radius);
			if( sqrt( pow( (groundAreaPos.x - (Pos.x + (fSensorsDistance * cos(f_orientation)))), 2 ) + pow( (groundAreaPos.y - (Pos.y + (fSensorsDistance * sin(f_orientation)))), 2 )) > radius)
			{
				s_name = (*it)->GetName();
				return s_name;
			}
		}
		it++;
	}
	return s_name;
}


/******************************************************************************/
/******************************************************************************/

vector<CGroundArea*> CArena::GetGroundAreas(){
	return m_vecGroundArea;
}
/******************************************************************************/
/******************************************************************************/

void CArena::Draw(CRender* pc_render)
{
    pc_render->DrawArena(this);

    CSimObject::Draw(pc_render);
}


/******************************************************************************/
/******************************************************************************/

double CArena::GetGroundColor(dVector2 v_position)
{
    if (m_pColorFunction != NULL )
        return (*m_pColorFunction)(v_position.x, v_position.y);

    return 1.0;
}

/******************************************************************************/
/******************************************************************************/
void CArena::SimulationStep(unsigned int n_step_number, 
                            double f_time, 
                            double f_step_interval)
{


    CSimObject::SimulationStep(n_step_number, f_time, f_step_interval);

		/* Get light timing */
		vector<CLightObject*>::iterator it=m_vecLightObject.begin();
		while(it!=m_vecLightObject.end()){
			(*it)->Switch((*it)->GetTiming(n_step_number));
			it++;
		}
		
		/* Get blue light timing */
		vector<CBlueLightObject*>::iterator blue_it=m_vecBlueLightObject.begin();
		while(blue_it!=m_vecBlueLightObject.end()){
			(*blue_it)->Switch((*blue_it)->GetTiming(n_step_number));
			blue_it++;
		}
		
		/* Get blue light timing */
		vector<CRedLightObject*>::iterator red_it=m_vecRedLightObject.begin();
		while(red_it!=m_vecRedLightObject.end()){
			(*red_it)->Switch((*red_it)->GetTiming(n_step_number));
			red_it++;
		}
}

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
/******************************************************************************/

vector<CLightObject*> CArena::GetLightObject(){
	return m_vecLightObject;
}

/******************************************************************************/
/******************************************************************************/

void CArena::AddLightObject(CLightObject* pc_light_object){
	m_vecLightObject.push_back(pc_light_object);
	AddChild(pc_light_object);
}

/******************************************************************************/
/******************************************************************************/
bool CArena::LightDistance ( dVector2 Pos, double range, double *distance){
	vector<CLightObject*>::iterator it=m_vecLightObject.begin();
	while(it!=m_vecLightObject.end()){
		dVector2 lightObjectPos;
		(*it)->GetCenter(&lightObjectPos);
		*distance = sqrt( pow( (lightObjectPos.x - Pos.x), 2 ) + pow( (lightObjectPos.y - Pos.y), 2 ));
		//if( sqrt( pow( (lampPos.x - Pos.x), 2 ) + pow( (lampPos.y - Pos.y), 2 )) < range){
		if( *distance  < range){
			return true;
		}
		it++;
	}
	return false;
	}

/*****************************************************************************************************/
/*****************************************************************************************************/

bool CArena::GetNearestLight ( dVector2 Pos, double range, dVector2 *light_position, double *nearestDistance)
{
vector<CLightObject*>::iterator it=m_vecLightObject.begin();

	*nearestDistance = range;
	/* get all the Light Objects */
	while(it!=m_vecLightObject.end()){
		/* If on */
		if ((*it)->GetStatus())
		{
			/* Get tje Light Object Position */
			dVector2 lightObjectPos;
			(*it)->GetCenter(&lightObjectPos);
			/* Check the distance to the robot */
			double distance = sqrt( pow( (lightObjectPos.x - Pos.x), 2 ) + pow( (lightObjectPos.y - Pos.y), 2 ));
			/* If distance in range */
			if( (distance  < range) && (distance < *nearestDistance)){
				*nearestDistance = distance;
				(*light_position).x = lightObjectPos.x;
				(*light_position).y = lightObjectPos.y;
			}
		}
		it++;
	}

	if (*nearestDistance != range )
		return true;
	else
		return false;
}

/*****************************************************************************************************/
/*****************************************************************************************************/
bool CArena::GetNearestLightInSector ( dVector2 Pos, double f_orientation, double f_aperture, double range, dVector2 *light_position, double *nearestDistance, double *relativeAngle)
{
	vector<CLightObject*>::iterator it=m_vecLightObject.begin();

	double fMaxOrientation = 0.0;
	double fMinOrientation = 0.0;

	*nearestDistance = range;
	/* get all the Light Objects */
	while(it!=m_vecLightObject.end()){
		/* If on */
		if ((*it)->GetStatus())
		{
			//printf("Light on\n");

			/* Get the Light Object Position */
			dVector2 lightObjectPos;
			(*it)->GetCenter(&lightObjectPos);
			/* Check the distance to the robot */
			double distance = sqrt( pow( (lightObjectPos.x - Pos.x), 2 ) + pow( (lightObjectPos.y - Pos.y), 2 ));
			/* If distance in range */
			if( (distance  < range) && (distance < *nearestDistance))
			{
				//printf("Light in range\n");
				/* Check if in sector */
				/* Calc absolute incidence light beam angle */
				double fRobotLightAngle = atan2((lightObjectPos.y - Pos.y),(lightObjectPos.x - Pos.x));
				//fRobotLightAngle = NormalizeAngle(fRobotLightAngle);
				//printf("fRobotLight: %2.4f\n", fRobotLightAngle);

				/* Calc Sector */
				double fRelAngle = fRobotLightAngle - f_orientation;
				if (fabs(fRelAngle) > M_PI )
				{
					if (fRobotLightAngle > f_orientation )
						f_orientation += 2*M_PI;
					else
						fRobotLightAngle += 2*M_PI;
				}	
				
				fRelAngle = fRobotLightAngle - f_orientation;

				if (fabs(fRelAngle) <= f_aperture )
				{
					//printf("Light in Sector\n");
					*nearestDistance = distance;
					*relativeAngle   = fRelAngle;
					(*light_position).x = lightObjectPos.x;
					(*light_position).y = lightObjectPos.y;
				}
			}
		}
		it++;
	}

	if (*nearestDistance != range )
		return true;
	else
		return false;
}

/*****************************************************************************************************/
/*****************************************************************************************************/
bool CArena::GetNearestBlueLightInSector ( dVector2 Pos, double f_orientation, double f_aperture, double range, dVector2 *light_position, double *nearestDistance, double *relativeAngle)
{
	vector<CBlueLightObject*>::iterator it=m_vecBlueLightObject.begin();

	double fMaxOrientation = 0.0;
	double fMinOrientation = 0.0;

	*nearestDistance = range;
	/* get all the Light Objects */
	while(it!=m_vecBlueLightObject.end()){
		/* If on */
		if ((*it)->GetStatus())
		{
			//printf("Light on\n");

			/* Get the Light Object Position */
			dVector2 lightObjectPos;
			(*it)->GetCenter(&lightObjectPos);
			/* Check the distance to the robot */
			double distance = sqrt( pow( (lightObjectPos.x - Pos.x), 2 ) + pow( (lightObjectPos.y - Pos.y), 2 ));
			/* If distance in range */
			if( (distance  < range) && (distance < *nearestDistance))
			{
				//printf("Light in range\n");
				/* Check if in sector */
				/* Calc absolute incidence light beam angle */
				double fRobotLightAngle = atan2((lightObjectPos.y - Pos.y),(lightObjectPos.x - Pos.x));
				//fRobotLightAngle = NormalizeAngle(fRobotLightAngle);
				//printf("fRobotLight: %2.4f\n", fRobotLightAngle);

				/* Calc Sector */
				double fRelAngle = fRobotLightAngle - f_orientation;
				if (fabs(fRelAngle) > M_PI )
				{
					if (fRobotLightAngle > f_orientation )
						f_orientation += 2*M_PI;
					else
						fRobotLightAngle += 2*M_PI;
				}	
				
				fRelAngle = fRobotLightAngle - f_orientation;

				if (fabs(fRelAngle) <= f_aperture )
				{
					//printf("Light in Sector\n");
					*nearestDistance = distance;
					*relativeAngle   = fRelAngle;
					(*light_position).x = lightObjectPos.x;
					(*light_position).y = lightObjectPos.y;
				}
			}
		}
		it++;
	}

	if (*nearestDistance != range )
		return true;
	else
		return false;
}

/*****************************************************************************************************/
/*****************************************************************************************************/
bool CArena::GetNearestRedLightInSector ( dVector2 Pos, double f_orientation, double f_aperture, double range, dVector2 *light_position, double *nearestDistance, double *relativeAngle)
{
	vector<CRedLightObject*>::iterator it=m_vecRedLightObject.begin();

	double fMaxOrientation = 0.0;
	double fMinOrientation = 0.0;

	*nearestDistance = range;
	/* get all the Light Objects */
	while(it!=m_vecRedLightObject.end()){
		/* If on */
		if ((*it)->GetStatus())
		{
			//printf("Light on\n");

			/* Get the Light Object Position */
			dVector2 lightObjectPos;
			(*it)->GetCenter(&lightObjectPos);
			/* Check the distance to the robot */
			double distance = sqrt( pow( (lightObjectPos.x - Pos.x), 2 ) + pow( (lightObjectPos.y - Pos.y), 2 ));
			/* If distance in range */
			if( (distance  < range) && (distance < *nearestDistance))
			{
				//printf("Light in range\n");
				/* Check if in sector */
				/* Calc absolute incidence light beam angle */
				double fRobotLightAngle = atan2((lightObjectPos.y - Pos.y),(lightObjectPos.x - Pos.x));
				//fRobotLightAngle = NormalizeAngle(fRobotLightAngle);
				//printf("fRobotLight: %2.4f\n", fRobotLightAngle);

				/* Calc Sector */
				double fRelAngle = fRobotLightAngle - f_orientation;
				if (fabs(fRelAngle) > M_PI )
				{
					if (fRobotLightAngle > f_orientation )
						f_orientation += 2*M_PI;
					else
						fRobotLightAngle += 2*M_PI;
				}	
				
				fRelAngle = fRobotLightAngle - f_orientation;

				if (fabs(fRelAngle) <= f_aperture )
				{
					//printf("Light in Sector\n");
					*nearestDistance = distance;
					*relativeAngle   = fRelAngle;
					(*light_position).x = lightObjectPos.x;
					(*light_position).y = lightObjectPos.y;
				}
			}
		}
		it++;
	}

	if (*nearestDistance != range )
		return true;
	else
		return false;
}

/*****************************************************************************************************/
/*****************************************************************************************************/

void CArena::SwitchNearestLight (dVector2 Pos, int n_value)
{
	vector<CLightObject*>::iterator it=m_vecLightObject.begin();
	
	CLightObject* light;
	double nearestDistance = 10000;
	bool lightFound = false;
	/* get all the Light Objects */
	while(it!=m_vecLightObject.end()){
		/* Get the Light Object Position */
		dVector2 lightObjectPos;
		(*it)->GetCenter(&lightObjectPos);
		/* Check the distance to the robot */
		double distance = sqrt( pow( (lightObjectPos.x - Pos.x), 2 ) + pow( (lightObjectPos.y - Pos.y), 2 ));
		/* If distance in range */
		if( distance < nearestDistance ){
			light = (CLightObject*) (*it);
			nearestDistance = distance;
		}
		it++;
		lightFound = true;
	}

	if ( lightFound == true )
		light->Switch(n_value);
}

/*****************************************************************************************************/
/*****************************************************************************************************/

vector<CBlueLightObject*> CArena::GetBlueLightObject(){
	return m_vecBlueLightObject;
}

/******************************************************************************/
/******************************************************************************/

void CArena::AddBlueLightObject(CBlueLightObject* pc_blue_light_object){
	m_vecBlueLightObject.push_back(pc_blue_light_object);
	AddChild(pc_blue_light_object);
}

/******************************************************************************/
/******************************************************************************/
bool CArena::BlueLightDistance ( dVector2 Pos, double range, double *distance){
	vector<CBlueLightObject*>::iterator it=m_vecBlueLightObject.begin();
	while(it!=m_vecBlueLightObject.end()){
		dVector2 lightObjectPos;
		(*it)->GetCenter(&lightObjectPos);
		*distance = sqrt( pow( (lightObjectPos.x - Pos.x), 2 ) + pow( (lightObjectPos.y - Pos.y), 2 ));
		//if( sqrt( pow( (lampPos.x - Pos.x), 2 ) + pow( (lampPos.y - Pos.y), 2 )) < range){
		if( *distance  < range){
			return true;
		}
		it++;
	}
	return false;
	}

/*****************************************************************************************************/
/*****************************************************************************************************/


bool CArena::GetNearestBlueLight ( dVector2 Pos, double range, dVector2 *light_position, double *nearestDistance)
{
	vector<CBlueLightObject*>::iterator it=m_vecBlueLightObject.begin();
	
	*nearestDistance = range;
	/* get all the Light Objects */
	while(it!=m_vecBlueLightObject.end()){
		/* If on */
		if ((*it)->GetStatus())
		{
			/* Get tje Light Object Position */
			dVector2 lightObjectPos;
			(*it)->GetCenter(&lightObjectPos);
			/* Check the distance to the robot */
			double distance = sqrt( pow( (lightObjectPos.x - Pos.x), 2 ) + pow( (lightObjectPos.y - Pos.y), 2 ));
			/* If distance in range */
			if( (distance  < range) && (distance < *nearestDistance)){
				*nearestDistance = distance;
				(*light_position).x = lightObjectPos.x;
				(*light_position).y = lightObjectPos.y;
			}
		}
		it++;
	}

	if (*nearestDistance != range )
		return true;
	else
		return false;
}

/*****************************************************************************************************/
/*****************************************************************************************************/

void CArena::SwitchNearestBlueLight (dVector2 Pos, int n_value)
{
	vector<CBlueLightObject*>::iterator it=m_vecBlueLightObject.begin();
	
	CBlueLightObject* light;
	double nearestDistance = 10000;
	bool lightFound = false;
	/* get all the Light Objects */
	while(it!=m_vecBlueLightObject.end()){
		/* Get the Light Object Position */
		dVector2 lightObjectPos;
		(*it)->GetCenter(&lightObjectPos);
		/* Check the distance to the robot */
		double distance = sqrt( pow( (lightObjectPos.x - Pos.x), 2 ) + pow( (lightObjectPos.y - Pos.y), 2 ));
		/* If distance in range */
		if( distance < nearestDistance ){
			light = (CBlueLightObject*) (*it);
			nearestDistance = distance;
		}
		it++;
		lightFound = true;
	}

	if ( lightFound == true )
		light->Switch(n_value);
}

/*****************************************************************************************************/
/*****************************************************************************************************/

vector<CRedLightObject*> CArena::GetRedLightObject(){
	return m_vecRedLightObject;
}

/******************************************************************************/
/******************************************************************************/

void CArena::AddRedLightObject(CRedLightObject* pc_red_light_object){
	m_vecRedLightObject.push_back(pc_red_light_object);
	AddChild(pc_red_light_object);
}

/******************************************************************************/
/******************************************************************************/
bool CArena::RedLightDistance ( dVector2 Pos, double range, double *distance){
	vector<CRedLightObject*>::iterator it=m_vecRedLightObject.begin();
	while(it!=m_vecRedLightObject.end()){
		dVector2 lightObjectPos;
		(*it)->GetCenter(&lightObjectPos);
		*distance = sqrt( pow( (lightObjectPos.x - Pos.x), 2 ) + pow( (lightObjectPos.y - Pos.y), 2 ));
		//if( sqrt( pow( (lampPos.x - Pos.x), 2 ) + pow( (lampPos.y - Pos.y), 2 )) < range){
		if( *distance  < range){
			return true;
		}
		it++;
	}
	return false;
	}

/*****************************************************************************************************/
/*****************************************************************************************************/


bool CArena::GetNearestRedLight ( dVector2 Pos, double range, dVector2 *light_position, double *nearestDistance)
{
	vector<CRedLightObject*>::iterator it=m_vecRedLightObject.begin();
	
	*nearestDistance = range;
	/* get all the Light Objects */
	while(it!=m_vecRedLightObject.end()){
		/* If on */
		if ((*it)->GetStatus())
		{
			/* Get tje Light Object Position */
			dVector2 lightObjectPos;
			(*it)->GetCenter(&lightObjectPos);
			/* Check the distance to the robot */
			double distance = sqrt( pow( (lightObjectPos.x - Pos.x), 2 ) + pow( (lightObjectPos.y - Pos.y), 2 ));
			/* If distance in range */
			if( (distance  < range) && (distance < *nearestDistance)){
				*nearestDistance = distance;
				(*light_position).x = lightObjectPos.x;
				(*light_position).y = lightObjectPos.y;
			}
		}
		it++;
	}

	if (*nearestDistance != range )
		return true;
	else
		return false;
}

/*****************************************************************************************************/
/*****************************************************************************************************/

void CArena::SwitchNearestRedLight (dVector2 Pos, int n_value)
{
	vector<CRedLightObject*>::iterator it=m_vecRedLightObject.begin();
	
	CRedLightObject* light;
	double nearestDistance = 10000;
	bool lightFound = false;
	/* get all the Light Objects */
	while(it!=m_vecRedLightObject.end()){
		/* Get the Light Object Position */
		dVector2 lightObjectPos;
		(*it)->GetCenter(&lightObjectPos);
		/* Check the distance to the robot */
		double distance = sqrt( pow( (lightObjectPos.x - Pos.x), 2 ) + pow( (lightObjectPos.y - Pos.y), 2 ));
		/* If distance in range */
		if( distance < nearestDistance ){
			light = (CRedLightObject*) (*it);
			nearestDistance = distance;
		}
		it++;
		lightFound = true;
	}

	if ( lightFound == true )
		light->Switch(n_value);
}

/*****************************************************************************************************/
/*****************************************************************************************************/
