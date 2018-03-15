#include "simpledrawstuffrender.h"
#include <math.h>
#include <stdio.h>
using namespace std;
/******************************************************************************/
/******************************************************************************/

CSimpleDrawStuffRender::CSimpleDrawStuffRender(CSimulator* pc_simulator, int argc, char** argv) :
    CRender(pc_simulator),
    m_argc(argc), 
    m_argv(argv), 
    m_fSimulationFrames(0), 
    m_bEnableFollowSbot(false),
    m_bDisplayPrompt(true),
    m_bPauseAtStart(true),
    m_bUseCustomTextureForGround(false)
{
	m_unWindowSizeX = 640;
//m_unWindowSizeX = 1024;
//	m_unWindowSizeX = 300;
	m_unWindowSizeY = 480;
		//m_unWindowSizeY = 768;
//	m_unWindowSizeY = 300;
    m_bUseTextures = false;
    
    //m_bPauseAtStart = false;
	m_bDrawIRSensors=false;
	m_bDrawLightSensors=false;
	m_bDrawBlueLightSensors=false;
	m_bDrawRedLightSensors=false;
	m_bDrawContactSensors=false;
	m_bDrawCamera=true;
	m_bDrawSound=true;
}

/******************************************************************************/
/******************************************************************************/

void CSimpleDrawStuffRender::Start()
{
    dsFunctions fn;
    fn.version          = DS_VERSION;
    fn.start            = &CSimpleDrawStuffRender::DSStartCallback;
    fn.step             = &CSimpleDrawStuffRender::DSSimLoopCallback;
    fn.command          = &CSimpleDrawStuffRender::DSCommandCallback;
    fn.stop             = 0;
    fn.path_to_textures = "textures";

    char* argv[5];
    unsigned int unArguments = 1;
    argv[0] = "./twodee";
   
    if (!m_bUseTextures)
    {
        argv[unArguments++] = "-notex";
    } 
     
    if (m_bPauseAtStart)
    {
        argv[unArguments++] = "-pause";
    }
    dsSimulationLoop(unArguments, argv, m_unWindowSizeX, m_unWindowSizeY, &fn);    
    fprintf(stderr, "\n");

}

/******************************************************************************/
/******************************************************************************/

void CSimpleDrawStuffRender::SetFrameRate(float fFR){
	m_fFrameRate=fFR;
}

/******************************************************************************/
/******************************************************************************/

void CSimpleDrawStuffRender::DSCommandCallback(int key_code)
{
    ((CSimpleDrawStuffRender*) CSimpleDrawStuffRender::GetInstance())->CommandCallback(key_code);
}

/******************************************************************************/
/******************************************************************************/

void CSimpleDrawStuffRender::DSSimLoopCallback(int pause)
{
    ((CSimpleDrawStuffRender*) CSimpleDrawStuffRender::GetInstance())->SimLoopCallback(pause != 0);
}

/******************************************************************************/
/******************************************************************************/

void CSimpleDrawStuffRender::DSStartCallback()
{
    ((CSimpleDrawStuffRender*) CSimpleDrawStuffRender::GetInstance())->StartCallback();
}

/******************************************************************************/
/******************************************************************************/


void CSimpleDrawStuffRender::CommandCallback(int key_code)
{

	if (key_code == 'p')
		m_bDisplayPrompt = !m_bDisplayPrompt;

	if (key_code == '+')
		m_fFrameRate *= 1.1;

	if (key_code == '-')
		m_fFrameRate /= 1.1;

	/* Draw IR sensors */
	if (key_code == 'i') 
	{
		m_bDrawIRSensors=!m_bDrawIRSensors;
	}

	/* Draw LIGHT sensors */
	if(key_code == 'l'){
		m_bDrawLightSensors=!m_bDrawLightSensors;
	}
	
	/* Draw CONTACT sensors */
	if(key_code == 'c'){
		m_bDrawContactSensors=!m_bDrawContactSensors;
	}

	/* Draw BLUE LIGHT sensors */
	if(key_code == 'b'){
		m_bDrawBlueLightSensors=!m_bDrawBlueLightSensors;
	}
	
	if(key_code == 'r'){
		m_bDrawRedLightSensors=!m_bDrawRedLightSensors;
	}
	
	if(key_code == '.'){
		m_bDrawCamera=!m_bDrawCamera;
	}

	if(key_code == ','){
		m_bDrawSound=!m_bDrawSound;
	}

	if (m_fFrameRate < 0.1)
		m_fFrameRate = 0.1;

	m_pcSimulator->Keypressed(key_code);
}

/******************************************************************************/
/******************************************************************************/

void CSimpleDrawStuffRender::SimLoopCallback(int pause)
{
    while (m_fSimulationFrames < 0 && !m_pcSimulator->HasEnded() && !pause) 
    {
        // Let the geometry act
        m_pcSimulator->TakeSimulationStep();
        m_fSimulationFrames += 1;
        m_fIndicatorSinusAngle += 0.2;
    }

    if (!pause)
        m_fSimulationFrames -= m_fFrameRate;

    m_pcSimulator->Draw(this);    

    if (m_pcSimulator->HasEnded())
        dsStop();

	int intpart;
    //if( m_bProduceFrames && m_iFrame%10 == 0 )
    if( m_bProduceFrames )
    	dumpCurrentFrame();
    m_iFrame++;

}

/******************************************************************************/
/******************************************************************************/


void CSimpleDrawStuffRender::dumpCurrentFrame()
{
   // build the filename
   char filename[100];
   sprintf (filename, "frames/frame_%05d.ppm", m_iFrame);

   int width = m_unWindowSizeX;
   int height = m_unWindowSizeY;
   char frameBuffer[width*height];
   FILE* f = fopen(filename,"wb");
   fprintf(f, "P6\n# CREATOR : Volren\n%d %d\n255\n", width, height);
   for (int i = height - 1; i >= 0; i--)
   {
       glReadPixels(0, i, width, 1, GL_RGB, GL_UNSIGNED_BYTE, frameBuffer); 
       fwrite(frameBuffer, width * 3, 1, f);
   }
   fclose(f);
}

/******************************************************************************/
/******************************************************************************/

void CSimpleDrawStuffRender::StartCallback() 
{

	static float xyz[3] = {0.0f,0.0f, 2.91f};
	static float hpr[3] = {90.0,-90.0f,0.0f};
	dsSetViewpoint (xyz,hpr);
}

/******************************************************************************/
/******************************************************************************/

void CSimpleDrawStuffRender::DrawArena(CArena* pc_arena)
{

  dsRotationMatrix rotmat;
  double pos[3];
  double sides[3];
  dRotation2dsRotationMatrix(0, rotmat);

  dsSetTexture(DS_NONE);

  unsigned int unResX;
  unsigned int unResY;

  double fSizeX;
  double fSizeY;

  pc_arena->GetResolution(&unResX, &unResY);
  if (unResX == 0 || unResY == 0) 
  {
    printf("Only descrete arenas are supported by this render!");
    fflush(stdout);
    return;
  }   

  pc_arena->GetSize(&fSizeX, &fSizeY);

  //printf("SIZE: %2f, %2f RES: %2f, %2f\n",fSizeX,fSizeY,(double) unResX, (double) unResY);

  double fBoxSizeX = fSizeX / (double) unResX;
  double fBoxSizeY = fSizeY / (double) unResY;

  double fCurrentX = -fSizeX / 2 + fBoxSizeX / 2;

  sides[0] = fBoxSizeX;
  sides[1] = fBoxSizeY;

  for (int x = 0; x < unResX; x++)
  {
    double fCurrentY = -fSizeY / 2 + fBoxSizeY / 2;
    pos[0] = fCurrentX; 

    for (int y = 0; y < unResY; y++)
    {
      eArenaHeight h = pc_arena->GetHeight(fCurrentX, fCurrentY);

      double fBoxHeight = 0;

      switch (h) {
        case HEIGHT_HOLE:     
          if (m_bUseTextures)
          {
            dsSetTexture(DS_NONE); 
          }                    

          dsSetColor(0.0, 0.0, 0.0); 
          fBoxHeight = 0.01; 
          break;
        case HEIGHT_NORMAL: {
                              dVector2 vPosition = { fCurrentX, fCurrentY };
                              double fColor = pc_arena->GetGroundColor(vPosition);
                              dsSetColor(fColor, fColor, fColor);

                              if (m_bUseTextures)
                              {
                                dsSetTexture(1); 
                                if (m_bUseCustomTextureForGround)
                                {    
                                  dsSetColor(1, 1, 1); 
                                }
                              }

                              fBoxHeight = 0.2; 
                              break;
                            }
        case HEIGHT_OBSTACLE: 
                            if (m_bUseTextures)
                            {
                              dsSetTexture(DS_NONE); 
                            }                    
                            dsSetColor(0.9, 0.9, 1.0); 
                            fBoxHeight = 0.4; 
                            break;
        default: {
                   printf("Unknown arena height!");
                   fflush(stdout);
                 }
      }

      pos[1] = fCurrentY; 
      pos[2] = fBoxHeight / 2; 
      sides[2] = fBoxHeight;
      dsDrawBoxD(pos, rotmat, sides);

      fCurrentY += fBoxSizeY;
    }

    fCurrentX += fBoxSizeX;
  }

  vector<CCircleCollisionObject*> vShelters=pc_arena->GetShelters();
  vector<CCircleCollisionObject*>::iterator it=vShelters.begin();


  dsSetColor(0.3, 0.0, 1.0);
  double fShelterHeight=0.0;

  while(it!=vShelters.end()){
    sides[0]=(*it)->GetRadius();
    pos[0]=((*it)->GetPosition()).x;
    pos[1]=((*it)->GetPosition()).y;
    pos[2]=0.26;

    //dsDrawBoxD(pos,rotmat,sides);
    dsDrawCylinderD(pos,
        (double *) &rotmat,
        0.01,
        sides[0]);
    it++;
  }

	/* Ground Area */

	vector<CGroundArea*> vGroundAreas=pc_arena->GetGroundAreas();
	vector<CGroundArea*>::iterator it_ground=vGroundAreas.begin();

	double f_radius;
	double f_color;
	while(it_ground!=vGroundAreas.end()){
		(*it_ground)->GetColor(&f_color);
		dsSetColor(f_color, f_color, f_color);

		(*it_ground)->GetExtRadius(&f_radius);
		dVector2 vCenter;
		(*it_ground)->GetCenter(&vCenter);
		pos[0]=vCenter.x;
		pos[1]=vCenter.y;
		//pos[2]=0.20;
		double fHeight;
		(*it_ground)->GetHeight(&fHeight);
		pos[2]=fHeight;

		//printf("X: %2f Y: %2f Rad: %2f\n",vCenter.x, vCenter.y,f_radius);
		//dsDrawBoxD(pos,rotmat,sides);
		dsDrawCylinderD(pos,
				(double *) &rotmat,
				0.01,
				f_radius);
		it_ground++;
	}

	/* LIGHT OBJECT */
	vector<CLightObject*> vLightObject=pc_arena->GetLightObject();
	vector<CLightObject*>::iterator it_lightobject=vLightObject.begin();

	f_radius=0.05;
	while(it_lightobject!=vLightObject.end()){
		if ( (*it_lightobject)->GetStatus() ) 
		{
			dsSetColor(1.0,1.0,0);

			dVector2 vCenter;
			(*it_lightobject)->GetCenter(&vCenter);
			pos[0]=vCenter.x;
			pos[1]=vCenter.y;
			pos[2]=0.4;

			//dsDrawCylinderD(pos,
			//(double *) &rotmat,
			//0.1,
			//f_radius);

			dsDrawSphereD((double*) &pos, 
					(double*) &rotmat, 
					f_radius);    
		}
		it_lightobject++;
	}
	
	/* BLUE LIGHT OBJECT */
	vector<CBlueLightObject*> vBlueLightObject=pc_arena->GetBlueLightObject();
	vector<CBlueLightObject*>::iterator it_bluelightobject=vBlueLightObject.begin();

	f_radius=0.05;
	while(it_bluelightobject!=vBlueLightObject.end()){
		if ( (*it_bluelightobject)->GetStatus() ) 
		{
			dsSetColor(0.0,0.0,1.0);

			dVector2 vCenter;
			(*it_bluelightobject)->GetCenter(&vCenter);
			pos[0]=vCenter.x;
			pos[1]=vCenter.y;
			pos[2]=0.4;

			dsDrawSphereD((double*) &pos, 
					(double*) &rotmat, 
					f_radius);    
		}
		it_bluelightobject++;
	}
	
	/* RED LIGHT OBJECT */
	vector<CRedLightObject*> vRedLightObject=pc_arena->GetRedLightObject();
	vector<CRedLightObject*>::iterator it_redlightobject=vRedLightObject.begin();

	f_radius=0.05;
	while(it_redlightobject!=vRedLightObject.end()){
		if ( (*it_redlightobject)->GetStatus() ) 
		{
			dsSetColor(0.5,0.0,0.0);

			dVector2 vCenter;
			(*it_redlightobject)->GetCenter(&vCenter);
			pos[0]=vCenter.x;
			pos[1]=vCenter.y;
			pos[2]=0.4;

			dsDrawSphereD((double*) &pos, 
					(double*) &rotmat, 
					f_radius);    
		}
		it_redlightobject++;
	}
}

/******************************************************************************/
/******************************************************************************/
void CSimpleDrawStuffRender::DrawEpuck(CEpuck* pc_epuck, float f_color_red, float f_color_green, float f_color_blue){

	dsRotationMatrix rotmat;
	double pos[3];
	double sides[3];

	pc_epuck->GetPosition(&pos[0],&pos[1]);
	pos[2] = 0.28;

	dRotation2dsRotationMatrix(0, rotmat);

	dsSetColor (f_color_red, f_color_green, f_color_blue);
	dsSetTexture (DS_NONE);
	dsDrawCylinderD((double *) &pos, 
			(double *) &rotmat, 
			CEpuck::CHASSIS_RADIUS,
			CEpuck::CHASSIS_RADIUS);


	/* NEST PROJECTION SENSOR */
	if(pc_epuck->GetNestProjection())
	{
		dVector2 vNestPos;
		long int aux;
		if ( pc_epuck->GetNestPosition ( &vNestPos , &aux))
		{
			pos[0] = vNestPos.x;
			pos[1] = vNestPos.y;
			pos[2] = 0.22;

			dRotation2dsRotationMatrix(0,rotmat);

			dsSetColor (0.75,0.75,0);
			dsSetTexture(DS_NONE);
			dsDrawCylinderD((double *) &pos, 
					(double *) &rotmat,
					0.02,
					0.02);
		}
	}

	/* PREY PROJECTION SENSOR */
	if(pc_epuck->GetPreyProjection())
	{
		dVector2 vPreyPos;
		long int aux;
		if ( pc_epuck->GetPreyPosition ( &vPreyPos , &aux))
		{
			pos[0] = vPreyPos.x;
			pos[1] = vPreyPos.y;
			pos[2] = 0.22;

			dRotation2dsRotationMatrix(0,rotmat);

			dsSetColor (0,0.5,0);
			dsSetTexture(DS_NONE);
			dsDrawCylinderD((double *) &pos, 
					(double *) &rotmat,
					0.02,
					0.02);
		}
	} 
	/* END DRAW PROJECTION */
	

	/* DRAW ROBOT HEADING */
	dVector2 vOrigin,vEnd;

	//Draws arrow indicating epuck heading
	vOrigin=pc_epuck->GetPosition();
	float fXend = CEpuck::CHASSIS_RADIUS * 2 * cos(pc_epuck->GetRotation());
	float fYend = CEpuck::CHASSIS_RADIUS * 2 * sin(pc_epuck->GetRotation());
	vEnd.x=fXend;
	vEnd.y=fYend;
	dVec2Add(vEnd,vOrigin,vEnd);
	Draw2DLine(vOrigin,vEnd,0.28, 0,0,1);	
	/* END DRAW ROBOT HEADING */

	/* Draw Ir Sensors */
	if(m_bDrawIRSensors){

		CSensor* pcProxSensor=pc_epuck->GetSensor(SENSOR_PROXIMITY);			
		if(pcProxSensor!=NULL){ 
			float fMaxRange=((CEpuckProximitySensor*)pcProxSensor)->GetMaxRange()+CEpuck::CHASSIS_RADIUS;
			const double* fIRHeadings=((CEpuckProximitySensor*)pcProxSensor)->GetSensorDirections();
			//Draws the proximity sensors

			for(int i=0;i<pcProxSensor->GetNumberOfInputs();i++){
				fXend=fMaxRange*cos(pc_epuck->GetRotation()+fIRHeadings[i]);
				fYend=fMaxRange*sin(pc_epuck->GetRotation()+fIRHeadings[i]);
				vEnd.x=fXend;
				vEnd.y=fYend;
				dVec2Add(vEnd,vOrigin,vEnd);
				Draw2DLine(vOrigin,vEnd,0.281,1,0,0);
			}
		}
	}//if draw ir sensors


	/* Draw Light Sensors */
	if(m_bDrawLightSensors){

		CSensor* pcLightSensor=pc_epuck->GetSensor(SENSOR_LIGHT);			
		if(pcLightSensor!=NULL){ 
			float fMaxRange=((CLightSensor*)pcLightSensor)->GetMaxRange()+CEpuck::CHASSIS_RADIUS;
			const double* fIRHeadings=((CLightSensor*)pcLightSensor)->GetSensorDirections();

			for(int i=0;i<pcLightSensor->GetNumberOfInputs();i++){
				fXend=fMaxRange*cos(pc_epuck->GetRotation()+fIRHeadings[i]);
				fYend=fMaxRange*sin(pc_epuck->GetRotation()+fIRHeadings[i]);
				vEnd.x=fXend;
				vEnd.y=fYend;
				dVec2Add(vEnd,vOrigin,vEnd);
				Draw2DLine(vOrigin,vEnd,0.28,1,1,0);
			}
		}
	}//if draw light sensors
	
	/* Draw Real Light Sensors */
  if(m_bDrawLightSensors){

		CSensor* pcLightSensor=pc_epuck->GetSensor(SENSOR_REAL_LIGHT);			
		if(pcLightSensor!=NULL){ 
			float fMaxRange=((CRealLightSensor*)pcLightSensor)->GetMaxRange()+CEpuck::CHASSIS_RADIUS;
			const double* fIRHeadings=((CRealLightSensor*)pcLightSensor)->GetSensorDirections();

			for(int i=0;i<pcLightSensor->GetNumberOfInputs();i++){
				fXend=fMaxRange*cos(pc_epuck->GetRotation()+fIRHeadings[i]);
				fYend=fMaxRange*sin(pc_epuck->GetRotation()+fIRHeadings[i]);
				vEnd.x=fXend;
				vEnd.y=fYend;
				dVec2Add(vEnd,vOrigin,vEnd);
				Draw2DLine(vOrigin,vEnd,0.28,1,1,0);
			}
		}
	}//if draw real light sensors
	
	/* Draw Blue Light Sensors */
	//if(m_bDrawBlueLightSensors){

		//CSensor* pcBlueLightSensor=pc_epuck->GetSensor(SENSOR_BLUE_LIGHT);			
		//if(pcBlueLightSensor!=NULL){ 
			//float fMaxRange=((CBlueLightSensor*)pcBlueLightSensor)->GetMaxRange()+CEpuck::CHASSIS_RADIUS;
			//const double* fIRHeadings=((CBlueLightSensor*)pcBlueLightSensor)->GetSensorDirections();

			//for(int i=0;i<pcBlueLightSensor->GetNumberOfInputs();i++){
				//fXend=fMaxRange*cos(pc_epuck->GetRotation()+fIRHeadings[i]);
				//fYend=fMaxRange*sin(pc_epuck->GetRotation()+fIRHeadings[i]);
				//vEnd.x=fXend;
				//vEnd.y=fYend;
				//dVec2Add(vEnd,vOrigin,vEnd);
				//Draw2DLine(vOrigin,vEnd,0.28,0,0,1);
			//}
		//}
	//}//if draw light sensors
	
  /* Draw Real Blue Light Sensors */
  if(m_bDrawBlueLightSensors){

		CSensor* pcLightSensor=pc_epuck->GetSensor(SENSOR_REAL_BLUE_LIGHT);			
		if(pcLightSensor!=NULL){ 
			float fMaxRange=((CRealBlueLightSensor*)pcLightSensor)->GetMaxRange()+CEpuck::CHASSIS_RADIUS;
			const double* fIRHeadings=((CRealBlueLightSensor*)pcLightSensor)->GetSensorDirections();

			for(int i=0;i<pcLightSensor->GetNumberOfInputs();i++){
				fXend=fMaxRange*cos(pc_epuck->GetRotation()+fIRHeadings[i]);
				fYend=fMaxRange*sin(pc_epuck->GetRotation()+fIRHeadings[i]);
				vEnd.x=fXend;
				vEnd.y=fYend;
				dVec2Add(vEnd,vOrigin,vEnd);
				Draw2DLine(vOrigin,vEnd,0.28,0,0,1);
			}
		}
	}//if draw real light sensors
	
	/* Draw Red Light Sensors */
	//if(m_bDrawRedLightSensors){

		//CSensor* pcRedLightSensor=pc_epuck->GetSensor(SENSOR_RED_LIGHT);			
		//if(pcRedLightSensor!=NULL){ 
			//float fMaxRange=((CRedLightSensor*)pcRedLightSensor)->GetMaxRange()+CEpuck::CHASSIS_RADIUS;
			//const double* fIRHeadings=((CRedLightSensor*)pcRedLightSensor)->GetSensorDirections();

			//for(int i=0;i<pcRedLightSensor->GetNumberOfInputs();i++){
				//fXend=fMaxRange*cos(pc_epuck->GetRotation()+fIRHeadings[i]);
				//fYend=fMaxRange*sin(pc_epuck->GetRotation()+fIRHeadings[i]);
				//vEnd.x=fXend;
				//vEnd.y=fYend;
				//dVec2Add(vEnd,vOrigin,vEnd);
				//Draw2DLine(vOrigin,vEnd,0.28,0.5,0,0);
			//}
		//}
	//}//if draw light sensors
  
  /* Draw Real Red Light Sensors */
  if(m_bDrawRedLightSensors){

		CSensor* pcLightSensor=pc_epuck->GetSensor(SENSOR_REAL_RED_LIGHT);			
		if(pcLightSensor!=NULL){ 
			float fMaxRange=((CRealRedLightSensor*)pcLightSensor)->GetMaxRange()+CEpuck::CHASSIS_RADIUS;
			const double* fIRHeadings=((CRealRedLightSensor*)pcLightSensor)->GetSensorDirections();

			for(int i=0;i<pcLightSensor->GetNumberOfInputs();i++){
				fXend=fMaxRange*cos(pc_epuck->GetRotation()+fIRHeadings[i]);
				fYend=fMaxRange*sin(pc_epuck->GetRotation()+fIRHeadings[i]);
				vEnd.x=fXend;
				vEnd.y=fYend;
				dVec2Add(vEnd,vOrigin,vEnd);
				Draw2DLine(vOrigin,vEnd,0.28,0.5,0,0);
			}
		}
	}//if draw real light sensors

	/* Draw Contact Sensors */
	if(m_bDrawContactSensors){

		CSensor* pcContactSensor=pc_epuck->GetSensor(SENSOR_CONTACT);			
		if(pcContactSensor!=NULL){ 
			float fMaxRange=((CContactSensor*)pcContactSensor)->GetMaxRange()+CEpuck::CHASSIS_RADIUS;
			const double* fIRHeadings=((CContactSensor*)pcContactSensor)->GetSensorDirections();

			for(int i=0;i<pcContactSensor->GetNumberOfInputs();i++){
				fXend=fMaxRange*cos(pc_epuck->GetRotation()+fIRHeadings[i]);
				fYend=fMaxRange*sin(pc_epuck->GetRotation()+fIRHeadings[i]);
				vEnd.x=fXend;
				vEnd.y=fYend;
				dVec2Add(vEnd,vOrigin,vEnd);
				Draw2DLine(vOrigin,vEnd,0.282,1,1,1);
			}
		}
	}//if draw contact sensors

	//if(m_bDrawCamera){
		//CSensor* pcCameraSensor = pc_epuck->GetSensor(SENSOR_SIMPLE_MODULAR_CAMERA);
		//if(pcCameraSensor!=NULL){
			//float fMaxRange=((CSimpleModularCamera*)pcCameraSensor)->GetRange();
			//float fSpan=((CSimpleModularCamera*)pcCameraSensor)->GetSpan();
			//dVector2 vOrigin=pc_epuck->GetPosition();
			//dVector2 vEnd;
			//float fXend,fYend;
			//int numSect=((CSimpleModularCamera*)pcCameraSensor)->GetSectorsNumber();
			//double fHeading=-fSpan/2.0+fSpan/(2.0*numSect);
			//double* fLastReadings=((CSimpleModularCamera*)pcCameraSensor)->GetComputedSensorReadings();
			//for(int i=0;i<numSect;i++){
				//fHeading+=fSpan/numSect;
				//fXend=fMaxRange*cos(pc_epuck->GetRotation()-fHeading);
				//fYend=fMaxRange*sin(pc_epuck->GetRotation()-fHeading);
				//vEnd.x=fXend;
				//vEnd.y=fYend;
				//dVec2Add(vEnd,vOrigin,vEnd);
				////printf("RENDER, readings[%d]: %f\n",i,fLastReadings[i]);
				//if(fLastReadings[i]==0.0){
					//Draw2DLine(vOrigin,vEnd,1,0,0);
				//}else if(fLastReadings[i]==0.5){
					//Draw2DLine(vOrigin,vEnd,0,1,0);	
				//}else{
					//Draw2DLine(vOrigin,vEnd,0,0,0);		
				//}
			//}
		//}
	//}

	//if(m_bDrawSound){
		//CSensor* pcSoundSensor=pc_epuck->GetSensor(SENSOR_SOUND_SIMPLE);
		//if(pcSoundSensor!=NULL){
			//double curReading=((CSimpleSoundSensor*)pcSoundSensor)->GetComputedSensorReadings()[0];
			//if(curReading>0.5){
				//pos[2]=0.1;
				//dsSetColor(0.0,0.0,1.0);
				//dsDrawCylinderD((double *) &pos, 
						//(double *) &rotmat, 
						//0.01,
						//2*CEpuck::CHASSIS_RADIUS);
			//}
		//}
	//}


	if(pc_epuck->GetGripperPresence())
	{
		DrawGripper(pc_epuck, 0.0, 1.0, 0.0);
	}
}

/******************************************************************************/
/******************************************************************************/

void CSimpleDrawStuffRender::DrawGripper(CEpuck* pc_epuck, float f_color_red, float f_color_green, float f_color_blue)
{
	double epuckX, epuckY, epuckRot;
	pc_epuck->GetPosition(&epuckX, &epuckY);
	epuckRot = pc_epuck->GetRotation();
	double epuckRadius = CEpuck::CHASSIS_RADIUS;
	double f_gripper_angle = pc_epuck->GetGripperAngle();
	double f_gripper_length = pc_epuck->GetGripperLength();

	dsSetColor(f_color_red, f_color_green, f_color_blue);

	float v_origRightGripper[3];
	float v_destRightGripper[3];
	float v_origLeftGripper[3];
	float v_destLeftGripper[3];

	// DRAW THE RIGHT GRIPPER (LINE)
	v_origRightGripper[0] = epuckX + epuckRadius * cos(epuckRot - f_gripper_angle);
	v_origRightGripper[1] = epuckY + epuckRadius * sin(epuckRot - f_gripper_angle);
	v_origRightGripper[2] = 0.28;
	v_destRightGripper[0] = v_origRightGripper[0] + f_gripper_length * cos(epuckRot);
	v_destRightGripper[1] = v_origRightGripper[1] + f_gripper_length * sin(epuckRot);
	v_destRightGripper[2] = v_origRightGripper[2];
	dsDrawLine(v_origRightGripper, v_destRightGripper);

	// DRAW THE LEFT GRIPPER (LINE)
	v_origLeftGripper[0] = epuckX + epuckRadius * cos(epuckRot + f_gripper_angle);
	v_origLeftGripper[1] = epuckY + epuckRadius * sin(epuckRot + f_gripper_angle);
	v_origLeftGripper[2] = 0.28;
	v_destLeftGripper[0] = v_origLeftGripper[0] + f_gripper_length * cos(epuckRot);
	v_destLeftGripper[1] = v_origLeftGripper[1] + f_gripper_length * sin(epuckRot);
	v_destLeftGripper[2] = v_origLeftGripper[2];
	dsDrawLine(v_origLeftGripper, v_destLeftGripper);

/*
	// DRAW A BOX
	dsRotationMatrix rotmat;
	double pos[3];
	double sides[3];
	dRotation2dsRotationMatrix(0, rotmat);

	//Gripper sizes :
	sides[0] = pc_epuck->GetGripperLength();	//x
	sides[1] = 0.01;				//y
	sides[2] = 0.01;				//z

	//Right gripper's bottom coordinates :
	pos[0] = epuckX + CEpuck::CHASSIS_RADIUS * cos(epuckRot-pc_epuck->GetGripperAngle());
	pos[1] = epuckY + CEpuck::CHASSIS_RADIUS * sin(epuckRot-pc_epuck->GetGripperAngle());
	pos[2] = 0.28;
	//printf("pos[0]:%f , pos[1]:%f , pos[2]:%f , epuckRot:%f , CEpuck::GRIPPER_ANGLE:%f\n", pos[0], pos[1], pos[2], epuckRot, pc_epuck->GetGripperAngle());
	dsDrawBoxD(pos, rotmat, sides);

	//Left gripper's bottom coordinates :
	pos[0] = epuckX + CEpuck::CHASSIS_RADIUS * cos(-(epuckRot-pc_epuck->GetGripperAngle()));
	pos[1] = epuckY + CEpuck::CHASSIS_RADIUS * sin(-(epuckRot-pc_epuck->GetGripperAngle()));
	pos[2] = 0.28;
	//printf("pos[0]:%f , pos[1]:%f , pos[2]:%f , epuckRot:%f , CEpuck::GRIPPER_ANGLE:%f\n", pos[0], pos[1], pos[2], epuckRot, pc_epuck->GetGripperAngle());
	dsDrawBoxD(pos, rotmat, sides);
*/
}

/******************************************************************************/
/******************************************************************************/

void CSimpleDrawStuffRender::DrawPuck(CPuck* pc_puck, float f_color_red, float f_color_green, float f_color_blue){

    dsRotationMatrix rotmat;
    double pos[3];
    double sides[3];

    pc_puck->GetPosition(&pos[0],&pos[1]);
    pos[2] = 0.28;

	//printf("DRAWSTUFF RENDER, Puck position: %f - %f \n",pos[0],pos[1]);
	//fflush(stdout);
    dRotation2dsRotationMatrix(0, rotmat);
    
    dsSetColor (f_color_red, f_color_green, f_color_blue);
    dsSetTexture (DS_NONE);
    /*dsDrawSphereD ((double *) &pos, 
                   (double *) &rotmat, 
                   CPuck::CHASSIS_RADIUS);
	*/
	dsDrawCylinderD((double *) &pos, 
                   (double *) &rotmat, 
					CPuck::CHASSIS_RADIUS,
                   CPuck::CHASSIS_RADIUS);
}

/******************************************************************************/
/******************************************************************************/


void CSimpleDrawStuffRender::DrawIndicator(float f_x, float f_y, float f_color_red, float f_color_green, float f_color_blue)
{
    dsRotationMatrix rotmat;
    double pos[3];
    dRotation2dsRotationMatrix(0, rotmat);    

    pos[0] = f_x;
    pos[1] = f_y;
    pos[2] = 0.45 + sin(m_fIndicatorSinusAngle) / 20;

    dsSetColor (f_color_red, f_color_green, f_color_blue);
    dsDrawSphereD((double*) &pos, 
                  (double*) &rotmat, 
                  0.03);    
}

/******************************************************************************/
/******************************************************************************/

void CSimpleDrawStuffRender::Draw2DLine(dVector2 v_orig, dVector2 v_dest, float height, float color_red, float color_green, float color_blue ) {
  double pos1[3] = {v_orig.x, v_orig.y, height};//0.35
  double pos2[3] = {v_dest.x, v_dest.y, height};//0.35
  dsSetColorAlpha(color_red, color_green, color_blue, 0.2);
  dsDrawLineD( pos1, pos2 );
}
/******************************************************************************/
/******************************************************************************/
/*
void CSimpleDrawStuffRender::DrawColoredWall(CColoredWall* pc_wall){
	
}

/******************************************************************************/
/******************************************************************************/
/*
void CSimpleDrawStuffRender::DrawLight(CLightObject* pc_light){

	dsRotationMatrix rotmat;
    double pos[3];
	
	if(pc_light->GetStatus()){
		dsSetColor(1.0, 0.0, 0.0);
	}else{
		dsSetColor(0.0, 0.0, 1.0);
	}
		
	pos[0]=(pc_light->GetPosition()).x;
	pos[1]=(pc_light->GetPosition()).y;
	pos[2]=0.28;
	
		
	dsDrawSphereD ((double *) &pos, 
                  (double *) &rotmat, 
                  0.02);
}

/******************************************************************************/
/******************************************************************************/
/*
void CSimpleDrawStuffRender::DrawPalet(CPalet* pc_palet, float f_color_red, float f_color_green, float f_color_blue)
{
    dsRotationMatrix rotmat;
    double pos[3];
    double sides[3];
    dsSetTexture(DS_NONE);

    double fSizeX;
    double fSizeY;
    
    pc_palet->GetSize(&fSizeX, &fSizeY);

    sides[0] = fSizeX;
    sides[1] = fSizeY;

    // Get the size of the palet:
    pc_palet->GetSize(&sides[0], &sides[1]);
    sides[2] = 0.01;

    dRotation2dsRotationMatrix(pc_palet->GetRotation(), rotmat);
    dsSetColor(f_color_red, f_color_green, f_color_blue);
    
    pos[0] = pc_palet->GetPosition().x;
    pos[1] = pc_palet->GetPosition().y;
    pos[2] = 0.205 + pc_palet->GetHeight();

    dsDrawBoxD ((double *) &pos, (double *) &rotmat, (double*) &sides);

    TMovableObjectLedsVector* pvecLeds = pc_palet->GetAllLeds();
    
    // Draw the vertical bars:
    sides[0] = 0.02;
    sides[1] = 0.02;
    sides[2] = 0.065;

    pos[2] = 0.2375 + pc_palet->GetHeight();
    dVector2 vBarPosition;
    vBarPosition.x = fSizeX / 2 - 0.01;
    vBarPosition.y = fSizeY / 2 - 0.01;
    dVec2Rotate(pc_palet->GetRotation(), vBarPosition);
    dVec2Add(vBarPosition, vBarPosition, pc_palet->GetPosition());
    pos[0] = vBarPosition.x;
    pos[1] = vBarPosition.y;
    dsDrawBoxD ((double *) &pos, (double *) &rotmat, (double*) &sides);

    vBarPosition.x = -fSizeX / 2 + 0.01;
    vBarPosition.y = fSizeY / 2 - 0.01;
    dVec2Rotate(pc_palet->GetRotation(), vBarPosition);
    dVec2Add(vBarPosition, vBarPosition, pc_palet->GetPosition());
    pos[0] = vBarPosition.x;
    pos[1] = vBarPosition.y;
    dsDrawBoxD ((double *) &pos, (double *) &rotmat, (double*) &sides);

    vBarPosition.x = fSizeX / 2 - 0.01;
    vBarPosition.y = -fSizeY / 2 + 0.01;
    dVec2Rotate(pc_palet->GetRotation(), vBarPosition);
    dVec2Add(vBarPosition, vBarPosition, pc_palet->GetPosition());
    pos[0] = vBarPosition.x;
    pos[1] = vBarPosition.y;
    dsDrawBoxD ((double *) &pos, (double *) &rotmat, (double*) &sides);
    
    vBarPosition.x = -fSizeX / 2 + 0.01;
    vBarPosition.y = -fSizeY / 2 + 0.01;
    dVec2Rotate(pc_palet->GetRotation(), vBarPosition);
    dVec2Add(vBarPosition, vBarPosition, pc_palet->GetPosition());
    pos[0] = vBarPosition.x;
    pos[1] = vBarPosition.y;
    dsDrawBoxD ((double *) &pos, (double *) &rotmat, (double*) &sides);

    // Draw the horizontal bars:
    dsSetColor(f_color_red * 0.9, f_color_green * 0.9, f_color_blue * 0.9);
    sides[0] = fSizeX;
    sides[1] = 0.02;
    sides[2] = 0.02;

    pos[2] = 0.255 + pc_palet->GetHeight();
    vBarPosition.x = 0;
    vBarPosition.y = fSizeY / 2 - 0.01;
    dVec2Rotate(pc_palet->GetRotation(), vBarPosition);
    dVec2Add(vBarPosition, vBarPosition, pc_palet->GetPosition());
    pos[0] = vBarPosition.x;
    pos[1] = vBarPosition.y;
    dsDrawBoxD ((double *) &pos, (double *) &rotmat, (double*) &sides);
    
    vBarPosition.x = 0;
    vBarPosition.y = -fSizeY / 2 + 0.01;
    dVec2Rotate(pc_palet->GetRotation(), vBarPosition);
    dVec2Add(vBarPosition, vBarPosition, pc_palet->GetPosition());
    pos[0] = vBarPosition.x;
    pos[1] = vBarPosition.y;
    dsDrawBoxD ((double *) &pos, (double *) &rotmat, (double*) &sides);

    sides[0] = 0.02;
    sides[1] = fSizeY;
    sides[2] = 0.02;

    vBarPosition.x = fSizeX / 2 - 0.01;
    vBarPosition.y = 0;
    dVec2Rotate(pc_palet->GetRotation(), vBarPosition);
    dVec2Add(vBarPosition, vBarPosition, pc_palet->GetPosition());
    pos[0] = vBarPosition.x;
    pos[1] = vBarPosition.y;
    dsDrawBoxD ((double *) &pos, (double *) &rotmat, (double*) &sides);
    
    vBarPosition.x = -fSizeX / 2 + 0.01;
    vBarPosition.y = 0;
    dVec2Rotate(pc_palet->GetRotation(), vBarPosition);
    dVec2Add(vBarPosition, vBarPosition, pc_palet->GetPosition());
    pos[0] = vBarPosition.x;
    pos[1] = vBarPosition.y;
    dsDrawBoxD ((double *) &pos, (double *) &rotmat, (double*) &sides);

    // Draw leds (use boxes to make it faster)
    for (int i = 0; i < pvecLeds->size(); i++)
    {
        sides[0] = 0.01;
        sides[1] = 0.01;
        sides[2] = 0.01;

        pos[0] = (*pvecLeds)[i]->vAbsolutePosition.x;
        pos[1] = (*pvecLeds)[i]->vAbsolutePosition.y;
        pos[2] = 0.262 + pc_palet->GetHeight();

        unsigned int unLedColor = (*pvecLeds)[i]->cColor;
        switch (unLedColor)
        {
        case LED_COLOR_BLACK  : dsSetColor (0, 0, 0); break;
        case LED_COLOR_RED    : dsSetColor (1, 0, 0); break;
        case LED_COLOR_GREEN  : dsSetColor (0, 1, 0); break;
        case LED_COLOR_BLUE   : dsSetColor (0, 0, 1); break;
        case LED_COLOR_YELLOW : dsSetColor (1, 0.8, 0); break;
        case LED_COLOR_WHITE  : dsSetColor (1, 1, 1); break;
        default:
            ERROR1("Unknown led color %d", unLedColor);
        }

        dsDrawBoxD ((double *) &pos, (double *) &rotmat, (double*) &sides);
    }


//     CRectangleCollisionObject* pcCollisionObject = (CRectangleCollisionObject*) pc_palet->GetCollisionObject();
//     dRotation2dsRotationMatrix(pcCollisionObject->GetRotation(), rotmat);

//     pos[2] = 0.3 + pc_palet->GetHeight();

//     dsSetColor (1, 1, 1);

//     pos[0] = pcCollisionObject->m_vCornerX1Y1.x;
//     pos[1] = pcCollisionObject->m_vCornerX1Y1.y;
//     dsDrawBoxD ((double *) &pos, (double *) &rotmat, (double*) &sides);

//     pos[0] = pcCollisionObject->m_vCornerX1Y2.x;
//     pos[1] = pcCollisionObject->m_vCornerX1Y2.y;
//     dsDrawBoxD ((double *) &pos, (double *) &rotmat, (double*) &sides);

//     pos[0] = pcCollisionObject->m_vCornerX2Y1.x;
//     pos[1] = pcCollisionObject->m_vCornerX2Y1.y;
//     dsDrawBoxD ((double *) &pos, (double *) &rotmat, (double*) &sides);

//     pos[0] = pcCollisionObject->m_vCornerX2Y2.x;
//     pos[1] = pcCollisionObject->m_vCornerX2Y2.y;
//     dsDrawBoxD ((double *) &pos, (double *) &rotmat, (double*) &sides);

}

/******************************************************************************/
/******************************************************************************/

/*
CSimpleDrawStuffRender::CSimpleDrawStuffRender(CSimulator* pc_simulator, int argc, char** argv,
                                               CArguments* pc_render_arguments) :
    CRender(pc_simulator, pc_render_arguments),
    m_argc(argc), 
    m_argv(argv), 
    m_fSimulationFrames(0), 
    m_pcFollowSbot(NULL), 
    m_bEnableFollowSbot(false),
    m_bDisplayPrompt(true),
    m_bPauseAtStart(true),
    m_bUseCustomTextureForGround(false)
{
    if (m_pcRendererArguments->GetArgumentIsDefined("help"))
    {
        printf("SimpleDrawStuffRenderer Arguments: \n"
               "----------------------------------------------------------------------\n"
               "  winsizex #:           Render window size x   [640]\n"
               "  winsizey #:           Render window size y   [480]\n"
               "  disabletextures:      Do not use textures\n"
               "  enabletextures:       Use textures             [*]\n"
               "  disablepauseatstart:  Do not pause at start-up    \n"
               "  enablepauseatstart:   Pause at start-up        [*]\n"
               "----------------------------------------------------------------------\n"
            );
        exit(0);              
    }


    m_unWindowSizeX = 640;
    if (m_pcRendererArguments->GetArgumentIsDefined("winsizex"))
    {
        m_unWindowSizeX = m_pcRendererArguments->GetArgumentAsInt("winsizex");
    }

    m_unWindowSizeY = 480;
    if (m_pcRendererArguments->GetArgumentIsDefined("winsizey"))
    {
        m_unWindowSizeY = m_pcRendererArguments->GetArgumentAsInt("winsizey");
    }

    m_bUseTextures = true;
    if (m_pcRendererArguments->GetArgumentIsDefined("disabletextures"))
    {
        m_bUseTextures = false;
    }

    if (m_pcRendererArguments->GetArgumentIsDefined("enabletextures"))
    {
        m_bUseTextures = true;
    }

    if (m_pcRendererArguments->GetArgumentIsDefined("enablepauseatstart"))
    {
        m_bPauseAtStart = true;
    }

    if (m_pcRendererArguments->GetArgumentIsDefined("disablepauseatstart"))
    {
        m_bPauseAtStart = false;
    }

    m_pcSoundEmitter = NULL;
}


/******************************************************************************/
/******************************************************************************/
/*
void CSimpleDrawStuffRender::DrawSbot(CSbot* pc_sbot, float f_color_red, float f_color_green, float f_color_blue)
{
    dsRotationMatrix rotmat;
    double pos[3];
    double sides[3];

    pos[0] = pc_sbot->GetPosition().x;
    pos[1] = pc_sbot->GetPosition().y;
    pos[2] = 0.28;

//    PRINTVEC2("Sbot pos: ", pc_sbot->GetPosition());

    dRotation2dsRotationMatrix(pc_sbot->GetTurretRotation(), rotmat);
    
    // Turret:
    dsSetColor (f_color_red, f_color_green, f_color_blue);
    dsSetTexture (DS_NONE);
    dsDrawSphereD ((double *) &pos, 
                   (double *) &rotmat, 
                   CSbot::TURRET_RADIUS);
    

    // Draw gripper socket:
    dVector2 vGripper;
    vGripper.x = CSbot::GRIPPER_SOCKET_POS[0];
    vGripper.y = CSbot::GRIPPER_SOCKET_POS[1];
    dVec2Rotate(pc_sbot->GetTurretRotation(), vGripper);

    pos[0] = vGripper.x + pc_sbot->GetPosition().x;
    pos[1] = vGripper.y + pc_sbot->GetPosition().y;
    pos[2] = 0.28;

    sides[0] = CSbot::GRIPPER_SOCKET_LENGTH;
    sides[1] = CSbot::GRIPPER_SOCKET_WIDTH;
    sides[2] = 0.015;
    
    dsSetColor (0, 0, 0);
    dsDrawBoxD ((double *) &pos, (double *) &rotmat, (double*) &sides);


    // Draw gripper elevation socket::
    dVector2 vSocketElevation = { 0.02 / 2, 0 };
    dVec2Rotate(-pc_sbot->GetGripperElevationAngle(), vSocketElevation);
//    vSocketElevation.x += 0.01;

    dVector2 vSocketPos;
    vSocketPos.x = CSbot::GRIPPER_SOCKET_POS[0] + vSocketElevation.x ;
    vSocketPos.y = CSbot::GRIPPER_SOCKET_POS[1];
    dVec2Rotate(pc_sbot->GetTurretRotation(), vSocketPos);

    dTwoAngleRotation2dsRotationMatrix((pc_sbot->GetTurretRotation()), -pc_sbot->GetGripperElevationAngle(), rotmat);

    pos[0] = vSocketPos.x + pc_sbot->GetPosition().x;
    pos[1] = vSocketPos.y + pc_sbot->GetPosition().y;
    pos[2] = 0.28 - vSocketElevation.y;

    sides[0] = 0.02;
    sides[1] = 0.02;
    sides[2] = 0.015;
    
    dsSetColor (0, 0, 0);
    dsDrawBoxD ((double *) &pos, (double *) &rotmat, (double*) &sides);

    // Draw gripper claw:
    double fAperture = pc_sbot->GetGripperAperture() * 1.3;
    dVector2 vClawOffsetUp = { CSbot::GRIPPER_CLAW_SIDE_LENGTH / 2, 0 };
    dVec2Rotate(-pc_sbot->GetGripperElevationAngle() + fAperture, vClawOffsetUp);
    vGripper.x = CSbot::GRIPPER_SOCKET_POS[0] + CSbot::GRIPPER_CLAW_OFFSET + vClawOffsetUp.x + vSocketElevation.x * 2;
    vGripper.y = CSbot::GRIPPER_SOCKET_POS[1];
    dVec2Rotate((pc_sbot->GetTurretRotation()), vGripper);

    pos[0] = vGripper.x + pc_sbot->GetPosition().x;
    pos[1] = vGripper.y + pc_sbot->GetPosition().y;
    pos[2] = 0.28 - vClawOffsetUp.y - vSocketElevation.y * 2; //  - fAperture * 0.01

    if (fAperture > 0.99)
        dsSetColor (0, 1, 0);
    else if (fAperture < 0.01)
        dsSetColor (1, 0, 0);
    else 
        dsSetColor(fAperture, fAperture, 0);

    dTwoAngleRotation2dsRotationMatrix((pc_sbot->GetTurretRotation()), -pc_sbot->GetGripperElevationAngle() + fAperture, rotmat);
        

    sides[0] = CSbot::GRIPPER_CLAW_SIDE_LENGTH;
    sides[1] = CSbot::GRIPPER_CLAW_SIDE_LENGTH;
    sides[2] = 0.005;
    
    dsDrawBoxD ((double *) &pos, (double *) &rotmat, (double*) &sides);

    dVector2 vClawOffsetDown = { CSbot::GRIPPER_CLAW_SIDE_LENGTH / 2, 0 };
    dVec2Rotate(-pc_sbot->GetGripperElevationAngle() - fAperture, vClawOffsetDown);
    vGripper.x = CSbot::GRIPPER_SOCKET_POS[0] + CSbot::GRIPPER_CLAW_OFFSET + vClawOffsetDown.x  + vSocketElevation.x * 2;
    vGripper.y = CSbot::GRIPPER_SOCKET_POS[1];
    dVec2Rotate((pc_sbot->GetTurretRotation()), vGripper);

    pos[0] = vGripper.x + pc_sbot->GetPosition().x;
    pos[2] = 0.28 - vClawOffsetDown.y - vSocketElevation.y * 2; // + fAperture * 0.01;

    dTwoAngleRotation2dsRotationMatrix((pc_sbot->GetTurretRotation()), -pc_sbot->GetGripperElevationAngle()  - fAperture, rotmat);
    dsDrawBoxD ((double *) &pos, (double *) &rotmat, (double*) &sides);

    // Draw leds (use boxes to make it faster)
    for (int i = 0; i < 8; i++)
    {
        dVector2 vLedPosition = { CSbot::TURRET_RADIUS, 0 };
        double fAngle = ((2.0 * M_PI) / 8.0) / 2.0 + (double) i * ((2.0 * M_PI) / 8.0) + pc_sbot->GetTurretRotation();
        dVec2Rotate(fAngle, vLedPosition);
        dRotation2dsRotationMatrix(fAngle, rotmat);

        sides[0] = 0.01;
        sides[1] = 0.01;
        sides[2] = 0.01;

        pos[0] = vLedPosition.x + pc_sbot->GetPosition().x;
        pos[1] = vLedPosition.y + pc_sbot->GetPosition().y;
        pos[2] = 0.28;

        unsigned int unLedColor = pc_sbot->GetColoredLed(i);
        switch (unLedColor)
        {
        case LED_COLOR_BLACK  : dsSetColor (0, 0, 0); break;
        case LED_COLOR_RED    : dsSetColor (1, 0, 0); break;
        case LED_COLOR_GREEN  : dsSetColor (0, 1, 0); break;
        case LED_COLOR_BLUE   : dsSetColor (0, 0, 1); break;
        case LED_COLOR_YELLOW : dsSetColor (1, 0.8, 0); break;
        case LED_COLOR_WHITE  : dsSetColor (1, 1, 1); break;
        default:
            ERROR1("Unknown led color %d", unLedColor);
        }

        dsDrawBoxD ((double *) &pos, (double *) &rotmat, (double*) &sides);
    }

    // Chassis:
    dRotation2dsRotationMatrix(pc_sbot->GetChassisRotation(), rotmat);
    pos[0] = pc_sbot->GetPosition().x;
    pos[1] = pc_sbot->GetPosition().y;
    pos[2] = 0.25;

    dsSetColor (0,1,1);
    dsSetTexture (DS_NONE);
    dsDrawSphereD ((double *) &pos, 
                   (double *) &rotmat, 
                   CSbot::CHASSIS_RADIUS);
  
    // Wheels:
    dsSetTexture (DS_NONE);
    dVector2 vLeftWheel;
    vLeftWheel.x = CSbot::TWEEL_POSITION[0];
    vLeftWheel.y = CSbot::TWEEL_POSITION[1];

    dVec2Rotate(pc_sbot->GetChassisRotation(), vLeftWheel);
    pos[0] += vLeftWheel.x;
    pos[1] += vLeftWheel.y;
    pos[2] = CSbot::TWEEL_POSITION[2] + 0.2;

    dsSetColor (0,0,1);
    dsDrawSphereD((double*) &pos, 
                  (double*) &rotmat, 
                  CSbot::TWEEL_RADIUS);

    pos[0] -= 2 * vLeftWheel.x;
    pos[1] -= 2 * vLeftWheel.y;

    dsSetColor (0,1,0);
    dsDrawSphereD((double*) &pos, 
                  (double*) &rotmat, 
                  CSbot::TWEEL_RADIUS);
    
}


/******************************************************************************/
/******************************************************************************/
/*
void CSimpleDrawStuffRender::DrawStoy(CStoy* pc_stoy, float f_color_red, float f_color_green, float f_color_blue)
{

    dsRotationMatrix rotmat;
    double pos[3];

    pos[0] = pc_stoy->GetPosition().x;
    pos[1] = pc_stoy->GetPosition().y;
    pos[2] = 0.25;

//    PRINTVEC2("Sbot pos: ", pc_sbot->GetPosition());

    dRotation2dsRotationMatrix(0, rotmat);
    
    dsSetColor(1.0,1.0,1.0);
    dsSetTexture (DS_NONE);
    dsDrawSphereD ((double *) &pos, 
                   (double *) &rotmat, 
                   CStoy::RADIUS);
    

    if( f_color_red != 0.0 || 
	f_color_green != 0.0 || 
	f_color_blue != 0.0 ) {
      pos[2] = 0.15;
      dsSetColorAlpha(f_color_red, f_color_green, f_color_blue, 0.1);
      dsDrawSphereD ((double *) &pos, 
		     (double *) &rotmat, 
		     CStoy::RADIUS+0.1);
    }
    
}


/******************************************************************************/
/******************************************************************************/


/*
void CSimpleDrawStuffRender::DrawLight(CLightEmitter* pc_light) {
    dsSetTexture(DS_NONE);
    
    dsRotationMatrix rotmat;
    double pos[3];
    dRotation2dsRotationMatrix(0, rotmat);    

    pos[0] = pc_light->GetPosition().x;
    pos[1] = pc_light->GetPosition().y;
    pos[2] = 0.5;
    
    double status = pc_light->GetStatus();
    double fRed, fGreen, fBlue;
    pc_light->GetRGBColor( &fRed, &fGreen, &fBlue );
    dsSetColor(fRed*status, fGreen*status, fBlue*status);
    dsDrawSphereD((double*) &pos, 
                  (double*) &rotmat, 
                  0.05);
}

/******************************************************************************/
/******************************************************************************/
/*
void CSimpleDrawStuffRender::DrawSound(CSoundEmitter* pc_sound)
{
    
    dsRotationMatrix rotmat;
    double pos[3];
    dRotation2dsRotationMatrix(0, rotmat);    

    pos[0] = pc_sound->GetPosition().x;
    pos[1] = pc_sound->GetPosition().y;
    pos[2] = 0.4;

    double intensity = pc_sound->GetStatus()*pc_sound->GetMaxIntensity();
    if( intensity > 0 ) {
      dsSetColor(0.0, 1.0, 0.0);
      dsDrawSphereD((double*) &pos, 
		    (double*) &rotmat, 
		    0.03 + intensity/100);
    }
}


/******************************************************************************/
/******************************************************************************/
/*
void CSimpleDrawStuffRender::DrawIntensitySound(CSoundEmitter* pc_sound, double f_intensity)
{    
  if( pc_sound->GetStatus() > 0 ) {
    dsRotationMatrix rotmat;
    double pos[3];
    dRotation2dsRotationMatrix(0, rotmat);    
    
    pos[0] = pc_sound->GetPosition().x;
    pos[1] = pc_sound->GetPosition().y;
    pos[2] = 0.4;
    
    dsSetColorAlpha(0.0, 1.0, 0.0, 0.1);
    dsDrawSphereD((double*) &pos, 
		  (double*) &rotmat, 
		  f_intensity/10);
  }
}

/******************************************************************************/
/******************************************************************************/


void CSimpleDrawStuffRender::dRotation2dsRotationMatrix ( double angle, double* matrix)
{
	
     matrix[0 * 4 + 2] = matrix[1 * 4 + 2] = matrix[2 * 4 + 0] = matrix[2 * 4 + 1] = 0;                         
     matrix[0 * 4 + 3] = matrix[1 * 4 + 3] = matrix[2 * 4 + 3] = 0;                                             
     matrix[0 * 4 + 0] = matrix[1 * 4 + 1] = cos(angle);                                                        
     matrix[0 * 4 + 1] = -sin(angle);                                                                           
     matrix[1 * 4 + 0] = sin(angle);                                                                            
     matrix[2 * 4 + 2] = 1;					
}
void CSimpleDrawStuffRender::dTwoAngleRotation2dsRotationMatrix(double z , double y, double* matrix )
{
     matrix[0 * 4 + 0] = cos(y)*cos(z);           
		 matrix[0 * 4 + 1] = -sin(z);
		 matrix[0 * 4 + 2] = sin(y)*cos(z);         
		 matrix[0 * 4 + 3] = 0;   
     matrix[1 * 4 + 0] = cos(y)*sin(z);           
		 matrix[1 * 4 + 1] = cos(z);  
		 matrix[1 * 4 + 2] = sin(y)*sin(z);         
		 matrix[1 * 4 + 3] = 0;  
     matrix[2 * 4 + 0] = -sin(y); 
		 matrix[2 * 4 + 1] = 0;  
		 matrix[2 * 4 + 2] = cos(y);                
		 matrix[2 * 4 + 3] = 0;   
}
