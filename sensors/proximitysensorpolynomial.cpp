#include "proximitysensorpolynomial.h"
#include <iostream>
#include <fstream>

/******************************************************************************/
/******************************************************************************/

double   CProximitySensorPolynomial::m_fIRSensorDir[NUM_PROXIMITY_SENSORS] 	= {-0.2967, -0.8727, -1.5708, -2.6180, -3.6652, -4.7124, -5.4105, -5.9865};
//double   CProximitySensorPolynomial::m_fIRSensorDir[NUM_PROXIMITY_SENSORS] 	= {0.2967, 0.8727, 1.5708, 2.6180, 3.6652, 4.7124, 5.4105, 5.9865};
//Second version of the angles: used when i make means of two sensors readings on the real robot
//double   CProximitySensorPolynomial::m_fIRSensorDir[NUM_PROXIMITY_SENSORS] 	= {0.00000, 0.584685, 1.221730, 2.094395, 3.141593, 4.188790, 5.061455, 5.698500};

/******************************************************************************/
/******************************************************************************/

CProximitySensorPolynomial::CProximitySensorPolynomial(const char* pch_name) : 
    CSensor(pch_name, NUM_PROXIMITY_SENSORS )
{
	/*
	printf("proximity polynomial instantiated\n");
	printf("0.00000, ");
	printf("%f, ",33.5*M_PI/180);
	printf("%f, ",70*M_PI/180);
	printf("%f, ",120*M_PI/180);
	printf("%f, ",180*M_PI/180);
	printf("%f, ",240*M_PI/180);
	printf("%f, ",290*M_PI/180);
	printf("%f",326.5*M_PI/180);
	getchar();/**/
	fflush(stdout);
}

/******************************************************************************/
/******************************************************************************/

CProximitySensorPolynomial::~CProximitySensorPolynomial()
{    
}

/******************************************************************************/
/******************************************************************************/
// this function computes the content of m_unIRReadings,
// an array of NUM_PROXIMITY_SENSORS (8) unsigned int values
double* CProximitySensorPolynomial::ComputeSensorReadings(CEpuck* pc_epuck, 
                                                CSimulator* pc_simulator)
{
    for( int j=0 ; j<NUM_PROXIMITY_SENSORS ; j++ )
    {// Initialization to 0
        SetInput(j,0.0);
		m_fIRReadings[j]=0.0;
    }
    double fPosX=0, fPosY=0;
    pc_epuck->GetPosition(&fPosX, &fPosY);

    // first check proximity to epucks
    TEpuckVector* pvEpucks = pc_simulator->GetEpucks();
    for( int nEpucks=0 ; nEpucks<(*pvEpucks).size() ; nEpucks++ )
    {
        if( (*pvEpucks)[nEpucks] != pc_epuck )
        {
            double xCurrentEpuck, yCurrentEpuck, xTargetEpuck, yTargetEpuck;
            pc_epuck->GetPosition(&xCurrentEpuck, &yCurrentEpuck);
            (*pvEpucks)[nEpucks]->GetPosition(&xTargetEpuck, &yTargetEpuck);
            double fDistance;
            fDistance=sqrt(pow(xCurrentEpuck-xTargetEpuck,2)+pow(yCurrentEpuck-yTargetEpuck,2));
			//printf("EPUCKS DISTANCE: %f\n",fDistance);
			//fflush(stdout);
            //subtract robot radius
            //double fTurretRadius = CEpuck::CHASSIS_RADIUS;
            fDistance -= CEpuck::CHASSIS_RADIUS*2;
            if( fDistance < MAXRANGE )
            {
                double fAngleToTarget;
                fAngleToTarget=atan((yTargetEpuck-yCurrentEpuck)/(xTargetEpuck-xCurrentEpuck));
                double fDirection=NormalizeAngle(fAngleToTarget-pc_epuck->GetRotation());
                //ComputeEpuckReading((*pvEpucks)[nEpucks],pc_epuck);
				ComputeEpuckReading(pc_epuck,(*pvEpucks)[nEpucks]);
            }
        }
    }

	//printf("\n\n\n\n\n\nCHECKING WALLS \n");
	//fflush(stdout);
	
    // then check for wall proximity 
    CArena* pcArena = pc_simulator->GetArena();
	
    // then check for wall proximity in a squared arena 
    // and for rectangular obstacles
    if( pcArena->GetArenaType() == ARENA_TYPE_SQUARE )
    {

        double fSizeX, fSizeY;
        unsigned int unNumCellsX, unNumCellsY;
        pcArena->GetSize(&fSizeX,&fSizeY);
        pcArena->GetResolution(&unNumCellsX,&unNumCellsY);
        double fCellSizeX = fSizeX/(double)unNumCellsX;
        double fCellSizeY = fSizeY/(double)unNumCellsY;
        if( fabs(fCellSizeX - fCellSizeY)>1.23456e-7 )
        {
            printf("ERROR: in CProximitySensorPolynomial::ComputeSensorReadings wrong cell size: fSizeX/unNumCellsX:%f , fSizeY/unNumCellsY:%f , diff:%f\n",fCellSizeX,fCellSizeY,fCellSizeX-fCellSizeY);
        }
        double fCellSize = fCellSizeX;
      
        bool bWallEncountered = false;
        double fX = fPosX;
        double fY = fPosY;
    
        double fDistanceToLeftCellBorder 	= ((fX+fSizeX/2)/fCellSize)*fCellSize-(unsigned int)((fX+fSizeX/2)/fCellSize)*fCellSize;
        double fDistanceToRightCellBorder 	= fCellSize - fDistanceToLeftCellBorder;
        double fDistanceToBottomCellBorder 	= ((fY+fSizeY/2)/fCellSize)*fCellSize-(unsigned int)((fY+fSizeY/2)/fCellSize)*fCellSize;
        double fDistanceToTopCellBorder   	= fCellSize - fDistanceToBottomCellBorder;
    
        // check to the left
        while( !bWallEncountered && 
               fX > fPosX-MAXRANGE )
        {
            fX -= fCellSize;
            if( pcArena->GetHeight(fX,fY) == HEIGHT_OBSTACLE )
            {
                bWallEncountered = true;
                double fDistance = fPosX - fX - fDistanceToRightCellBorder;
                if( fDistance < MAXRANGE )
                {
                    double fDirection = NormalizeAngle( M_PI-pc_epuck->GetRotation() );
     				ComputeStraightWallReading(pc_epuck,fDistance,0);
                }
            }
        }
    
        // check to the right
        fX = fPosX;
        bWallEncountered = false;
        while( !bWallEncountered && 
               fX < fPosX+MAXRANGE )
        {
            fX += fCellSize;
            if( pcArena->GetHeight(fX,fY) == HEIGHT_OBSTACLE )
            {
                bWallEncountered = true;
                double fDistance = fX - fPosX - fDistanceToLeftCellBorder;
    // 	  printf("fDistance:%f , fX:%f , ownX:%f , toRight:%f \n",fDistance,fX,fPosX,fDistanceToRightCellBorder);
                if( fDistance < MAXRANGE )
                {
                    double fDirection = NormalizeAngle( -pc_epuck->GetRotation());
                    ComputeStraightWallReading(pc_epuck,fDistance,1);
                }
            }
        }
    
        // check to the top
        fX = fPosX;
        bWallEncountered = false;
        while( !bWallEncountered && 
               fY < fPosY+MAXRANGE)
        {
            fY += fCellSize;
            if( pcArena->GetHeight(fX,fY) == HEIGHT_OBSTACLE )
            {
                bWallEncountered = true;
                double fDistance = fY - fPosY - fDistanceToBottomCellBorder;
    // 	  printf("fDistance:%f , fY:%f , ownY:%f , toTop:%f \n", fDistance,fY,fPosY,fDistanceToTopCellBorder);
                if( fDistance < MAXRANGE )
                {
                    double fDirection = NormalizeAngle( M_PI/2.0 -pc_epuck->GetRotation());
                    ComputeStraightWallReading(pc_epuck,fDistance,2);
                }
            }
        }
    
        // check to the bottom
        fY = fPosY;
        bWallEncountered = false;
        while( !bWallEncountered && 
               fY > fPosY-MAXRANGE )
        {
            fY -= fCellSize;
            if( pcArena->GetHeight(fX,fY) == HEIGHT_OBSTACLE )
            {
                bWallEncountered = true;
                double fDistance = fPosY - fY - fDistanceToTopCellBorder;
    // 	  printf("fDistance:%f , fY:%f , ownY:%f , toBottom:%f \n",fDistance,fY,fPosY,fDistanceToBottomCellBorder);
                if( fDistance < MAXRANGE )
                {
                    double fDirection = NormalizeAngle( -M_PI/2.0 -pc_epuck->GetRotation());
                    ComputeStraightWallReading(pc_epuck,fDistance,3);
                }
            }
        }
    }
	
    // then check for wall proximity in a round arena
	
	for(int i=0;i<NUM_PROXIMITY_SENSORS;i++){
				
		//if(m_fIRReadings[i]<MIN_MEASURE) m_fIRReadings[i]=MIN_MEASURE;
		
		float fNoiseValue;
		if(m_fIRReadings[i]>MIN_MEASURE){
			fNoiseValue=Random::nextDouble(-0.1,0.1)*(MAX_MEASURE-MIN_MEASURE);
			//fNoiseValue=0.0;
		}else{
			fNoiseValue=Random::nextDouble(0.0,0.25)*(MAX_MEASURE-MIN_MEASURE);
			//fNoiseValue=0.0;
		}	
		//printf("Noise: %f \n",fNoiseValue);
		m_fIRReadings[i]+=fNoiseValue;
		
		if(m_fIRReadings[i]<MIN_MEASURE){
			m_fIRReadings[i]=MIN_MEASURE;
		}
		if(m_fIRReadings[i]>MAX_MEASURE){
			m_fIRReadings[i]=MAX_MEASURE;
		}
	
		SetInput(i,m_fIRReadings[i]);
		
		//printf("Curr sensor %d -- value: %f \n",i,m_fIRReadings[i]);
		//fflush(stdout);
		
		/**/
	}


//sleep(1);
	NormalizeReadings();	
    return GetInputs();
}

/******************************************************************************/
/******************************************************************************/

unsigned int CProximitySensorPolynomial::GetType()
{
    return SENSOR_PROXIMITY_POLYNOMIAL;
}

/******************************************************************************/
/******************************************************************************/

const double* CProximitySensorPolynomial::GetIRReadings()
{
    return m_fIRReadings;
}

/******************************************************************************/
/******************************************************************************/

double CProximitySensorPolynomial::GetProximityMaxValue(){
	return CProximitySensorPolynomial::MAX_MEASURE;
}

/******************************************************************************/
/******************************************************************************/

void CProximitySensorPolynomial::NormalizeReadings()
{
    for( int i=0 ; i<m_unNumberOfInputs ; i++ )
    {
        double fNormalizedInput = (m_pfInputs[i]-CProximitySensorPolynomial::MIN_MEASURE)/(CProximitySensorPolynomial::MAX_MEASURE-CProximitySensorPolynomial::MIN_MEASURE);
		if(fNormalizedInput<0) fNormalizedInput=0.0;
		
		//printf("Sensor: %d, value (normalized): %f\n",i,fNormalizedInput);
		//fflush(stdout);	
        SetInput(i,fmin(1,fNormalizedInput));
		//SetInput(i,11.0);
    }
}

/******************************************************************************/
/******************************************************************************/

void CProximitySensorPolynomial::ComputeEpuckReading(CEpuck* p_pcThisEpuck,CEpuck* p_pcTargetEpuck){
	
	
	dVector2 mPos=p_pcThisEpuck->GetPosition();
	dVector2 tPos=p_pcTargetEpuck->GetPosition();
	/*printf("COMPUTE EPUCK READING °°°°°°°°°°°°°°°°°\n");
	printf("	Epuck position: %f  %f \n",mPos.x,mPos.y);
	printf("	Heading: %f\n",p_pcThisEpuck->GetRotation()*180/M_PI);
	printf("	Target epuck's position: %f  %f \n",tPos.x,tPos.y);
	printf("	Heading: %f\n",p_pcTargetEpuck->GetRotation()*180/M_PI);
	fflush(stdout);/**/
	for(int i=0;i<NUM_PROXIMITY_SENSORS;i++){
		//printf("Current sensor:%d    -------------------------------------------------------------\n",i);
		//fflush(stdout);
		dVector2 vSensOrigin;
		float sensorHeading=p_pcThisEpuck->GetRotation()+m_fIRSensorDir[i];
		/*if(i==5){
		printf("	Heading: %f\n",sensorHeading*180/M_PI);
		}/**/	
		float fSensX=CEpuck::CHASSIS_RADIUS*cos(sensorHeading);
		float fSensY=CEpuck::CHASSIS_RADIUS*sin(sensorHeading);
		dVector2 tmp={fSensX,fSensY};
		dVec2Add(vSensOrigin,mPos,tmp);
		//printf("	Position: %f--%f\n",vSensOrigin.x,vSensOrigin.y);
		float fInterX1,fInterY1,fInterX2,fInterY2;
		//printf("	Sensor heading: %f\n",sensorHeading*180/M_PI);
		//printf("	MAXRANGE*cos(sensorHeading): %f\n",MAXRANGE*cos(sensorHeading));
		//printf("	MAXRANGE*sin(sensorHeading): %f\n",MAXRANGE*sin(sensorHeading));
		//fflush(stdout);
		if(CircleLineIntersections(vSensOrigin.x,vSensOrigin.y,vSensOrigin.x+MAXRANGE*cos(sensorHeading),vSensOrigin.y+MAXRANGE*sin(sensorHeading),
								tPos.x,tPos.y,CEpuck::CHASSIS_RADIUS,&fInterX1,&fInterY1,&fInterX2,&fInterY2))
		{
		
			dVector2 vTmp1={fInterX1,fInterY1};
			dVector2 vTmp2={fInterX2,fInterY2};
			//printf("	Intersection point 1: %f  %f \n",fInterX1,fInterY1);
			//printf("	Intersection point 2: %f  %f \n",fInterX2,fInterY2);
			//fflush(stdout);
			dVec2Sub(vTmp1,vSensOrigin,vTmp1)
			dVec2Sub(vTmp2,vSensOrigin,vTmp2)
			
			float d1=dVec2Length(vTmp1);
			float d2=dVec2Length(vTmp2);
			float curRead=0.0;
			
			if(d1<d2){
			//	printf("	Sensor: %d Distance value : %f\n",i,d1);
			//	fflush(stdout);
				if(d1<MAXRANGE){
					curRead=ReadingGivenDistance(d1);
				}
			}else{
			//	printf("	Sensor %d Distance value: %f\n",i,d2);
			//	fflush(stdout);
				if(d2<MAXRANGE){
					curRead=ReadingGivenDistance(d2);
				}	
			}
			if(curRead > m_fIRReadings[i]) m_fIRReadings[i]=curRead;
		}/*else{
			printf("	No intersections for current sensor!\n");	
			fflush(stdout);
		}/**/
		//printf("End sensor %d     ####################################\n",i);
		//fflush(stdout);
	}
}

/******************************************************************************/
/******************************************************************************/

void CProximitySensorPolynomial::ComputeStraightWallReading(CEpuck* p_pcEpuck,float pfDistance,int piWallPosition){
//printf("Compute straight wall reading\n");
	float fWallX1,fWallY1,fWallX2,fWallY2;
	//printf("Straight wall reading\n");
	if(piWallPosition==0){//Wall on the left side
		fWallX1=(p_pcEpuck->GetPosition()).x-pfDistance;
		fWallY1=(p_pcEpuck->GetPosition()).y+MAXRANGE;
		fWallX2=fWallX1;
		fWallY2=fWallY1-2*MAXRANGE;
	}else if(piWallPosition==1){//Wall on the right
		fWallX1=(p_pcEpuck->GetPosition()).x+pfDistance;
		fWallY1=(p_pcEpuck->GetPosition()).y+MAXRANGE;
		fWallX2=fWallX1;
		fWallY2=fWallY1-2*MAXRANGE;		
	}else if(piWallPosition==2){//Wall on top side
		fWallX1=(p_pcEpuck->GetPosition()).x+MAXRANGE;
		fWallY1=(p_pcEpuck->GetPosition()).y+pfDistance;
		fWallX2=fWallX1-2*MAXRANGE;
		fWallY2=fWallY1;
	}else{//Wall on bottom side
		fWallX1=(p_pcEpuck->GetPosition()).x+MAXRANGE;
		fWallY1=(p_pcEpuck->GetPosition()).y-pfDistance;
		fWallX2=fWallX1-2*MAXRANGE;
		fWallY2=fWallY1;		
	}
	
	//printf("WALL SEGMENT: (%f,%f)--(%f,%f)\n",fWallX1,fWallY1,fWallX2,fWallY2);
	dVector2 mPos=p_pcEpuck->GetPosition();
	//printf("Epuck position: %f,%f \n",mPos.x,mPos.y);
	//printf("Epuck heading: %f\n",p_pcEpuck->GetRotation());
	
	for(int i=0;i<NUM_PROXIMITY_SENSORS;i++){
//		printf("SENSOR: %d  -----------------------------------------------\n\n",i);
		dVector2 vSensOrigin;
		float sensorHeading=p_pcEpuck->GetRotation()+m_fIRSensorDir[i];
		sensorHeading=NormalizeAngle(sensorHeading);
		
		float fSensX=CEpuck::CHASSIS_RADIUS*cos(sensorHeading);
		float fSensY=CEpuck::CHASSIS_RADIUS*sin(sensorHeading);
		dVector2 tmp={fSensX,fSensY};
		dVec2Add(vSensOrigin,mPos,tmp);
		
		float fMaxX=vSensOrigin.x+MAXRANGE*cos(sensorHeading);
		float fMaxY=vSensOrigin.y+MAXRANGE*sin(sensorHeading);		
		
		float fIntX,fIntY; //Intersection point between current sensor and given segment (wall)
		bool bIntersectPossible=true;
		//printf("Sensor origin and maxrange: (%f,%f) - (%f,%f)\n",vSensOrigin.x,vSensOrigin.y,fMaxX,fMaxY);
	//	printf("Sensor heading: %f\n",sensorHeading*180/M_PI);
		if(fabs(vSensOrigin.x-fMaxX)>0.001){//Sensor segment not vertical
			float fCoef=(fMaxY-vSensOrigin.y)/(fMaxX-vSensOrigin.x);
			//printf("Computed coefficient: %f\n",fCoef);
			if((piWallPosition==0)||(piWallPosition==1)){//vertical wall
				if(piWallPosition==0){
					//printf("Wall position zero\n");
					if(fWallX1<fMaxX) bIntersectPossible=false;
				}else if(piWallPosition==1){
					//printf("Wall position one\n");
					if(fWallX1>fMaxX) bIntersectPossible=false;
				}
				if(bIntersectPossible){
					fIntX=fWallX1;
					fIntY=vSensOrigin.y+fCoef*(fIntX-vSensOrigin.x);
				}	
			}else{//horizontal wall
				if(piWallPosition==2){
					//printf("Wall position two\n");
					if(fWallY1>fMaxY) bIntersectPossible=false;
				}else{
					//printf("Wall position four\n");
					if(fWallY1<fMaxY) bIntersectPossible=false;
				}
				if(bIntersectPossible){
					fIntY=fWallY1;
					//fIntX=(fIntY-vSensOrigin.y)/fCoef-vSensOrigin.x;
					fIntX=(fIntY-vSensOrigin.y)/fCoef+vSensOrigin.x;
				}
			}
		}else{//Sensor segment is vertical
			if(piWallPosition!=0 && piWallPosition!=1){
				fIntX=vSensOrigin.x;
				fIntY=fWallY1;
			}
		}
		if(bIntersectPossible){
		//	printf("Intersection point: %f , %f\n",fIntX,fIntY);
			float fDistance=sqrt((fIntX-vSensOrigin.x)*(fIntX-vSensOrigin.x)+(fIntY-vSensOrigin.y)*(fIntY-vSensOrigin.y));
	/*		printf("Sensor %d, computed distance: %f\n",i,fDistance);
			fflush(stdout);
		*/	
			float fReading=0.0;
			if(fDistance<MAXRANGE){
				fReading=ReadingGivenDistance(fDistance);
				//printf("Current sensor: %d, distance: %f, reading not normalized: %f\n",i,fDistance,fReading);
				//fflush(stdout);
			}else{
				//printf("Out of range, sensor: %d\n",i);
				//fflush(stdout);
				fReading=0.0;
				}	
			if(fReading>m_fIRReadings[i]) m_fIRReadings[i]=fReading;
			}
		/*else{
			printf("Intersection not possible!!\n");
			}
		*/
	}
}

/******************************************************************************/
/******************************************************************************/

float CProximitySensorPolynomial::ReadingGivenDistance(float distance){

	distance*=DISTANCE_UNIT;
	return (3.377e+03 +(-2.048e+03)*distance + (5.759e+02)*distance*distance + (-7.928e+01)*distance*distance*distance + (5.651e+00)*distance*distance*distance*distance+(-1.998e-01)*distance*distance*distance*distance*distance+(2.766e-03)*distance*distance*distance*distance*distance*distance);
}

/******************************************************************************/
/******************************************************************************/
/*
// TAKEN FROM: http://mathworld.wolfram.com/Circle-LineIntersection.html

bool CProximitySensorPolynomial::CircleLineIntersections(float fX1,float fY1,float fX2,float fY2,float fXCirc,float fYCirc,float fRadius,float *fIntX1,float *fIntY1,float *fIntX2,float *fIntY2)
{
	printf("	CircleLineIntersections -----------------\n");
	printf("	Line: %f,%f  ->  %f,%f \n",fX1,fY1,fX2,fY2);
	printf("	Circle: %f %f    radius: %f \n",fXCirc,fYCirc,fRadius);
	//fflush(stdout);
	
	fX1-=fXCirc;
	fY1-=fYCirc;
	fX2-=fXCirc;
	fY2-=fYCirc;
	
	double dX=fX2-fX1;
	double dY=fY2-fY1;
	
	double dR=sqrt(dX*dX+dY*dY);
	double D=fX1*fY2-fX2*fY1;
	double Delta=fRadius*fRadius*dR*dR-D*D;
	
	if(Delta<=0.0){
		//printf("Delta negativo\n");
		//fflush(stdout);
		return false;	
	}
	
	if(dY>=0.0){
		*fIntX1=(D*dY+(dX*sqrt(Delta)))/(dR*dR);
		*fIntX2=(D*dY-(dX*sqrt(Delta)))/(dR*dR);
		*fIntY1=(-D*dX+(dY*sqrt(Delta)))/(dR*dR);
		*fIntY2=(-D*dX+(dY*sqrt(Delta)))/(dR*dR);
	}else{
		*fIntX1=(D*dY+(-dX*sqrt(Delta)))/(dR*dR);
		*fIntX2=(D*dY-(-dX*sqrt(Delta)))/(dR*dR);
		*fIntY1=(-D*dX+(-dY*sqrt(Delta)))/(dR*dR);
		*fIntY2=(-D*dX+(-dY*sqrt(Delta)))/(dR*dR);
	}
	
	bool bPoint1=true;
	bool bPoint2=true;
	
	if((*fIntX1<(fX1-0.001) && *fIntX1<(fX2-0.001))||(*fIntX1>(fX1+0.001) && *fIntX1>(fX2+0.001))){
		printf("	Intersection point 1 outside range X\n");
		fflush(stdout);
		bPoint1=false;
	}
	if((*fIntX2<(fX1-0.001) && *fIntX2<(fX2-0.001))||(*fIntX2>(fX1+0.001) && *fIntX2>(fX2+0.001))){
		printf("	Intersection point 2 outside range X\n");
		fflush(stdout);
		bPoint2=false;	
	}
	if((*fIntY1<(fY1-0.001) && *fIntY1<(fY2-0.001))||(*fIntY1>(fY1+0.001) && *fIntY1>(fY2+0.001))){
		printf("	Intersection point 1 outside range Y\n");
		fflush(stdout);
		bPoint1=false;
	}
	if((*fIntY2<(fY1-0.001) && *fIntY2<(fY2-0.001))||(*fIntY2>(fY1+0.001) && *fIntY2>(fY2+0.001))){
		printf("	Intersection point 2 outside range Y\n");
		fflush(stdout);
		bPoint2=false;
	}
	
	if(bPoint1 || bPoint2){
		*fIntX1+=fXCirc;
		*fIntX2+=fXCirc;
		*fIntY1+=fYCirc;
		*fIntY2+=fYCirc;
		printf("IN CIRCLE LINE INTERSECTIONS, INTERSECTIONS: (%f;%f) -- (%f;%f)\n",*fIntX1,*fIntY1,*fIntX2,*fIntY2);
		fflush(stdout);
		return true;
	}
	
	printf("	NO INTERSECTIONS (OUT OF RANGE): (%f;%f) -- (%f;%f)\n",*fIntX1+fXCirc,*fIntY1+fYCirc,*fIntX2+fXCirc,*fIntY2+fYCirc);	
	fflush(stdout);
	return false;
}

/******************************************************************************/
/******************************************************************************/

// TAKEN FROM: http://astronomy.swin.edu.au/~pbourke/geometry/ see there for details

//Compute the intersection points between the segment (fX1,fY1)(fX2,fY2) and the circle
//(fXCirc,fYCirc)-fRadius

bool CProximitySensorPolynomial::CircleLineIntersections(float fX1,float fY1,float fX2,float fY2,float fXCirc,float fYCirc,float fRadius,float *fIntX1,float *fIntY1,float *fIntX2,float *fIntY2)
{
	/*
	printf("	CircleLineIntersections -----------------\n");
	printf("	Line: %f,%f  ->  %f,%f \n",fX1,fY1,fX2,fY2);
	printf("	Circle: %f %f    radius: %f \n",fXCirc,fYCirc,fRadius);
	fflush(stdout);
	/**/
	/*
	if((sqrt(fX1*fX1+fY1*fY1)-fRadius<0.000001)&&(sqrt(fX2*fX2+fY2*fY2)-fRadius<0.000001)) {
		
		printf("AL primo\n");
		return false;}/**/
		
    double a,b,c;
    double bb4ac;
	double dmu1,dmu2;
    float fTmpX,fTmpY;	

    fTmpX=fX2-fX1;
	fTmpY=fY2-fY1;
	
	a=fTmpX*fTmpX + fTmpY*fTmpY;
    b=2*( fTmpX * (fX1 - fXCirc) + fTmpY * (fY1 - fYCirc) );
	c=fXCirc*fXCirc+fYCirc*fYCirc;
	c+=fX1*fX1+fY1*fY1;	
	c-=2*(fXCirc*fX1 + fYCirc*fY1);
	c-=fRadius*fRadius;
    bb4ac = b * b - 4 * a * c;
	
   if (fabs(a) < 0.00001 || bb4ac <= 0) {
      dmu1 = 0;
      dmu2 = 0;
	  // printf("bb4ac negativo\n");
	  // fflush(stdout);
      return(false);
   }

   dmu1 = (-b + sqrt(bb4ac)) / (2 * a);
   dmu2 = (-b - sqrt(bb4ac)) / (2 * a);
   
   
   *fIntX1=fX1 + dmu1*(fX2-fX1);
   *fIntX2=fX1 + dmu2*(fX2-fX1);
   
   *fIntY1=fY1 + dmu1*(fY2-fY1);
   *fIntY2=fY1 + dmu2*(fY2-fY1);
  
   	bool bPoint1=true;
	bool bPoint2=true;
	
	if((*fIntX1<(fX1-0.001) && *fIntX1<(fX2-0.001))||(*fIntX1>(fX1+0.001) && *fIntX1>(fX2+0.001))){
		//printf("	Intersection point 1 outside range X\n");
		//fflush(stdout);
		bPoint1=false;
	}
	if((*fIntX2<(fX1-0.001) && *fIntX2<(fX2-0.001))||(*fIntX2>(fX1+0.001) && *fIntX2>(fX2+0.001))){
		//printf("	Intersection point 2 outside range X\n");
		//fflush(stdout);
		bPoint2=false;	
	}
	if((*fIntY1<(fY1-0.001) && *fIntY1<(fY2-0.001))||(*fIntY1>(fY1+0.001) && *fIntY1>(fY2+0.001))){
	//	printf("	Intersection point 1 outside range Y\n");
	//	fflush(stdout);
		bPoint1=false;
	}
	if((*fIntY2<(fY1-0.001) && *fIntY2<(fY2-0.001))||(*fIntY2>(fY1+0.001) && *fIntY2>(fY2+0.001))){
		//printf("	Intersection point 2 outside range Y\n");
		//fflush(stdout);
		bPoint2=false;
	}
	
	return (bPoint1||bPoint2);
	/*
	if(bPoint1 || bPoint2){
		//printf("IN CIRCLE LINE INTERSECTIONS, INTERSECTIONS: (%f;%f) -- (%f;%f)\n",*fIntX1,*fIntY1,*fIntX2,*fIntY2);
		//fflush(stdout);
		return true;
	}
	
	//printf("	NO INTERSECTIONS (OUT OF RANGE): (%f;%f) -- (%f;%f)\n",*fIntX1,*fIntY1,*fIntX2,*fIntY2);	
	//fflush(stdout);
	return false;
	/**/
}

/******************************************************************************/
/******************************************************************************/


const double* CProximitySensorPolynomial::GetIRSensorDirections()
{
    return m_fIRSensorDir;
}

/******************************************************************************/
/******************************************************************************/
