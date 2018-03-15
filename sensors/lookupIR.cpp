#include "lookupIR.h"

/******************************************************************************/
/******************************************************************************/

double   CLookupProximity::m_fIRSensorDir[NUM_SENSORS] 	= {-0.2967, -0.8727, -1.5708, -2.6180, -3.6652, -4.7124, -5.4105, -5.9865};
//double   CLookupProximity::m_fIRSensorDir[NUM_SENSORS] 	= {0.00000, 0.584685, 1.221730, 2.094395, 3.141593, 4.188790, 5.061455, 5.698500};

/******************************************************************************/
/******************************************************************************/

CLookupProximity::CLookupProximity(const char* pch_name) : 
    CSensor(pch_name, NUM_SENSORS )
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
	
	for(int t=0;t<8;t++){
		m_fMaxReadings[t]=0.0;
	}
}

/******************************************************************************/
/******************************************************************************/

CLookupProximity::~CLookupProximity()
{    
}

/******************************************************************************/
/******************************************************************************/

void CLookupProximity::SetupSensor(const char* pch_fileName1,const char* pch_fileName2){
	ifstream in1,in2;
	in1.open(pch_fileName1,ios::in);
	in2.open(pch_fileName2,ios::in);
	
	printf("Using data from file: %s\n",pch_fileName1);
	
	if( !in1 ) {
        printf("ERROR: 'Lookup sensor setup' could not open file '%s'.\n",  pch_fileName1 );
        return;
    }

	if( !in2 ) {
        printf("ERROR: 'Lookup sensor setup' could not open file '%s'.\n",  pch_fileName2 );
        return;
    }
	

	float tmp;
	for(int i=0;i<NUMDISTANCES;i++){
		for(int j=0;j<NUMANGLES;j++){
			in1 >> tmp;
			in1 >> tmp; //discard first two numbers in the line (distance and angle)
			in2 >> tmp;
			in2 >> tmp;
			for(int k=0;k<NUM_SENSORS;k++){
				in1>>m_fTable1[i][j][k];
				if(m_fTable1[i][j][k] <0.0) m_fTable1[i][j][k]=0.0;
				in2>>m_fTable2[i][j][k];
				if(m_fTable2[i][j][k] <0.0) m_fTable2[i][j][k]=0.0;
				if(m_fTable2[i][j][k]>m_fMaxReadings[k]){
					m_fMaxReadings[k]=m_fTable2[i][j][k];
				}
				if(m_fTable1[i][j][k]>m_fMaxReadings[k]){
					m_fMaxReadings[k]=m_fTable1[i][j][k];
				}
			}
			//getchar();
		}//cycle over different headings
	}//cycle over all the distances
	/*
	for(int r=0;r<NUM_SENSORS;r++){
		printf("Max delta: %d, value: %f\n",r,m_fMaxReadings[r]);
	}
	sleep(2);/**/
}

/******************************************************************************/
/******************************************************************************/

// this function computes the content of m_unIRReadings,
// an array of NUM_SENSORS (8) unsigned int values
double* CLookupProximity::ComputeSensorReadings(CEpuck* pc_epuck, 
                                                CSimulator* pc_simulator)
{
	//printf("Compute sensor readings called **************************************\n");
	
    for( int j=0 ; j<NUM_SENSORS ; j++ )
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
            fDistance -= CEpuck::CHASSIS_RADIUS*2; //Used to compute the first index in the table
            if( fDistance < LOOKUP_MAXRANGE )
            {
                double fAngleToTarget=NormalizeAngle(atan2(yTargetEpuck-yCurrentEpuck,xTargetEpuck-xCurrentEpuck));
				//double fAngleToTarget=NormalizeAngle(atan2(yCurrentEpuck-yTargetEpuck,xCurrentEpuck-xTargetEpuck));
				//printf("Epuck angle to target: %f\n",fAngleToTarget*180/M_PI);
                double fDirection=NormalizeAngle(fAngleToTarget-pc_epuck->GetRotation()); //Used to compute second index of the table
				double fSensorDir=0.0; //Used to take care of target relative heading with respect to the sensor and thus choose the table
				double fDeltaDirection=0.0;
				int y,j;
				y=(int)(fDistance/DISTANCE_STEP);
				if(fDistance/DISTANCE_STEP-y>0.5){
					y++;
				}
				//printf("First (position) index: %d\n",y);
				//printf("Direction value: %f\n",fDirection*180/M_PI);
				j=((int)fDirection*180/M_PI)/ANGLE_STEP; //Second index
				if((fDirection*180/M_PI-j*ANGLE_STEP)>5){
					j++;
					if(j>35){
						j=0;
					}
				}
				
				//printf("Second (Angle) index: %d\n",j);
				
				for(int i=0;i<NUM_SENSORS;i++){
					//printf("Sensor: %d     -----------------------------------------\n",i);
					fSensorDir=NormalizeAngle(pc_epuck->GetRotation()+m_fIRSensorDir[i]);
					fDeltaDirection=NormalizeAngle(fSensorDir-NormalizeAngle((*pvEpucks)[nEpucks]->GetRotation()));
					//printf("	Sensor direction: %f\n",fSensorDir*180/M_PI);
					//printf("	Delta direction: %f\n",fDeltaDirection*180/M_PI);
					if(fDeltaDirection<M_PI/4 || fDeltaDirection>(7*M_PI/4)){//Epuck hit in front-back, table1
						if(m_fIRReadings[i]<m_fTable1[y][j][i]){
							m_fIRReadings[i]=m_fTable1[y][j][i];
						}
						//printf("Case 1, epuck in front-back, +-45, value: %f\n",m_fIRReadings[i]);
					}else if(fDeltaDirection > 3*M_PI/4 && fDeltaDirection< 5*M_PI/4){//Epuck hit in front-back, table1
						if(m_fIRReadings[i]<m_fTable1[y][j][i]){
							m_fIRReadings[i]=m_fTable1[y][j][i];
						}
						//printf("Case 1, epuck in front-back, 180+-45, value: %f\n",m_fIRReadings[i]);
					}else{//Epuck hit on the side, table2
						if(m_fIRReadings[i]<m_fTable2[y][j][i]){
							m_fIRReadings[i]=m_fTable2[y][j][i];
						}
					//	printf("Case 2, epuck in side, value: %f\n",m_fIRReadings[i]);
					}
				}
            }
        }
    }

	for(int i=0;i<NUM_SENSORS;i++){
		
		//NOISE: 20% background noise, 10% noise
		//		peercentage referred to maximum sensor delta
		//		changed the third evolution in 30% and 20%
		float fNoiseValue=0.0;
		if(m_fIRReadings[i]<0.00001){
			fNoiseValue=Random::nextDouble(0,0.3)*m_fMaxReadings[i];
			//printf("Sensor: %d, adding background noise: %f\n",i,fNoiseValue);
		}else{
			fNoiseValue=Random::nextDouble(-0.2,0.2)*m_fMaxReadings[i];
			//printf("Sensor: %d, adding normal noise: %f\n",i,fNoiseValue);
		}
		
		m_fIRReadings[i]+=fNoiseValue;
		
		if(m_fIRReadings[i]<0.00001){
			m_fIRReadings[i]=0.0;
		}
		SetInput(i,m_fIRReadings[i]);
	} 
	NormalizeReadings();	
	//sleep(1);
    return GetInputs();
}

/******************************************************************************/
/******************************************************************************/

unsigned int CLookupProximity::GetType()
{
    return SENSOR_PROXIMITY_LOOKUP;
}

/******************************************************************************/
/******************************************************************************/

const double* CLookupProximity::GetIRReadings()
{
    return m_fIRReadings;
}

/******************************************************************************/
/******************************************************************************/


void CLookupProximity::NormalizeReadings()
{
    for( int i=0 ; i<m_unNumberOfInputs ; i++ )
    {
		double fNormalizedInput =m_pfInputs[i]/m_fMaxReadings[i];
		if(fNormalizedInput<0) fNormalizedInput=0.0;
		//printf("Sensor: %d, value (normalized): %f\n",i,fNormalizedInput);
		//fflush(stdout);	
        SetInput(i,fmin(1.0,fNormalizedInput));
    }
}

/******************************************************************************/
/******************************************************************************/


const double* CLookupProximity::GetIRSensorDirections()
{
    return m_fIRSensorDir;
}

/******************************************************************************/
/******************************************************************************/
