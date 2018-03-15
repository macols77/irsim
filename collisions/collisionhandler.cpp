#include "collisionhandler.h"
//TO BE REMOVED
#include <fstream>
/******************************************************************************/
/******************************************************************************/
//Set of functions to handle collisions
/******************************************************************************/
/******************************************************************************/

dVector2 GetCompenetrationVector(CCollisionObject* pcEpuckCollObj,CCollisionObject* pcWallCollObj,float fEpuckHeading);
int GetAngleInt(float fAngle);
int CircleLineIntersections(float fX1,float fY1,float fX2,float fY2,float fXCirc,float fYCirc,float fRadius,float *fIntX1,float *fIntY1,float *fIntX2,float *fIntY2);
//TO BE REMOVED
void WriteToFile(float,float);

float g_fEpuckRadius=0;
float g_fPuckRadius=0;
bool verbose=false;
//this function ask the collision manager for collisions and, if needed (collision occurred), resets to the previous epuck position
void ResetToPreviousPosition(CCollisionManager* pc_collMgr,CCollisionEpuck* pc_epuck,dVector2 vPrev_pos){

	TCollisionObjectVector vCollisionVector = pc_collMgr->CalculateAndGetCollisionVector(pc_epuck->GetCollisionObject());

    if (vCollisionVector.size() > 0)
    {//reset position and return
        pc_epuck->SetPosition(vPrev_pos);
	return;
    }

    //vector<dVector2> vWallCollisionVector = pc_collMgr->CalculateAndGetCollisionsWithWalls(pc_epuck->GetCollisionObject());
	TCollisionObjectVector vWallCollisionVector = pc_collMgr->CalculateAndGetCollisionsWithWalls2(pc_epuck->GetCollisionObject());

    if (vWallCollisionVector.size() > 0 )
    {//reset position and return
//		printf("WALL COLLISION\n");
	//	fflush(stdout);
        pc_epuck->SetPosition(vPrev_pos);
	return;
    }
}

/******************************************************************************/
/******************************************************************************/

//TO BE REMOVED last parameter, used for debugging purpose
void BounceCollision(CCollisionManager* pc_collMgr,TEpuckVector* vEpuck,int stepNo){
	bool bGlobalCollision=true;

	int ciclenum=0; //Only in cicle 1 the collisions should be counted

	do{
		bGlobalCollision=false;
		ciclenum++;

//		printf("Cicle number: %d\n",ciclenum);
	//	fflush(stdout);

		//Diplacement vectors of the epucks
		vector<dVector2> displacements;
		TEpuckIterator iEpuckIterator=(*vEpuck).begin();

		//TO BE DELETED
		//verbose=true;
		//if((*iEpuckIterator)->GetBuggy()&&stepNo>=1637){
/*		if(stepNo>=240){
		 verbose=true;
		}
	*/
		if((*vEpuck).size()!=0)
		{
			if(g_fEpuckRadius==0){
				CCollisionObject *pcTmpCollObj=((CCollisionEpuck*)(*iEpuckIterator))->GetCollisionObject();
				TCollisionObjectVector* vChilds=((CCompoundCollisionObject*)pcTmpCollObj)->GetCollisionChildren();
				TCollisionObjectVector::iterator tmp_it=(*vChilds).begin();
				CCircleCollisionObject* pcEpuckCircleCollObj = (CCircleCollisionObject*)(*tmp_it);
				g_fEpuckRadius=pcEpuckCircleCollObj->GetRadius();
			}
		}
		else
		{
			g_fEpuckRadius=0;
		}
		while(iEpuckIterator!=(*vEpuck).end()){


			dVector2 vDisp=ZEROVECTOR2;
			TCollisionObjectVector vCollisionVector = pc_collMgr->CalculateAndGetCollisionVector(((CCollisionEpuck*)(*iEpuckIterator))->GetCollisionObject());

			if(vCollisionVector.size()>0){
			    //printf("Collision detected with another collision object\n");
				bGlobalCollision=true;
				if(ciclenum==1){
					(*iEpuckIterator)->AddCollision();
				}
				TCollisionObjectVector::iterator it=vCollisionVector.begin();
				while(it!=vCollisionVector.end()){
					//Computing the displacement vector
					dVector2 distance_centres;
					dVec2Sub(distance_centres,((CCollisionEpuck*)(*iEpuckIterator))->GetPosition(),(*it)->GetPosition());
					//Displacment measure
					float fDisplacementAmount=2*g_fEpuckRadius-dVec2Length(distance_centres);
					dVec2Normalize(distance_centres);

					dVec2MultiplyScalar(distance_centres,fabs(fDisplacementAmount));
					//updating the e-puck displacement vector
					dVec2Rotate(M_PI,distance_centres);
					dVec2Add(vDisp,vDisp,distance_centres);
					//next collision object
					it++;
				}//while cicle, cicling on the mobile objects current epuck collided with
			}

			//HANDLING COLLISIONS WITH WALLS
			//TO BE REMOVED
			if(verbose){
				printf("\n\nHANDLER: Begin walls management\n");
				printf("Curren epuck: %s\n",(*iEpuckIterator)->GetName());
				printf("Epuck position:  %f -- %f\n",((*iEpuckIterator)->GetPosition()).x,((*iEpuckIterator)->GetPosition()).y);
				printf("Epuck heading: %f\n",(*iEpuckIterator)->GetRotation()*180/M_PI);
				fflush(stdout);
			}
			TCollisionObjectVector vWallCollisionVector=pc_collMgr->CalculateAndGetCollisionsWithWalls2(((CCollisionEpuck*)(*iEpuckIterator))->GetCollisionObject());
			if(vWallCollisionVector.size()>0){
			    //printf("Collision detected with walls\n");
				if(verbose){
					printf("Collision vector .size > 0 \n");
					fflush(stdout);
				}
				bGlobalCollision=true;
				if(ciclenum==1){
					//printf("Adding a collision, \n");
					(*iEpuckIterator)->AddCollision();
				}
				TCollisionObjectVector::iterator wall_it=vWallCollisionVector.begin();

				dVector2 vResult=ZEROVECTOR2;
				while(wall_it!=vWallCollisionVector.end()){
				/*if((*iEpuckIterator)->GetBuggy()){
					printf("Calling get compenetration vector\n");
					fflush(stdout);
				}*/
					//TO BE REMOVED
					if(verbose){
						printf("HANDLER: Calling get compenetration vector\n");
						fflush(stdout);
					}
					dVector2 vComp=GetCompenetrationVector(((CCollisionEpuck*)(*iEpuckIterator))->GetCollisionObject(),(*wall_it),(*iEpuckIterator)->GetRotation());
					if(verbose){
						printf("Compenetration with current wall piece: %f --%f \n",vComp.x,vComp.y);
						fflush(stdout);
					}
					//bounce effect with the wall
					dVec2MultiplyScalar(vComp,2);
					dVec2Rotate(M_PI,vComp);
					dVec2Add(vDisp,vDisp,vComp);

					//This is used to give a direction to default displacement vector,
					//doesn't work if robot goes inside the wall more than half of the width
					dVector2 vWallPos=(*wall_it)->GetPosition();
					dVec2Add(vResult,vResult,vWallPos);

					if((*wall_it)->GetCollisionObjectType()==COLLISION_OBJECT_TYPE_COMPOUND){
    					((CCompoundCollisionObject*)(*wall_it))->DeleteChildren();
	    				delete *wall_it;
	    			}
					//Next piece of wall
					wall_it++;
				}
			    /*if((*iEpuckIterator)->GetBuggy()){
				printf("Exit while cycle\n");
				fflush(stdout);
				}*/
				if(fabs(vDisp.x)<=0.000001 && fabs(vDisp.y)<=0.000001){
						vDisp.x=0.0001;
						vDisp.y=0.0002;
						//TO BE REMOVED
					if(verbose){
							printf("Default displacement\n");
							fflush(stdout);
						}
						float fRotationAmount=dVec2OwnAngle(vResult)+M_PI;
						//dVec2Rotate((*iEpuckIterator)->GetRotation()+M_PI,vDisp);
						//dVec2Rotate((*iEpuckIterator)->GetMovementHeading()+M_PI,vDisp);
						dVec2Rotate(fRotationAmount,vDisp);
						if(verbose){
							printf("Movement heading: %f\n",(*iEpuckIterator)->GetMovementHeading()*180/M_PI);
							printf("Displ Vector (default, rotated bi PI): %f --%f\n",vDisp.x,vDisp.y);
							fflush(stdout);
						}
					}
			}

		//Storing the displacement for the current epuck
		displacements.push_back(vDisp);
		iEpuckIterator++;
		}//while cicle, cicling on all the epucks


		//Now updating the positions of the epucks, using saved displacements
		iEpuckIterator=(*vEpuck).begin();
		vector<dVector2>::iterator displ_iterator=displacements.begin();

		if(verbose) printf("\n HANDLER: applying displacements \n");
		while(iEpuckIterator!=(*vEpuck).end()){
			//TO BE REMOVED
			if(verbose)
				printf("Current epuck: %s\n",(*iEpuckIterator)->GetName());

			dVector2 vNewPos;
			dVec2Add(vNewPos,(*iEpuckIterator)->GetPosition(),(*displ_iterator));
			//TO BE DELETED
			if(verbose){
				printf("	Cicle number: %d, displacement vector: %f -- %f\n",ciclenum,(*displ_iterator).x,(*displ_iterator).y);
				printf("	NEW position:  %f -- %f\n",vNewPos.x,vNewPos.y);
				fflush(stdout);
			}
			((CCollisionEpuck*)(*iEpuckIterator))->SetPosition(vNewPos);
			((CCollisionEpuck*)(*iEpuckIterator))->UpdateCollisionPosition();
			fflush(stdout);
			iEpuckIterator++;
			displ_iterator++;
		}
	}while(bGlobalCollision); //While a collision is detected the robot must be repositioned

	//if(verbose) sleep(1);
}

/******************************************************************************/
/******************************************************************************/

//TO BE REMOVED last parameter, used for debugging purpose
void BounceCollision(CCollisionManager* pc_collMgr,TPuckVector* vPuck,int stepNo){
	bool bGlobalCollision=true;

	int ciclenum=0; //Only in cicle 1 the collisions should be counted

	if(vPuck->size()!=0)
	{
    	do{
    		bGlobalCollision=false;
    		ciclenum++;

    		//Diplacement vectors of the epucks
    		vector<dVector2> displacements;
    		TPuckIterator iPuckIterator=(*vPuck).begin();

    		if(g_fPuckRadius==0){
    			CCollisionObject *pcTmpCollObj=((CCollisionPuck*)(*iPuckIterator))->GetCollisionObject();
    			TCollisionObjectVector* vChilds=((CCompoundCollisionObject*)pcTmpCollObj)->GetCollisionChildren();
    			TCollisionObjectVector::iterator tmp_it=(*vChilds).begin();
    			CCircleCollisionObject* pcPuckCircleCollObj = (CCircleCollisionObject*)(*tmp_it);
    			g_fPuckRadius=pcPuckCircleCollObj->GetRadius();
    		}
    		while(iPuckIterator!=(*vPuck).end()){


    			dVector2 vDisp=ZEROVECTOR2;
    			TCollisionObjectVector vCollisionVector = pc_collMgr->CalculateAndGetCollisionVector(((CCollisionPuck*)(*iPuckIterator))->GetCollisionObject());

    			if(vCollisionVector.size()>0){
    				bGlobalCollision=true;
    				if(ciclenum==1){
    					(*iPuckIterator)->AddCollision();
    				}
    				TCollisionObjectVector::iterator it=vCollisionVector.begin();
    				while(it!=vCollisionVector.end()){
    					//Computing the displacement vector
    					dVector2 distance_centres;
    					dVec2Sub(distance_centres,((CCollisionPuck*)(*iPuckIterator))->GetPosition(),(*it)->GetPosition());
    					//Displacment measure
    					float fDisplacementAmount=2*g_fPuckRadius-dVec2Length(distance_centres);
    					dVec2Normalize(distance_centres);

    					dVec2MultiplyScalar(distance_centres,fabs(fDisplacementAmount));
    					//updating the e-puck displacement vector
    					dVec2Rotate(M_PI,distance_centres);
    					dVec2Add(vDisp,vDisp,distance_centres);
    					//next collision object
    					it++;
    				}//while cicle, cicling on the mobile objects current epuck collided with
    			}

    			//HANDLING COLLISIONS WITH WALLS
    			//TO BE REMOVED
    			if(verbose){
    				printf("\n\nHANDLER: Begin walls management\n");
    				printf("Current puck: %s\n",(*iPuckIterator)->GetName());
    				printf("Puck position:  %f -- %f\n",((*iPuckIterator)->GetPosition()).x,((*iPuckIterator)->GetPosition()).y);
    				printf("Puck heading: %f\n",(*iPuckIterator)->GetRotation()*180/M_PI);
    				fflush(stdout);
    			}
    			TCollisionObjectVector vWallCollisionVector=pc_collMgr->CalculateAndGetCollisionsWithWalls2(((CCollisionPuck*)(*iPuckIterator))->GetCollisionObject());
    			if(vWallCollisionVector.size()>0){
    				if(verbose){
    					printf("Collision vector .size > 0 \n");
    					fflush(stdout);
    				}
    				bGlobalCollision=true;
    				if(ciclenum==1){
    		//			printf("Adding a collision, \n");
    					(*iPuckIterator)->AddCollision();
    				}
    				TCollisionObjectVector::iterator wall_it=vWallCollisionVector.begin();

    				dVector2 vResult=ZEROVECTOR2;
    				while(wall_it!=vWallCollisionVector.end()){
    					//TO BE REMOVED
    					if(verbose){
    						printf("HANDLER: Calling get compenetration vector\n");
    					}
    					dVector2 vComp=GetCompenetrationVector(((CCollisionPuck*)(*iPuckIterator))->GetCollisionObject(),(*wall_it),(*iPuckIterator)->GetRotation());
    					if(verbose){
    						printf("Compenetration with current wall piece: %f --%f \n",vComp.x,vComp.y);
    					}
    					//bounce effect with the wall
    					dVec2MultiplyScalar(vComp,2);
    					dVec2Rotate(M_PI,vComp);
    					dVec2Add(vDisp,vDisp,vComp);

    					//This is used to give a direction to default displacement vector,
    					//doesn't work if puck goes inside the wall more than half of the width
    					dVector2 vWallPos=(*wall_it)->GetPosition();
    					dVec2Add(vResult,vResult,vWallPos);

    					if((*wall_it)->GetCollisionObjectType()==COLLISION_OBJECT_TYPE_COMPOUND){
        					((CCompoundCollisionObject*)(*wall_it))->DeleteChildren();
    	    				delete *wall_it;
    	    			}
    					//Next piece of wall
    					wall_it++;
    				}
    				if(fabs(vDisp.x)<=0.000001 && fabs(vDisp.y)<=0.000001){
    						vDisp.x=0.0001;
    						vDisp.y=0.0002;
    						//TO BE REMOVED
        					if(verbose){
    							printf("Default displacement\n");
    						}
    						float fRotationAmount=dVec2OwnAngle(vResult)+M_PI;
    						//dVec2Rotate((*iEpuckIterator)->GetRotation()+M_PI,vDisp);
    						//dVec2Rotate((*iEpuckIterator)->GetMovementHeading()+M_PI,vDisp);
    						dVec2Rotate(fRotationAmount,vDisp);
    						if(verbose){
    							//printf("Movement heading: %f\n",(*iPuckIterator)->GetMovementHeading()*180/M_PI);
    							printf("Displ Vector (default, rotated bi PI): %f --%f\n",vDisp.x,vDisp.y);
    						}
    					}
    			}

    		//Storing the displacement for the current puck
    		displacements.push_back(vDisp);
    		iPuckIterator++;
    		}//while cicle, cicling on all the pucks


    		//Now updating the positions of the epucks, using saved displacements
    		iPuckIterator=(*vPuck).begin();
    		vector<dVector2>::iterator displ_iterator=displacements.begin();

    		if(verbose) printf("\n HANDLER: applying displacements \n");
    		while(iPuckIterator!=(*vPuck).end()){
    			//TO BE REMOVED
    			if(verbose)
    				printf("Current puck: %s\n",(*iPuckIterator)->GetName());

    			dVector2 vNewPos;
    			dVec2Add(vNewPos,(*iPuckIterator)->GetPosition(),(*displ_iterator));
    			//TO BE DELETED
    			if(verbose){
    				printf("	Cicle number: %d, displacement vector: %f -- %f\n",ciclenum,(*displ_iterator).x,(*displ_iterator).y);
    				printf("	NEW position:  %f -- %f\n",vNewPos.x,vNewPos.y);
    			}
    			((CCollisionPuck*)(*iPuckIterator))->SetPosition(vNewPos);
    			((CCollisionPuck*)(*iPuckIterator))->UpdateCollisionPosition();
    			iPuckIterator++;
    			displ_iterator++;
    		}
    	}while(bGlobalCollision); //While a collision is detected the robot must be repositioned
	}
	//if(verbose) sleep(1);
}

/******************************************************************************/
/******************************************************************************/

dVector2 GetCompenetrationVector(CCollisionObject* pcEpuckCollObj,CCollisionObject* pcWallCollObj, float fEpuckHeading){

	//NOTE: ASSUMING O1 AS A CIRCULAR COLLISION OBJECT, BECAUSE IT'S A COLLISION EPUCK SIMPLE AND THUS ROUND
	dVector2 VComp=ZEROVECTOR2;
	TCollisionObjectVector* vChilds=((CCompoundCollisionObject*)pcEpuckCollObj)->GetCollisionChildren();
	TCollisionObjectVector::iterator tmp_it=(*vChilds).begin();
	CCircleCollisionObject* pcEpuckCircleCollObj = (CCircleCollisionObject*)(*tmp_it);


	//CHECKING FOR THE WALL OBJECT TYPE
	if(pcWallCollObj->GetCollisionObjectType()==COLLISION_OBJECT_TYPE_COMPOUND){
		TCollisionObjectVector* vChilds=((CCompoundCollisionObject*)pcWallCollObj)->GetCollisionChildren();
		TCollisionObjectVector::iterator it=(*vChilds).begin();
		while(it!=(*vChilds).end()){
				//Recursively call the function
				dVector2 vTmp=GetCompenetrationVector(pcEpuckCollObj,(*it),fEpuckHeading);
				//This should work with the compound object being a single object (which is the case for the wall object)
				dVec2Add(VComp,VComp,vTmp);
				it++;
			}
		return VComp;

	}else if(pcWallCollObj->GetCollisionObjectType()==COLLISION_OBJECT_TYPE_RECTANGLE){
		fflush(stdout);
		//RECTANGULAR OBJECTS MANAGEMENT
		//Getting the rectangle's corners
		dVector2 vRectPosition=pcWallCollObj->GetPosition();
		float fX1=vRectPosition.x-((CRectangleCollisionObject*)pcWallCollObj)->GetHalfSizeX();
		float fX2=vRectPosition.x+((CRectangleCollisionObject*)pcWallCollObj)->GetHalfSizeX();
		float fY1=vRectPosition.y+((CRectangleCollisionObject*)pcWallCollObj)->GetHalfSizeY();
		float fY2=vRectPosition.y-((CRectangleCollisionObject*)pcWallCollObj)->GetHalfSizeY();
		if(verbose){
			printf("Current rectangle: %f/%f  -- %f/%f\n",fX1,fY1,fX2,fY2);
			fflush(stdout);
		}

		float fIntX1_1,fIntX2_1,fIntY1_1,fIntY2_1;
		float fIntX1_2,fIntX2_2,fIntY1_2,fIntY2_2;
		float fIntX1_3,fIntX2_3,fIntY1_3,fIntY2_3;
		float fIntX1_4,fIntX2_4,fIntY1_4,fIntY2_4;


		//Checking intersections (thus collisions) between circle and rectangle's sides
		if(CircleLineIntersections(fX1,fY2,fX1,fY1,(pcEpuckCircleCollObj->GetPosition()).x,(pcEpuckCircleCollObj->GetPosition()).y,pcEpuckCircleCollObj->GetRadius(),&fIntX1_1,&fIntY1_1,&fIntX2_1,&fIntY2_1)){
		//		printf("Side 1 intersections: %f--%f  %f--%f\n",fIntX1_1,fIntY1_1,fIntX2_1,fIntY2_1);
		//		fflush(stdout);
				if(((fIntY1_1>fY2) && (fIntY1_1<fY1)) || ((fIntY2_1>fY2) && (fIntY2_1<fY1))){
					float fTmpY=(fIntY1_1+fIntY2_1)/2;
					dVector2 vTmp={fX1,fTmpY};
					float fCompenetration=pcEpuckCircleCollObj->GetRadius()-dVec2Distance(vTmp,pcEpuckCircleCollObj->GetPosition());
					dVector2 vComTmp={fCompenetration,0};
					dVec2Add(VComp,VComp,vComTmp);
				}
		}
		if(CircleLineIntersections(fX1,fY2,fX2,fY2,(pcEpuckCircleCollObj->GetPosition()).x,(pcEpuckCircleCollObj->GetPosition()).y,pcEpuckCircleCollObj->GetRadius(),&fIntX1_2,&fIntY1_2,&fIntX2_2,&fIntY2_2)){
				//printf("Side 2 intersections: %f--%f  %f--%f\n",fIntX1_2,fIntY1_2,fIntX2_2,fIntY2_2);
				//fflush(stdout);
			if(((fIntX1_2>fX1) && (fIntX1_2<fX2)) || ((fIntX2_2>fX1) && (fIntX2_2<fX2))){
				float fTmpX=(fIntX1_2+fIntX2_2)/2;
				dVector2 vTmp={fTmpX,fY2};
				float fCompenetration=pcEpuckCircleCollObj->GetRadius()-dVec2Distance(vTmp,pcEpuckCircleCollObj->GetPosition());
				dVector2 vComTmp={0,fCompenetration};
				dVec2Add(VComp,VComp,vComTmp);
				}
		}
		if(CircleLineIntersections(fX2,fY2,fX2,fY1,(pcEpuckCircleCollObj->GetPosition()).x,(pcEpuckCircleCollObj->GetPosition()).y,pcEpuckCircleCollObj->GetRadius(),&fIntX1_3,&fIntY1_3,&fIntX2_3,&fIntY2_3)){
				//printf("Side 3 intersections: %f--%f  %f--%f\n",fIntX1_3,fIntY1_3,fIntX2_3,fIntY2_3);
				//fflush(stdout);
			if(((fIntY1_3>fY2) && (fIntY1_3<fY1)) || ((fIntY2_3>fY2) && (fIntY2_3<fY1))){
				float fTmpY=(fIntY1_3+fIntY2_3)/2;
				dVector2 vTmp={fX2,fTmpY};
				float fCompenetration=pcEpuckCircleCollObj->GetRadius()-dVec2Distance(vTmp,pcEpuckCircleCollObj->GetPosition());
				dVector2 vComTmp={-fCompenetration,0};
				dVec2Add(VComp,VComp,vComTmp);
				}
		}
		if(CircleLineIntersections(fX1,fY1,fX2,fY1,(pcEpuckCircleCollObj->GetPosition()).x,(pcEpuckCircleCollObj->GetPosition()).y,pcEpuckCircleCollObj->GetRadius(),&fIntX1_4,&fIntY1_4,&fIntX2_4,&fIntY2_4)){
				//printf("Side 4 intersections: %f--%f  %f--%f\n",fIntX1_4,fIntY1_4,fIntX2_4,fIntY2_4);
				//fflush(stdout);
			if(((fIntX1_4>fX1) && (fIntX1_4<fX2)) || ((fIntX2_4>fX1) && (fIntX2_4<fX2))){
				float fTmpX=(fIntX1_4+fIntX2_4)/2;
				dVector2 vTmp={fTmpX,fY1};
				float fCompenetration=pcEpuckCircleCollObj->GetRadius()-dVec2Distance(vTmp,pcEpuckCircleCollObj->GetPosition());
				dVector2 vComTmp={0,-fCompenetration};
				dVec2Add(VComp,VComp,vComTmp);
				}
		}
		return VComp;

	}else if(pcWallCollObj->GetCollisionObjectType()==COLLISION_OBJECT_TYPE_RING){

		float fRingRadius=((CRingCollisionObject*)pcWallCollObj)->GetRadius();
		float fEpuckRadius=pcEpuckCircleCollObj->GetRadius();

		float fCompenetration=dVec2Length(pcEpuckCircleCollObj->GetPosition())+fEpuckRadius-fRingRadius;

		//HERE ASSUMING EPUCK AND RING OBJECTS HAVING THE SAME ORIGIN
		if(!fCompenetration==0){
			dVector2 tmp=pcEpuckCircleCollObj->GetPosition();
			dVec2Normalize(tmp);
			//maybe useless
			while(fabs(fCompenetration)<0.001){
				fCompenetration *=10;
			}
			dVec2MultiplyScalar(tmp,fCompenetration);
			return tmp;
		}
		return ZEROVECTOR2;
	}
}


/******************************************************************************/
/******************************************************************************/

// TAKEN FROM: http://astronomy.swin.edu.au/~pbourke/geometry/ see there for details

//Compute the intersection points between the segment (fX1,fY1)(fX2,fY2) and the circle
//(fXCirc,fYCirc),fRadius

int CircleLineIntersections(float fX1,float fY1,float fX2,float fY2,float fXCirc,float fYCirc,float fRadius,float *fIntX1,float *fIntY1,float *fIntX2,float *fIntY2)
{

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

   if (fabs(a) < 0.00001 || bb4ac < 0) {
      dmu1 = 0;
      dmu2 = 0;
      return(false);
   }

   dmu1 = (-b + sqrt(bb4ac)) / (2 * a);
   dmu2 = (-b - sqrt(bb4ac)) / (2 * a);


   *fIntX1=fX1 + dmu1*(fX2-fX1);
   *fIntX2=fX1 + dmu2*(fX2-fX1);

   *fIntY1=fY1 + dmu1*(fY2-fY1);
   *fIntY2=fY1 + dmu2*(fY2-fY1);


   //Here I check if the intersection point is extern with respoect to the circle,
   //works only with vertical and horizontal segments (wich is the case of arena's obstacles)
   float fMaxX=fXCirc+fRadius;
   float fMinX=fXCirc-fRadius;
   float fMaxY=fYCirc+fRadius;
   float fMinY=fYCirc-fRadius;
   /*
   if(((*fIntX1<fMinX) || (*fIntX1>fMaxX)) && ((*fIntX2<fMinX) || (*fIntX2>fMaxX)))
   {
	   /*printf("\n\n Min coord: %f--%f    max coord: %f--%f\n",fMinX,fMinY,fMaxX,fMaxY);
	   printf("Intersection points: %f--%f   and %f--%f\n",*fIntX1,*fIntY1,*fIntX2,*fIntY2);
	   printf("Intersection not possible, X out of range\n");
	   fflush(stdout);
	   //return false;
	   }
   if(((*fIntY1<fMinY) || (*fIntY1>fMaxY)) && ((*fIntY2<fMinY) || (*fIntY2>fMaxY))){
	   /*printf("\n\n Min coord: %f--%f    max coord: %f--%f\n",fMinX,fMinY,fMaxX,fMaxY);
	   printf("Intersection points: %f--%f   and %f--%f\n",*fIntX1,*fIntY1,*fIntX2,*fIntY2);
	   printf("Intersection not possible, Y out of reange\n");
	   fflush(stdout);/*
      return false;
   }
   */
   return true;
}

/******************************************************************************/
/******************************************************************************/
//TO BE REMOVED
void WriteToFile(float x,float y)
{
	ofstream outfile ("/home/juan/documenti/workdir/simulatore/optimization/work_directory/path.txt", ios::app);
	outfile<<x<<" "<<y<<endl;
	outfile.close();

}

/******************************************************************************/
/******************************************************************************/
//BACKUP (boncing also with walls)
/*
void BounceCollision(CCollisionManager* pc_collMgr,TEpuckVector* vEpuck){
	bool bGlobalCollision=true;

	//TO BE DELETED
	int ciclenum=0;

	do{
		bGlobalCollision=false;

		//TO BE DELETED
		ciclenum++;

		//Diplacement vectors of the epucks
		vector<dVector2> displacements;
		TEpuckIterator iEpuckIterator=(*vEpuck).begin();

		if(g_fEpuckRadius==0){
			CCollisionObject *pcTmpCollObj=((CCollisionEpuck*)(*iEpuckIterator))->GetCollisionObject();
			TCollisionObjectVector* vChilds=((CCompoundCollisionObject*)pcTmpCollObj)->GetCollisionChildren();
			TCollisionObjectVector::iterator tmp_it=(*vChilds).begin();
			CCircleCollisionObject* pcEpuckCircleCollObj = (CCircleCollisionObject*)(*tmp_it);
			g_fEpuckRadius=pcEpuckCircleCollObj->GetRadius();
		}
		while(iEpuckIterator!=(*vEpuck).end()){

			dVector2 vDisp=ZEROVECTOR2;
			TCollisionObjectVector vCollisionVector = pc_collMgr->CalculateAndGetCollisionVector(((CCollisionEpuck*)(*iEpuckIterator))->GetCollisionObject());

			if(vCollisionVector.size()>0){
				bGlobalCollision=true;
				TCollisionObjectVector::iterator it=vCollisionVector.begin();
				while(it!=vCollisionVector.end()){
					//Computing the displacement vector
					dVector2 distance_centres;
					dVec2Sub(distance_centres,((CCollisionEpuck*)(*iEpuckIterator))->GetPosition(),(*it)->GetPosition());
					//Displacment measure
					float fDisplacementAmount=2*g_fEpuckRadius-dVec2Length(distance_centres);
					dVec2Normalize(distance_centres);

					dVec2MultiplyScalar(distance_centres,fabs(fDisplacementAmount));
					//updating the e-puck displacement vector
					dVec2Rotate(M_PI,distance_centres);
					dVec2Add(vDisp,vDisp,distance_centres);
					//next collision object
					it++;
				}//while cicle, cicling on the mobile objects current epuck collided with
			}

			//HANDLING COLLISIONS WITH WALLS
			TCollisionObjectVector vWallCollisionVector=pc_collMgr->CalculateAndGetCollisionsWithWalls2(((CCollisionEpuck*)(*iEpuckIterator))->GetCollisionObject());

			if(vWallCollisionVector.size()>0){
				bGlobalCollision=true;
				TCollisionObjectVector::iterator wall_it=vWallCollisionVector.begin();

				while(wall_it!=vWallCollisionVector.end()){
					dVector2 vComp=GetCompenetrationVector(((CCollisionEpuck*)(*iEpuckIterator))->GetCollisionObject(),(*wall_it),(*iEpuckIterator)->GetRotation());
					//bounce effect with the wall
					dVec2MultiplyScalar(vComp,2);
					dVec2Rotate(M_PI,vComp);
					dVec2Add(vDisp,vDisp,vComp);
					if((*wall_it)->GetCollisionObjectType()==COLLISION_OBJECT_TYPE_COMPOUND){
    					((CCompoundCollisionObject*)(*wall_it))->DeleteChildren();
	    				delete *wall_it;
	    			}
					//Next piece of wall
					wall_it++;
				}

				if(fabs(vDisp.x)<=0.000001 && fabs(vDisp.y)<=0.000001){//No displacement causes infinite loop
						vDisp.x=0.0001;
						vDisp.y=0.0002;
						//TO BE REMOVED
						if((*iEpuckIterator)->GetBuggy()){
							printf("Displ heading: %f\n",((*iEpuckIterator)->GetRotation()+M_PI)*180/M_PI);
							fflush(stdout);
							sleep(1);
						}
						dVec2Rotate((*iEpuckIterator)->GetRotation()+M_PI,vDisp);
					}
			}

		//Storing the displacement for the current epuck
		displacements.push_back(vDisp);
		iEpuckIterator++;
		}//while cicle, cicling on all the epucks


		//Now updating the positions of the epucks, using saved displacements
		iEpuckIterator=(*vEpuck).begin();
		vector<dVector2>::iterator displ_iterator=displacements.begin();

		while(iEpuckIterator!=(*vEpuck).end()){
			dVector2 vNewPos;
			dVec2Add(vNewPos,(*iEpuckIterator)->GetPosition(),(*displ_iterator));
			if((*iEpuckIterator)->GetBuggy()){
				//printf("OLD position:  %f -- %f\n",((*iEpuckIterator)->GetPosition()).x,((*iEpuckIterator)->GetPosition()).y);
				//printf("NEW position:  %f -- %f\n",vNewPos.x,vNewPos.y);
				//if(vNewPos.x>-3.5)
				//	WriteToFile(vNewPos.x,vNewPos.y);
			}
			((CCollisionEpuck*)(*iEpuckIterator))->SetPosition(vNewPos);
			((CCollisionEpuck*)(*iEpuckIterator))->UpdateCollisionPosition();
			//TO BE DELETED
			if((*iEpuckIterator)->GetBuggy()){
				printf("Cicle number: %d, displacement vector: %f -- %f\n",ciclenum,(*displ_iterator).x,(*displ_iterator).y);
				fflush(stdout);
				if(ciclenum>=2){
					sleep(1);
				}
			}
			iEpuckIterator++;
			displ_iterator++;
		}
	}while(bGlobalCollision); //While a collision is detected the robot must be repositioned
}

/******************************************************************************/
/******************************************************************************/
