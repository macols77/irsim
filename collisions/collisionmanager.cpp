#include "collisionmanager.h"
#include "compoundcollisionobject.h"
#include "rectanglecollisionobject.h"
#include "circlecollisionobject.h"
#include "ringcollisionobject.h"
#include "arena.h"

CCollisionManager* CCollisionManager::m_pcInstance = NULL;

/******************************************************************************/
/******************************************************************************/

// Class wich checks collisions between objects 

/******************************************************************************/
/******************************************************************************/

CCollisionManager::CCollisionManager(char* pch_name, CArena* pc_arena) : 
    CSimObject(pch_name), m_pcArena(pc_arena), m_unNumberOfCollisionsDetected(0)
{
    m_pcInstance = this;
    CGeometry::SetCollisionManager(this);
}

/******************************************************************************/
/******************************************************************************/

CCollisionManager::~CCollisionManager()
{
    m_pcInstance = NULL;
    CGeometry::SetCollisionManager(NULL);
}

/******************************************************************************/
/******************************************************************************/

void CCollisionManager::AddCollisionObject(CCollisionObject* pc_collision_object)
{
	m_vecCollisionObjects.push_back(pc_collision_object);
	//printf("Adding collision object: %s\n",pc_collision_object->GetName());
	//printf("CURRENT COLLISION VECTOR SIZE: %d\n",m_vecCollisionObjects.size());
}

/******************************************************************************/
/******************************************************************************/

void CCollisionManager::RemoveCollisionObject(CCollisionObject* pc_collision_object)
{
    TCollisionObjectIterator i = m_vecCollisionObjects.begin(); 
    //printf("	CM, called remove collision object, obj: %s\n",pc_collision_object->GetName());
    while (i != m_vecCollisionObjects.end())
    {
        if ((*i) == pc_collision_object)
        {
            m_vecCollisionObjects.erase(i);
            break;
        }
        i++; 
    }
	//printf("COLLISION OBJS VECTOR AFTER REMOVAL: %d\n",m_vecCollisionObjects.size());
}

/******************************************************************************/
/******************************************************************************/

bool CCollisionManager::CheckCollision(CCollisionObject* pc_collision_object, bool b_count_collisions)
{

    bool bCollisionFound = false;
    TCollisionObjectIterator i = m_vecCollisionObjects.begin(); 

    while (i != m_vecCollisionObjects.end() &&
        !bCollisionFound)
    {
        if ((*i)->IsEnabled() && (*i) != pc_collision_object)
            bCollisionFound = CheckCollisionBetweenTwoObjects(pc_collision_object, *i);
        
        i++; 
    }    

    if (bCollisionFound && b_count_collisions){
//			printf("COLLISION MANAGER: COLLISION DETECTED\n");
//	fflush(stdout);
        m_unNumberOfCollisionsDetected++;
	}
    return bCollisionFound;
}

/******************************************************************************/
/******************************************************************************/

TCollisionObjectVector CCollisionManager::CalculateAndGetCollisionVector(CCollisionObject* pc_collision_object, bool b_count_collisions)
{	
    unsigned int unNumberOfCollisions = 0;
    TCollisionObjectIterator i = m_vecCollisionObjects.begin(); 
    TCollisionObjectVector vCollisionVector;

    while (i != m_vecCollisionObjects.end())
    {


        if ((*i)->IsEnabled() && (*i) != pc_collision_object &&
            CheckCollisionBetweenTwoObjects(pc_collision_object, *i))
        {
            vCollisionVector.push_back(*i);
            unNumberOfCollisions++;            
        }
        i++; 
    }    

    return vCollisionVector;
}

/******************************************************************************/
/******************************************************************************/

vector<dVector2> CCollisionManager::CalculateAndGetCollisionsWithWalls(CCollisionObject* pc_collision_object)
{
  vector<dVector2> vWallPositions;

  if(pc_collision_object->GetCollisionObjectType()==COLLISION_OBJECT_TYPE_CIRCLE){
	  double fSizeX, fSizeY;
	  unsigned int unNumCellsX, unNumCellsY;
	
	  m_pcArena->GetSize(&fSizeX,&fSizeY);
	  m_pcArena->GetResolution(&unNumCellsX,&unNumCellsY);
	
	  double fCellSizeX = fSizeX/(double)unNumCellsX;
	  double fCellSizeY = fSizeY/(double)unNumCellsY;
	  if( fabs(fCellSizeX - fCellSizeY)>1.23456e-7 )
	  {
		  static bool bErrorMessagePrinted = false;
		  if (!bErrorMessagePrinted)
		  {
	
			printf("ERROR: in CCollisionManager::CalculateAndGetCollisionsWithWalls wrong cell size: fSizeX/unNumCellsX:%f , fSizeY/unNumCellsY:%f , diff:%f\n",fCellSizeX,fCellSizeY,fCellSizeX-fCellSizeY);
			bErrorMessagePrinted = true;
		  }
		  return vWallPositions;      
	  }
	
	  double fCellSize = fCellSizeX;
	  
	  bool bWallEncountered = false;
	  dVector2 vOwnPosition = pc_collision_object->GetPosition();
	  double fX = vOwnPosition.x;
	  double fY = vOwnPosition.y;
	
	  double fDistanceToLeftCellBorder 	= ((fX+fSizeX/2)/fCellSize)*fCellSize-(unsigned int)((fX+fSizeX/2)/fCellSize)*fCellSize;
	  double fDistanceToRightCellBorder 	= fCellSize - fDistanceToLeftCellBorder;
	  double fDistanceToBottomCellBorder 	= ((fY+fSizeY/2)/fCellSize)*fCellSize-(unsigned int)((fY+fSizeY/2)/fCellSize)*fCellSize;
	  double fDistanceToTopCellBorder   	= fCellSize - fDistanceToBottomCellBorder;
	
	  double fRadius = ((CCircleCollisionObject*)pc_collision_object)->GetRadius();
	
	  while( !bWallEncountered && fX > vOwnPosition.x-fRadius )
		{
			  fX -= fCellSize;
			  if( m_pcArena->GetHeight(fX,fY) == HEIGHT_OBSTACLE )
			{
			  bWallEncountered = true;
			  double fDistance = vOwnPosition.x - fX - fDistanceToRightCellBorder;
			  if( fDistance < fRadius )
				{
				  dVector2 vWallPosition;
				  vWallPosition.x = fX+fCellSize-fRadius;
				  vWallPosition.y = fY;
				  vWallPositions.push_back(vWallPosition);
				}
			}
		}
	
	  // check to the right
	  fX = vOwnPosition.x;
	  bWallEncountered = false;
	  while( !bWallEncountered &&  fX < vOwnPosition.x+fRadius )
		{
			  fX += fCellSize;
			  if( m_pcArena->GetHeight(fX,fY) == HEIGHT_OBSTACLE )
			{
			  bWallEncountered = true;
			  double fDistance = fX - vOwnPosition.x - fDistanceToLeftCellBorder;
			  if( fDistance < fRadius )
				{
				  dVector2 vWallPosition;
				  vWallPosition.x = fX-fCellSize+fRadius;
				  vWallPosition.y = fY;
				  vWallPositions.push_back(vWallPosition);
				}
			}
		}
	
	  // check to the top
	  fX = vOwnPosition.x;
	  bWallEncountered = false;
	  while( !bWallEncountered && fY < vOwnPosition.y+fRadius )
		{
			  fY += fCellSize;
			  if( m_pcArena->GetHeight(fX,fY) == HEIGHT_OBSTACLE )
			{
			  bWallEncountered = true;
			  double fDistance = fY - vOwnPosition.y - fDistanceToBottomCellBorder;
			  if( fDistance < fRadius )
				{
				  dVector2 vWallPosition;
				  vWallPosition.x = fX;
				  vWallPosition.y = fY-fCellSize+fRadius;
				  vWallPositions.push_back(vWallPosition);
				}
			}
		}
	
	  // check to the bottom
	  fY = vOwnPosition.y;
	  bWallEncountered = false;
	  while( !bWallEncountered && fY > vOwnPosition.y-fRadius )
		{
			  fY -= fCellSize;
			  if( m_pcArena->GetHeight(fX,fY) == HEIGHT_OBSTACLE )
			{
			  bWallEncountered = true;
			  double fDistance = vOwnPosition.y - fY - fDistanceToTopCellBorder;
			  if( fDistance < fRadius )
				{
				  dVector2 vWallPosition;
				  vWallPosition.x = fX;
				  vWallPosition.y = fY+fCellSize-fRadius;
				  vWallPositions.push_back(vWallPosition);
				}
			}
		}

  }
  return vWallPositions;
}

/******************************************************************************/
/******************************************************************************/

void CCollisionManager::Reset(){
	/*
	printf("CM resetted, number of collision object: %d\n",m_vecCollisionObjects.size());
	for(int i=0;i<m_vecCollisionObjects.size();i++){
		printf("	Collision object: %s\n",m_vecCollisionObjects[i]->GetName());
	}
	sleep(1);
	/**/
	m_unNumberOfCollisionsDetected=0;	
}

/******************************************************************************/
/******************************************************************************/
/* THIS WAS THE ORIGINAL CODE (IN WHICH I SIMPLY ADDED THE ROUND ARENA MANAGEMENT)

vector<dVector2> CCollisionManager::CalculateAndGetCollisionsWithWalls(CCollisionObject* pc_collision_object)
{
  vector<dVector2> vWallPositions;
  if( m_pcArena->GetArenaType()==ARENA_TYPE_ROUND){
		if(CheckCollisionBetweenTwoObjects(pc_collision_object, ((CRoundArena*)m_pcArena)->GetCollisionObject())){
				dVector2 vTmp=pc_collision_object->GetPosition();
				vWallPositions.push_back(vTmp);
			}
}else {

  double fSizeX, fSizeY;
  unsigned int unNumCellsX, unNumCellsY;

  m_pcArena->GetSize(&fSizeX,&fSizeY);
  m_pcArena->GetResolution(&unNumCellsX,&unNumCellsY);

  double fCellSizeX = fSizeX/(double)unNumCellsX;
  double fCellSizeY = fSizeY/(double)unNumCellsY;
  if( fabs(fCellSizeX - fCellSizeY)>1.23456e-7 )
  {
      static bool bErrorMessagePrinted = false;
      if (!bErrorMessagePrinted)
      {

        printf("ERROR: in CCollisionManager::CalculateAndGetCollisionsWithWalls wrong cell size: fSizeX/unNumCellsX:%f , fSizeY/unNumCellsY:%f , diff:%f\n",fCellSizeX,fCellSizeY,fCellSizeX-fCellSizeY);
        bErrorMessagePrinted = true;
      }
      return vWallPositions;      
  }

  double fCellSize = fCellSizeX;
  
  bool bWallEncountered = false;
  dVector2 vOwnPosition = pc_collision_object->GetPosition();
  double fX = vOwnPosition.x;
  double fY = vOwnPosition.y;

  double fDistanceToLeftCellBorder 	= ((fX+fSizeX/2)/fCellSize)*fCellSize-(unsigned int)((fX+fSizeX/2)/fCellSize)*fCellSize;
  double fDistanceToRightCellBorder 	= fCellSize - fDistanceToLeftCellBorder;
  double fDistanceToBottomCellBorder 	= ((fY+fSizeY/2)/fCellSize)*fCellSize-(unsigned int)((fY+fSizeY/2)/fCellSize)*fCellSize;
  double fDistanceToTopCellBorder   	= fCellSize - fDistanceToBottomCellBorder;

  double fSbotRadius = 0.06;//CSbot::TURRET_RADIUS;  
  // check to the left
  while( !bWallEncountered && 
	 fX > vOwnPosition.x-fSbotRadius )
    {
      fX -= fCellSize;
      if( m_pcArena->GetHeight(fX,fY) == HEIGHT_OBSTACLE )
	{
	  bWallEncountered = true;
	  double fDistance = vOwnPosition.x - fX - fDistanceToRightCellBorder;
	  if( fDistance < fSbotRadius )
	    {
	      dVector2 vWallPosition;
	      vWallPosition.x = fX+fCellSize-fSbotRadius;
	      vWallPosition.y = fY;
	      vWallPositions.push_back(vWallPosition);
// 	      printf("fDistance:%f , fX:%f , ownX:%f , toLeft:%f \n",fDistance,fX,vOwnPosition.x,fDistanceToLeftCellBorder);
// 	      printf("COLLISION WITH WALL DETECTED LEFT!!! \n");
	    }
	}
    }

  // check to the right
  fX = vOwnPosition.x;
  bWallEncountered = false;
  while( !bWallEncountered && 
	 fX < vOwnPosition.x+fSbotRadius )
    {
      fX += fCellSize;
      if( m_pcArena->GetHeight(fX,fY) == HEIGHT_OBSTACLE )
	{
	  bWallEncountered = true;
	  double fDistance = fX - vOwnPosition.x - fDistanceToLeftCellBorder;
	  if( fDistance < fSbotRadius )
	    {
	      dVector2 vWallPosition;
	      vWallPosition.x = fX-fCellSize+fSbotRadius;
	      vWallPosition.y = fY;
	      vWallPositions.push_back(vWallPosition);
// 	      printf("fDistance:%f , fX:%f , ownX:%f , toRight:%f \n",fDistance,fX,vOwnPosition.x,fDistanceToRightCellBorder);
// 	      printf("COLLISION WITH WALL DETECTED RIGHT!!! \n");
	    }
	}
    }

  // check to the top
  fX = vOwnPosition.x;
  bWallEncountered = false;
  while( !bWallEncountered && 
	 fY < vOwnPosition.y+fSbotRadius )
    {
      fY += fCellSize;
      if( m_pcArena->GetHeight(fX,fY) == HEIGHT_OBSTACLE )
	{
	  bWallEncountered = true;
	  double fDistance = fY - vOwnPosition.y - fDistanceToBottomCellBorder;
	  if( fDistance < fSbotRadius )
	    {
	      dVector2 vWallPosition;
	      vWallPosition.x = fX;
	      vWallPosition.y = fY-fCellSize+fSbotRadius;
	      vWallPositions.push_back(vWallPosition);
// 	      printf("fDistance:%f , fY:%f , ownY:%f , toTop:%f \n",fDistance,fY,vOwnPosition.y,fDistanceToTopCellBorder);
// 	      printf("COLLISION WITH WALL DETECTED TOP!!! \n");
	    }
	}
    }

  // check to the bottom
  fY = vOwnPosition.y;
  bWallEncountered = false;
  while( !bWallEncountered && 
	 fY > vOwnPosition.y-fSbotRadius )
    {
      fY -= fCellSize;
      if( m_pcArena->GetHeight(fX,fY) == HEIGHT_OBSTACLE )
	{
	  bWallEncountered = true;
	  double fDistance = vOwnPosition.y - fY - fDistanceToTopCellBorder;
	  if( fDistance < fSbotRadius )
	    {
	      dVector2 vWallPosition;
	      vWallPosition.x = fX;
	      vWallPosition.y = fY+fCellSize-fSbotRadius;
	      vWallPositions.push_back(vWallPosition);
// 	      printf("fDistance:%f , fY:%f , ownY:%f , toBottom:%f \n",fDistance,fY,vOwnPosition.y,fDistanceToBottomCellBorder);
// 	      printf("COLLISION WITH WALL DETECTED BOTTOM!!! \n");
	    }
	}
    }


}//ELSE (ARENA NOT ROUND)
  return vWallPositions;
}


/******************************************************************************/
/******************************************************************************/

TCollisionObjectVector CCollisionManager::CalculateAndGetCollisionsWithWalls2(CCollisionObject* pc_collision_object)
{

	//printf("In COLL2\n");
  TCollisionObjectVector vWallCollisionObjects;	
	
	float fX,fY;
	fX=(pc_collision_object->GetPosition()).x;
	fY=(pc_collision_object->GetPosition()).y;

	  
	CAxisAlignedBoundingBox* pcAABB=pc_collision_object->GetAABB();
	double ftmpX1,ftmpX2;
	double ftmpY1,ftmpY2;
	pcAABB->GetCorners(&ftmpX1,&ftmpY1,&ftmpX2,&ftmpY2);
	  
	CCollisionObject* pcTmpCollisionObj=m_pcArena->GetHorizontalCollisionObject(fY,ftmpX1,ftmpX2);
	if(pcTmpCollisionObj!=NULL){
			vWallCollisionObjects.push_back(pcTmpCollisionObj);
			//printf("COLLISION DETECTED\n");
			//fflush(stdout);
			m_unNumberOfCollisionsDetected++;
	}
	pcTmpCollisionObj=m_pcArena->GetVerticalCollisionObject(fX,ftmpY1,ftmpY2);
	if(pcTmpCollisionObj!=NULL){
			vWallCollisionObjects.push_back(pcTmpCollisionObj);
			//printf("COLLISION DETECTED\n");
			//fflush(stdout);
			m_unNumberOfCollisionsDetected++;
	}
	return vWallCollisionObjects;	
}

/******************************************************************************/
/******************************************************************************/

bool CCollisionManager::CheckCollisionBetweenTwoObjects(CCollisionObject* pc_collision_object1,
                                                        CCollisionObject* pc_collision_object2)
{
    // Check if the objects are enabled:
    if (!pc_collision_object1->IsEnabled() || !pc_collision_object2->IsEnabled() || pc_collision_object1 == pc_collision_object2)
        return false;

    // Check if the AABBs overlap:
    if (!pc_collision_object1->GetAABB()->Overlaps(pc_collision_object2->GetAABB()))
        return false;

    CCompoundCollisionObject*  pcCompoundCollisionObject;



    // Handle compound collision objects:
    if (pc_collision_object1->GetCollisionObjectType() == COLLISION_OBJECT_TYPE_COMPOUND)
    {
		//printf("Collision manager, obj1 is compound\n");
	    pcCompoundCollisionObject = (CCompoundCollisionObject*) pc_collision_object1;
        TCollisionObjectVector* pvecCollisionChildren = pcCompoundCollisionObject->GetCollisionChildren();

        TCollisionObjectIterator i = pvecCollisionChildren->begin();

        bool bCollisionFound = false;
        while (i != pvecCollisionChildren->end() && !bCollisionFound)
        {
            bCollisionFound = CheckCollisionBetweenTwoObjects((*i), pc_collision_object2);
            //printf("collision between %s and %s : %d\n", (*i)->GetName(), pc_collision_object2->GetName(), bCollisionFound);
            i++;
        }
        //printf("Collision manager,check coll between 2 objs, returning: %d\n",bCollisionFound);
	    return bCollisionFound;
    } else if (pc_collision_object2->GetCollisionObjectType() == COLLISION_OBJECT_TYPE_COMPOUND) {
		//printf("Collision manager, obj2 is compound\n");
	    pcCompoundCollisionObject = (CCompoundCollisionObject*) pc_collision_object2;
        TCollisionObjectVector* pvecCollisionChildren = pcCompoundCollisionObject->GetCollisionChildren();

        TCollisionObjectIterator i = pvecCollisionChildren->begin();

        bool bCollisionFound = false;
        while (i != pvecCollisionChildren->end() && !bCollisionFound)
        {
            bCollisionFound = CheckCollisionBetweenTwoObjects(pc_collision_object1, (*i));
            i++;
        }
        //printf("Collision manager,check coll between 2 objs, returning: %d \n",bCollisionFound);
	    return bCollisionFound;
    
    // Handle circle/circle collisions:
    } else if (pc_collision_object1->GetCollisionObjectType() == COLLISION_OBJECT_TYPE_CIRCLE && 
               pc_collision_object2->GetCollisionObjectType() == COLLISION_OBJECT_TYPE_CIRCLE)
    {		
        double fMaxCollisionDistance = ((CCircleCollisionObject*) pc_collision_object1)->GetRadius() +
            ((CCircleCollisionObject*) pc_collision_object2)->GetRadius();

        dVector2 vDistance = pc_collision_object1->GetPosition();
        dVec2Sub(vDistance, vDistance, pc_collision_object2->GetPosition());

        if (dVec2Length(vDistance) < fMaxCollisionDistance)
        {
            pc_collision_object1->CollisionOccurred(pc_collision_object2);
			//printf("COLLISION MANAGER: COLLISION DETECTED | circle/circle\n");
			m_unNumberOfCollisionsDetected++;
            return true;
        } else {
            return false;
        }

    // Handle circle/ring collisions:
    } else if ((pc_collision_object1->GetCollisionObjectType() == COLLISION_OBJECT_TYPE_CIRCLE && 
                pc_collision_object2->GetCollisionObjectType() == COLLISION_OBJECT_TYPE_RING)
               || (pc_collision_object1->GetCollisionObjectType() == COLLISION_OBJECT_TYPE_RING && 
                   pc_collision_object2->GetCollisionObjectType() == COLLISION_OBJECT_TYPE_CIRCLE))
    {		
    	//printf("Collision manager, check collision between circle and ring\n");
    	double fMaxCollisionDistance;
    	dVector2 vCirclePosition;

    	if(pc_collision_object1->GetCollisionObjectType() == COLLISION_OBJECT_TYPE_CIRCLE){//object 1 is the circle
    		//printf("Current circle: %s\n",pc_collision_object1->GetName());
    		fMaxCollisionDistance = ((CRingCollisionObject*) pc_collision_object2)->GetRadius()-((CCircleCollisionObject*) pc_collision_object1)->GetRadius();
    		vCirclePosition=((CCircleCollisionObject*) pc_collision_object1)->GetPosition();
    	}else{//object 1 is the ring
    		//printf("Current circle: %s\n",pc_collision_object2->GetName());
    		fMaxCollisionDistance = ((CRingCollisionObject*) pc_collision_object1)->GetRadius()-((CCircleCollisionObject*) pc_collision_object2)->GetRadius();
    		vCirclePosition=((CCircleCollisionObject*) pc_collision_object2)->GetPosition();
    	}
    	//printf("Circle position: %f - %f \n",vCirclePosition.x,vCirclePosition.y);
    	//printf("Max collision distance: %f\n",fMaxCollisionDistance);
	    if (dVec2Length(vCirclePosition) >= fMaxCollisionDistance-0.0001)
        {
            pc_collision_object1->CollisionOccurred(pc_collision_object2);
			//printf("COLLISION MANAGER: COLLISION DETECTED | circle/ring\n");
			m_unNumberOfCollisionsDetected++;
            return true;
        } else {
            return false;
        }

    // Handle circle/rectangle collisions:
    } else if ((pc_collision_object1->GetCollisionObjectType() == COLLISION_OBJECT_TYPE_CIRCLE && 
                pc_collision_object2->GetCollisionObjectType() == COLLISION_OBJECT_TYPE_RECTANGLE)
               || (pc_collision_object1->GetCollisionObjectType() == COLLISION_OBJECT_TYPE_RECTANGLE && 
                   pc_collision_object2->GetCollisionObjectType() == COLLISION_OBJECT_TYPE_CIRCLE))
    {
        //printf("Collision manager, check collision between circle and rectangle\n");
        CRectangleCollisionObject* pcRectangle;
        CCircleCollisionObject*    pcCircle;

        // Figure out which one of the objects is the circle and which is the rectangle:
        if (pc_collision_object1->GetCollisionObjectType() == COLLISION_OBJECT_TYPE_CIRCLE)
        {
            pcRectangle = (CRectangleCollisionObject*) pc_collision_object2;
            pcCircle    = (CCircleCollisionObject*) pc_collision_object1;
        } else {
            pcRectangle = (CRectangleCollisionObject*) pc_collision_object1;
            pcCircle    = (CCircleCollisionObject*) pc_collision_object2;
        }
        // Move and rotate the center of the circle with respect to the pos and rot of the
        // rectangle:
        dVector2 vTranslatedCircleCenter = pcCircle->GetPosition();
        dVector2 vRectanglePosition      = pcRectangle->GetPosition();
        dVec2Sub(vTranslatedCircleCenter, vRectanglePosition, vTranslatedCircleCenter);

        double fRectangleRotation = pcRectangle->GetRotation();

        dVec2Rotate(-fRectangleRotation, vTranslatedCircleCenter);

        // Move everything to the first quadrant:
        vTranslatedCircleCenter.x = fabs(vTranslatedCircleCenter.x);
        vTranslatedCircleCenter.y = fabs(vTranslatedCircleCenter.y);
        
        double fHalfSizeX = pcRectangle->GetHalfSizeX();
        double fHalfSizeY = pcRectangle->GetHalfSizeY();

        // Check if anything touches anything:
        if ((vTranslatedCircleCenter.x - pcCircle->GetRadius() > fHalfSizeX) || 
            (vTranslatedCircleCenter.y - pcCircle->GetRadius() > fHalfSizeY))
        {
            return false;
        } else {
            // Check if the circle overlaps with any of the sides of the rectangle:
            if (vTranslatedCircleCenter.x < fHalfSizeX || vTranslatedCircleCenter.y < fHalfSizeY)
            {
			    //printf("COLLISION MANAGER: COLLISION DETECTED 01\n");
				m_unNumberOfCollisionsDetected++;
                return true;
            } else {
                // Check if the circle overlaps with a corner:
                double fDistanceFromRectangleCorner = vTranslatedCircleCenter.x - fHalfSizeX;
                double fTemp = vTranslatedCircleCenter.y - fHalfSizeY;

                fDistanceFromRectangleCorner = sqrt(fDistanceFromRectangleCorner * fDistanceFromRectangleCorner + fTemp * fTemp);
                if (fDistanceFromRectangleCorner < pcCircle->GetRadius())
				{
				    //printf("COLLISION MANAGER: COLLISION DETECTED 02\n");
					m_unNumberOfCollisionsDetected++;
                	return fDistanceFromRectangleCorner < pcCircle->GetRadius();
				}
            }
        }

    // Handle rectangle/rectangle collisions:    
    } else if (pc_collision_object1->GetCollisionObjectType() == COLLISION_OBJECT_TYPE_RECTANGLE && 
               pc_collision_object2->GetCollisionObjectType() == COLLISION_OBJECT_TYPE_RECTANGLE)
    {
        //printf("Collision manager, check collision between rectangle and rectangle\n");
        CRectangleCollisionObject* pcRectangle1 = (CRectangleCollisionObject*) pc_collision_object1;
        CRectangleCollisionObject* pcRectangle2 = (CRectangleCollisionObject*) pc_collision_object2; 

        bool bCollision = pcRectangle1->CheckCollisionWithRectangle(pcRectangle2);
		if(bCollision) 	{
			//printf("COLLISION MANAGER: COLLISION DETECTED | rectangle/rectangle\n");
			m_unNumberOfCollisionsDetected++;}
        return bCollision;
    
    // Handle ring/rectangle collisions:
    } else if ((pc_collision_object1->GetCollisionObjectType() == COLLISION_OBJECT_TYPE_RING && 
                pc_collision_object2->GetCollisionObjectType() == COLLISION_OBJECT_TYPE_RECTANGLE)
               || (pc_collision_object1->GetCollisionObjectType() == COLLISION_OBJECT_TYPE_RECTANGLE && 
                   pc_collision_object2->GetCollisionObjectType() == COLLISION_OBJECT_TYPE_RING))
    {
        //printf("Collision manager, check collision between ring and rectangle\n");
        CRectangleCollisionObject* pcRectangle;
        CRingCollisionObject*    pcRing;

        // Figure out which one of the objects is the ring and which is the rectangle:
        if (pc_collision_object1->GetCollisionObjectType() == COLLISION_OBJECT_TYPE_RING)
        {
            pcRectangle = (CRectangleCollisionObject*) pc_collision_object2;
            pcRing    = (CRingCollisionObject*) pc_collision_object1;
        } else {
            pcRectangle = (CRectangleCollisionObject*) pc_collision_object1;
            pcRing    = (CRingCollisionObject*) pc_collision_object2;
        }
        
        dVector2 vRectanglePosition = pcRectangle->GetPosition();
        double fHalfSizeX = pcRectangle->GetHalfSizeX();
        double fHalfSizeY = pcRectangle->GetHalfSizeY();
        dVector2 vDownRightCorner, vUpRightCorner, vDownLeftCorner, vUpLeftCorner;
        vDownRightCorner.x = fHalfSizeX;
        vDownRightCorner.y = -fHalfSizeY;
        vUpRightCorner.x = fHalfSizeX;
        vUpRightCorner.y = fHalfSizeY;
        vDownLeftCorner.x = -fHalfSizeX;
        vDownLeftCorner.y = -fHalfSizeY;
        vUpLeftCorner.x = -fHalfSizeX;
        vUpLeftCorner.y = fHalfSizeY;
        
        double fRectangleRotation = pcRectangle->GetRotation();
        dVec2Rotate(fRectangleRotation, vDownRightCorner);
        dVec2Rotate(fRectangleRotation, vUpRightCorner);
        dVec2Rotate(fRectangleRotation, vDownLeftCorner);
        dVec2Rotate(fRectangleRotation, vUpLeftCorner);
        
        dVec2Add(vDownRightCorner, vRectanglePosition, vDownRightCorner);
        dVec2Add(vUpRightCorner, vRectanglePosition, vUpRightCorner);
        dVec2Add(vDownLeftCorner, vRectanglePosition, vDownLeftCorner);
        dVec2Add(vUpLeftCorner, vRectanglePosition, vUpLeftCorner);
        
        double fRingRadius = pcRing->GetRadius();
        
        if(  vDownRightCorner.x > fRingRadius || vDownRightCorner.y > fRingRadius
          || vUpRightCorner.x > fRingRadius || vUpRightCorner.y > fRingRadius
          || vDownLeftCorner.x > fRingRadius || vDownLeftCorner.y > fRingRadius
          || vUpLeftCorner.x > fRingRadius || vUpLeftCorner.y > fRingRadius  )
        {
            m_unNumberOfCollisionsDetected++;
            return true;
        }
        else
        {
            return false;
        }
        
    }
}

/******************************************************************************/
/******************************************************************************/

vector<dVector2> CCollisionManager::CalculateAndGetCollisionsWithWallsDetailed(CCollisionObject* pc_collision_object)
{
    double fX1;
    double fY1;
    double fX2;
    double fY2;

    CCollisionObject* pcWallCollisionObject;
    pc_collision_object->GetAABB()->GetCorners(&fX1, &fY1, &fX2, &fY2);

    vector<dVector2> vecReturn;   

    // Check top:
    pcWallCollisionObject = m_pcArena->GetHorizontalCollisionObject(fY2, fX1 + 0.05, fX2 - 0.05);
    if (pcWallCollisionObject != NULL)
    {
        if (CheckCollisionBetweenTwoObjects(pc_collision_object, pcWallCollisionObject))
        {
            dVector2 v = { 0, -0.1 };
            vecReturn.push_back(v);
        } 
        ((CCompoundCollisionObject*) pcWallCollisionObject)->DeleteChildren();
        delete pcWallCollisionObject;
    }
    
    // Check bottom:
    pcWallCollisionObject = m_pcArena->GetHorizontalCollisionObject(fY1, fX1 + 0.05, fX2 - 0.05);
    if (pcWallCollisionObject != NULL)
    {
        if (CheckCollisionBetweenTwoObjects(pc_collision_object, pcWallCollisionObject))
        {
            dVector2 v = { 0, 0.1 };
            vecReturn.push_back(v);
        } 
        ((CCompoundCollisionObject*) pcWallCollisionObject)->DeleteChildren();
        delete pcWallCollisionObject;
    }

    // Check left:
    pcWallCollisionObject = m_pcArena->GetVerticalCollisionObject(fX1, fY1 + 0.05, fY2 - 0.05);
    if (pcWallCollisionObject != NULL)
    {
        if (CheckCollisionBetweenTwoObjects(pc_collision_object, pcWallCollisionObject))
        {
            dVector2 v = { 0.1, 0 };
            vecReturn.push_back(v);
        } 
        ((CCompoundCollisionObject*) pcWallCollisionObject)->DeleteChildren();
        delete pcWallCollisionObject;
    }
    
    // Check right:
    pcWallCollisionObject = m_pcArena->GetVerticalCollisionObject(fX2, fY1 + 0.05, fY2 - 0.05);
    if (pcWallCollisionObject != NULL)
    {
        if (CheckCollisionBetweenTwoObjects(pc_collision_object, pcWallCollisionObject))
        {
            dVector2 v = { -0.1, 0 };
            vecReturn.push_back(v);
        } 
        ((CCompoundCollisionObject*) pcWallCollisionObject)->DeleteChildren();
        delete pcWallCollisionObject;
    }

    return vecReturn;
}

/******************************************************************************/
/******************************************************************************/

CCollisionManager* CCollisionManager::GetInstance()
{
    return m_pcInstance;
}

/******************************************************************************/
/******************************************************************************/

unsigned int CCollisionManager::GetTotalNumberOfCollisions()
{
    return m_unNumberOfCollisionsDetected;
}

/******************************************************************************/
/******************************************************************************/
