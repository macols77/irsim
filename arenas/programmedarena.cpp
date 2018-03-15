#include "programmedarena.h"
#include "rectanglecollisionobject.h"
#include "compoundcollisionobject.h"

/******************************************************************************/
/******************************************************************************/

CProgrammedArena::CProgrammedArena(const char* pch_name, unsigned int un_res_x, 
                                   unsigned int un_res_y, float f_size_x, 
                                   float f_size_y) : CArena(pch_name)
{
    m_pArenaPixels = (eArenaHeight*) malloc(un_res_x * un_res_y * sizeof(eArenaHeight));

    // Reset the arena:
    for (int x = 0; x < un_res_x; x++) 
        for (int y = 0; y < un_res_y; y++)
            m_pArenaPixels[x + un_res_x * y] = HEIGHT_NORMAL;

    SetResolution(un_res_x, un_res_y);
    SetSize(f_size_x, f_size_y);    
}


/******************************************************************************/
/******************************************************************************/

CProgrammedArena::~CProgrammedArena()
{
    free(m_pArenaPixels);
}

/******************************************************************************/
/******************************************************************************/

eArenaHeight CProgrammedArena::GetHeight(double x, double y)
{    
    double fX = (((x + m_fSizeX / 2) * m_unResX) / m_fSizeX);
    double fY = (((y + m_fSizeY / 2) * m_unResY) / m_fSizeY);

    if(fX >= m_unResX || fY >= m_unResY || fX < 0 || fY < 0)
    {
        return HEIGHT_HOLE;
    }

    int nX = (int) fX;
    int nY = (int) fY;

    return m_pArenaPixels[nX + nY * m_unResX];
 
}

/******************************************************************************/
/******************************************************************************/

void CProgrammedArena::SetHeightPixel(unsigned int x, unsigned int y, eArenaHeight height)
{
    m_pArenaPixels[x + m_unResX * y] = height;    
}


/******************************************************************************/
/******************************************************************************/
 
eArenaHeight CProgrammedArena::GetHeightPixel(unsigned int x, unsigned int y)
{
    return m_pArenaPixels[x + m_unResX * y];    
}

/******************************************************************************/
/******************************************************************************/

void CProgrammedArena::SetHeightPixelsFromChars(const char* pch_arena_as_chars, 
                                                char c_hole, 
                                                char c_normal, 
                                                char c_obstacle)
{
    for (int n = 0; n < m_unResX * m_unResY; n++)
    {
        if (pch_arena_as_chars[n] == c_hole)
            m_pArenaPixels[n] = HEIGHT_HOLE;
        else if (pch_arena_as_chars[n] == c_normal)
            m_pArenaPixels[n] = HEIGHT_NORMAL;
        else if (pch_arena_as_chars[n] == c_obstacle)
            m_pArenaPixels[n] = HEIGHT_OBSTACLE;
        else
            {
                printf("Unknown char: %c (use one of: hole: %c, normal: %c, obstacle: %c)", 
                      pch_arena_as_chars[n], c_hole, c_normal, c_obstacle);
                fflush(stdout);   
            }
    }
}

/******************************************************************************/
/******************************************************************************/

CCompoundCollisionObject* CProgrammedArena::GetHorizontalCollisionObject(double y, double x1, double x2)
{
    int nY = (int) (((y + m_fSizeY / 2) * (double) m_unResY) / m_fSizeY);


    int nX1 = (int) (((x1 + m_fSizeX / 2) * (double) m_unResX) / m_fSizeX);
    int nX2 = (int) (((x2 + m_fSizeX / 2) * (double) m_unResX) / m_fSizeX);

    if (nX2 < nX1)
    {
        int nTemp = nX1;
        nX1 = nX2;
        nX2 = nTemp;
    }

    if (nY < 0 || nY > m_unResY - 1)
    {
        return NULL;
    }

    if (nX1 < 0)
    {
        nX1 = 0;
    }

    if (nX2 > m_unResX - 1)
    {

        nX2 = m_unResX - 1;
    }
    
    bool bObstacleFound = false;

    do 
    {
        if (m_pArenaPixels[nX1 + m_unResX * nY] == HEIGHT_OBSTACLE)
        {
            bObstacleFound = true;
        }
        else 
        {
            nX1++;
        }
    } while (nX1 <= nX2 && !bObstacleFound);

    if (!bObstacleFound)
    {
        return NULL;
    }
    else
    {
        CCompoundCollisionObject* pcCompoundCollisionObject = new CCompoundCollisionObject("Arena_horizontal_coll_obj", NULL, 0, 0);
        double fPosY = (((double) nY  - (double) m_unResY / 2.0) * m_fSizeY) / (double) m_unResY + ((m_fSizeY / (double) m_unResY) / 2.0);
        
        do 
        {
            if (m_pArenaPixels[nX1 + m_unResX * nY] == HEIGHT_OBSTACLE)
            {
                double fPosX = (((double) nX1  - (double) m_unResX / 2.0) * m_fSizeX) / (double) m_unResX + ((m_fSizeX / (double) m_unResX) / 2.0);
                CRectangleCollisionObject* pcRectangle = new CRectangleCollisionObject("Arena_horizontal_coll_obj_rectangle", NULL, fPosX, fPosY, 0, m_fSizeX / (double) m_unResX, m_fSizeY / (double) m_unResY);

                pcCompoundCollisionObject->AddCollisionChild(pcRectangle);
            }

            nX1++;
        } while (nX1 <= nX2);        

        pcCompoundCollisionObject->ComputeNewPositionAndRotationFromParent();
        return pcCompoundCollisionObject;
    }
}

/******************************************************************************/
/******************************************************************************/


CCompoundCollisionObject* CProgrammedArena::GetVerticalCollisionObject(double x, double y1, double y2)
{
	
    int nX = (int) (((x + m_fSizeX / 2.0) * (double)m_unResX) / m_fSizeX);


    int nY1 = (int) (((y1 + m_fSizeY / 2.0) * (double)m_unResY) / m_fSizeY);
    int nY2 = (int) (((y2 + m_fSizeY / 2.0) * (double)m_unResY) / m_fSizeY);

    if (nY2 < nY1)
    {
        int nTemp = nY1;
        nY1 = nY2;
        nY2 = nTemp;
    }

    if (nX < 0 || nX > m_unResX - 1)
    {
        return NULL;
    }

    if (nY1 < 0)
    {
        nY1 = 0;
    }

    if (nY2 > m_unResY - 1)
    {

        nY2 = m_unResY - 1;
    }
    
    bool bObstacleFound = false;

    do 
    {
        if (m_pArenaPixels[nX + m_unResX * nY1] == HEIGHT_OBSTACLE)
        {
            bObstacleFound = true;
        }
        else 
        {
            nY1++;
        }
    } while (nY1 <= nY2 && !bObstacleFound);

    if (!bObstacleFound)
    {
        return NULL;
    }
    else
    {
        CCompoundCollisionObject* pcCompoundCollisionObject = new CCompoundCollisionObject("Arena_vertical_coll_obj", NULL, 0, 0);

        double fPosX = (((double) nX  - (double) m_unResX / 2.0) * m_fSizeX) / (double) m_unResX + ((m_fSizeX / (double) m_unResX) / 2.0);        
        do 
        {
            if (m_pArenaPixels[nX + m_unResX * nY1] == HEIGHT_OBSTACLE)
            {
                double fPosY = (((double) nY1  - (double) m_unResY / 2.0) * m_fSizeY) / (double) m_unResY + ((m_fSizeY / (double) m_unResY) / 2.0);

                CRectangleCollisionObject* pcRectangle = new CRectangleCollisionObject("Arena_vertical_coll_obj_rectangle", NULL, fPosX, fPosY, 0, m_fSizeX / (double) m_unResX, m_fSizeY / (double) m_unResY);

                pcCompoundCollisionObject->AddCollisionChild(pcRectangle);
            }

            nY1++;
        } while (nY1 <= nY2);        

        pcCompoundCollisionObject->ComputeNewPositionAndRotationFromParent();
        return pcCompoundCollisionObject;
    }
}


/******************************************************************************/
/******************************************************************************/

int CProgrammedArena::GetArenaType(){
	return ARENA_TYPE_SQUARE;
}
