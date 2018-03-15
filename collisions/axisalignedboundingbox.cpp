#include "axisalignedboundingbox.h"

#include <math.h>
#include <vector>
#include <list>

using namespace std;

/******************************************************************************/
/******************************************************************************/

CAxisAlignedBoundingBox::CAxisAlignedBoundingBox(double f_center_x, double f_center_y)
{
    Reset(f_center_x, f_center_y);
}
    
/******************************************************************************/
/******************************************************************************/

void CAxisAlignedBoundingBox::MoveTo(double f_x, double f_y)
{
    m_fX1 = f_x - (m_fX2 - m_fX1) / 2;
    m_fX2 = f_x + (m_fX2 - m_fX1) / 2;

    m_fY1 = f_y - (m_fY2 - m_fY1) / 2;
    m_fY2 = f_y + (m_fY2 - m_fY1) / 2;   
}

/******************************************************************************/
/******************************************************************************/

void CAxisAlignedBoundingBox::Add(CAxisAlignedBoundingBox* pcAABB)
{
    if (pcAABB->m_fX1 < m_fX1)
        m_fX1 = pcAABB->m_fX1;

    if (pcAABB->m_fX2 > m_fX2)
        m_fX2 = pcAABB->m_fX2;

    if (pcAABB->m_fY1 < m_fY1)
        m_fY1 = pcAABB->m_fY1;

    if (pcAABB->m_fY2 > m_fY2)
        m_fY2 = pcAABB->m_fY2;
}

/******************************************************************************/
/******************************************************************************/

bool CAxisAlignedBoundingBox::Overlaps(CAxisAlignedBoundingBox* pcAABB)
{
    if (this->m_fX1 <= pcAABB->m_fX2)
        if (this->m_fX2 >= pcAABB->m_fX1)
            if (this->m_fY1 <= pcAABB->m_fY2)
                if (this->m_fY2 >= pcAABB->m_fY1)
                    return true;
    return false;
}

/******************************************************************************/
/******************************************************************************/

void CAxisAlignedBoundingBox::Reset(double f_center_x, double f_center_y)
{
    m_fX1 = m_fX2 = f_center_x;
    m_fY1 = m_fY2 = f_center_y;
}

/******************************************************************************/
/******************************************************************************/

void CAxisAlignedBoundingBox::Reset(double f_center_x, double f_center_y, 
                                    double f_size_x, double f_size_y)
{
    m_fX1 = f_center_x - f_size_x / 2;
    m_fX2 = f_center_x + f_size_x / 2;
    
    m_fY1 = f_center_y - f_size_y / 2;
    m_fY2 = f_center_y + f_size_y / 2;
}

/******************************************************************************/
/******************************************************************************/

void CAxisAlignedBoundingBox::GetCorners(double* pf_x1, double* pf_y1, 
                                         double* pf_x2, double* pf_y2)
{
    (*pf_x1) = m_fX1;
    (*pf_x2) = m_fX2;
    (*pf_y1) = m_fY1;
    (*pf_y2) = m_fY2;
}

/******************************************************************************/
/******************************************************************************/
