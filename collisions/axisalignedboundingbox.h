#ifndef AXISALIGNEDBOUNDINGBOX_H
#define AXISALIGNEDBOUNDINGBOX_H

/******************************************************************************/
/******************************************************************************/
// This is an axis aligned bounding box (AABB) class. A standard thing for doing
// collision detection. From the interface you can set the center and the size of
// the bounding box, but internally we just keep track of the corner coordinates
// of the box, because that speeds up the collision detection a fair bit.
/******************************************************************************/
/******************************************************************************/

#include "simmath.h"

/******************************************************************************/
/******************************************************************************/

//Used for fast collision check

class CAxisAlignedBoundingBox
{
public:
    CAxisAlignedBoundingBox(double f_center_x, double f_center_y);
    void MoveTo(double f_x, double f_y);
    
    void Reset(double f_center_x, double f_center_y);
    void Reset(double f_center_x, double f_center_y, double f_size_x, double f_size_y);

    bool Overlaps(CAxisAlignedBoundingBox* pcAABB);
    void Add(CAxisAlignedBoundingBox* pcAABB);

    void GetCorners(double* pf_x1, double* pf_y1, double* pf_x2, double* pf_y2);
    
protected:
    double m_fX1;
    double m_fX2;
    double m_fY1;
    double m_fY2;
    
};
/******************************************************************************/
/******************************************************************************/

#endif
