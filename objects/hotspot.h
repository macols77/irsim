#ifndef HOTSPOT_H_
#define HOTSPOT_H_

/******************************************************************************/
/******************************************************************************/

#include "geometry.h"
//#include "render.h"

class CHotSpot;
	
/******************************************************************************/
/******************************************************************************/

class CHotSpot : public CGeometry
{
public:
    CHotSpot(const char* pch_name);
    ~CHotSpot();

	void SetColor(float fGrey);
	void GetColor(float *fGrey);

	void SetHeight ( float fHeight);
	void GetHeight ( float *fHeight);
	
	void SetCenter(dVector2 dCenter);
	void GetCenter(dVector2 *dCenter);

	void SetIntRadius(float fRadius);
	void GetIntRadius(float *fRadius);

	void SetExtRadius(float fRadius);
	void GetExtRadius(float *fRadius);
private:
	float m_fGrey;
	dVector2 m_dCenter;
	float m_fIntRadius;
	float m_fExtRadius;
	float m_fHeight;
};

/******************************************************************************/
/******************************************************************************/

#endif
