#ifndef LIGHTOBJECT_H_
#define LIGHTOBJECT_H_

/******************************************************************************/
/******************************************************************************/

#include "geometry.h"

class CLightObject;
	
/******************************************************************************/
/******************************************************************************/

class CLightObject : public CGeometry
{
public:
    CLightObject(const char* pch_name);
    ~CLightObject();

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

	void Switch 	( int n_value );
	int GetStatus ( void );

	int GetTiming ( unsigned int n_step_number );
  
  void Reset ( void );
private:
	float m_fGrey;
	dVector2 m_dCenter;
	float m_fIntRadius;
	float m_fExtRadius;
	float m_fHeight;

	int m_nActivation;
};

/******************************************************************************/
/******************************************************************************/

#endif
