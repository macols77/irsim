#include "simmath.h"

dVector2 ZEROVECTOR2 = { 0, 0 };
dVector2 UNITVECTOR2 = { 1, 0 };
 

/******************************************************************************/
/******************************************************************************/

double ModPI( double angle ) {
  while( angle < 0 )
    angle += 2.0*M_PI;

  while( angle > 2.0*M_PI )
    angle -= 2.0*M_PI;

  return angle;
}


/******************************************************************************/
/******************************************************************************/

bool   RangeBoundLess( dRangeBound l1, dRangeBound l2 ) {
  return (l1.angle < l2.angle );
}

/******************************************************************************/
/******************************************************************************/

// compute the cross-correlations of two series
double CrossCorrelation( vector<double>* vec_first, vector<double>* vec_second, int n_num_steps ) {
  // compute the correlation series with null delay
  double fCCNumeratorSum         = 0.0;
  double fCCDenominatorSumFirst  = 0.0;
  double fCCDenominatorSumSecond = 0.0;

  for( int j = 0; j < n_num_steps; j++ ) {
    double fCCNumElemFirst  = (*vec_first)[j];
    double fCCNumElemSecond = (*vec_second)[j];

    fCCNumeratorSum         += fCCNumElemFirst*fCCNumElemSecond;
    fCCDenominatorSumFirst  += fCCNumElemFirst*fCCNumElemFirst;
    fCCDenominatorSumSecond += fCCNumElemSecond*fCCNumElemSecond;
  }

  double fCCDenominator    = sqrt(fCCDenominatorSumFirst*fCCDenominatorSumSecond);
  double fCrossCorrelation = fCCNumeratorSum/fCCDenominator;

  return fCrossCorrelation;
}
/****************************************/
/****************************************/

bool DoubleEq( const double x, const double y ) {
  //printf("X: %2f, Y: %2f, fabs: %2f, operation: %2f\n", x, y, fabs(x-y), EPSILON * Max ( fabs(x), fabs(y) ));
  if ( fabs ( x - y ) <=EPSILON )
    return true;
  else 
    return false;
      //return fabs( x - y ) <= EPSILON * Max ( fabs(x), fabs(y) );
}

/****************************************/
/****************************************/

bool DoubleEqAbsolute( const double x, const double y, const double absolute_epsylon ) {
	return fabs( x - y ) <= absolute_epsylon;
}

/****************************************/
/****************************************/

bool DoubleGT( const double x, const double y ) {
	return x > y;
}
///****************************************/
///****************************************/
bool DoubleLT( const double x, const double y ) {
	return x < y;
}

/****************************************/
/****************************************/
double Max( const double v1, const double v2 ) {
	return ( v1 > v2 ? v1 : v2 );
}

/****************************************/
/****************************************/

double Max( const int v1, const int v2 ) {
	return ( v1 > v2 ? v1 : v2 );
}

/****************************************/
/****************************************/

double Min( const double v1, const double v2 ) {
	return ( v1 < v2 ? v1 : v2 );
}

/****************************************/
/****************************************/

double Min( const int v1, const int v2 ) {
	return ( v1 < v2 ? v1 : v2 );
}

