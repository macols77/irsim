

/***************************************************************************
                          Random.cc  -  description
                             -------------------
    begin                : Fri Nov 10 2000
    copyright            : (C) 2000 by Christian Blum
    email                : cblum@ulb.ac.be
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/
#include "random.h"
#include <math.h>
#include <stdio.h>
#define VERBOSE(x) x
#define VERYVERBOSE(x)
#define PI 3.14159265358979323846264338327

// Any static (but not constant) _class_ variables must be explicitly
/// instantiated once (and only once), so we do it here.
/// Seed the random number generator
long int Random::seed = 123456789;   
/* pseudo-random number generator as proposed in numerical recipes in C 
   Input:   a long value; has to be the seed variable
   Output:  a pseudo-random number uniformly distributed in [0,1]
   Side effects: changes the value of the input variable, must be this way
*/
double Random::ran01( long *idum )
{
  long k;
  double ans;
  
//  printf("seed: %d", *idum);
  k =((*idum))/IQ;
  *idum = 1 + IA * (*idum - k * IQ) - IR * k;
  if (*idum < 0 ) *idum += IM;
  ans = AM * (*idum);
//  printf(", seed: %d\n", *idum);
  return ans;
}

  // uniformly distributed double in [lbound, ubound)
double Random::nextDouble(double lbound, double ubound){
  return nextDouble() * (ubound - lbound) + lbound;
}

  // uniformly distributed double in [0, ubound)
double Random::nextDouble(double ubound){
  return nextDouble() * ubound;
}

  // uniformly distributed integer in {lbound, ..., ubound-1}
int Random::nextInt(int lbound, int ubound){
  // [0, ubound+lbound)
  double tmp = nextDouble(ubound - lbound);
  
  // {0, 1, 2,  ..., ubound+lbound-1}
  int result = (int)tmp;
  
  // {-lbound, ..., ubound-1}
  result += lbound;
  
  return result;
}

  // uniformly distributed integer in {0, ..., ubound-1}
int Random::nextInt(int ubound){
  return nextInt(0, ubound);
}
	
  // N(0,1) distributed double, Box Muller method
double Random::nextNormGaussian(){
  double r1 = nextDouble();
  double r2 = nextDouble();
  
  double delta1 = sqrt (-2.0 * log(r1)) * cos(2.0 * PI * r2);
  //double delta2 = sqrt (-2.0 * log(r1)) * sin(2.0 * PI * r2);
  
  return delta1;
}

  // N(mean, sigma2) distributed double
double Random::nextGaussian(double mean, double sigma){
  return sigma * nextNormGaussian() + mean;
}

  // uniformly distributed boolean
bool Random::nextBoolean(){
  return (nextDouble() < 0.5);
}

// Any static (but not constant) _class_ variables must be explicitly
/// instantiated once (and only once), so we do it here.
/// Seed the random number generator
long int Random2::seed = 123456789;   
/* pseudo-random number generator as proposed in numerical recipes in C 
   Input:   a long value; has to be the seed variable
   Output:  a pseudo-random number uniformly distributed in [0,1]
   Side effects: changes the value of the input variable, must be this way
*/
double Random2::ran01( long *idum )
{
  long k;
  double ans;
  
//  printf("seed: %d", *idum);
  k =((*idum))/IQ;
  *idum = 1 + IA * (*idum - k * IQ) - IR * k;
  if (*idum < 0 ) *idum += IM;
  ans = AM * (*idum);
//  printf(", seed: %d\n", *idum);
  return ans;
}

  // uniformly distributed double in [lbound, ubound)
double Random2::nextDouble(double lbound, double ubound){
  return nextDouble() * (ubound - lbound) + lbound;
}

  // uniformly distributed double in [0, ubound)
double Random2::nextDouble(double ubound){
  return nextDouble() * ubound;
}

  // uniformly distributed integer in {lbound, ..., ubound-1}
int Random2::nextInt(int lbound, int ubound){
  // [0, ubound+lbound)
  double tmp = nextDouble(ubound - lbound);
  
  // {0, 1, 2,  ..., ubound+lbound-1}
  int result = (int)tmp;
  
  // {-lbound, ..., ubound-1}
  result += lbound;
  
  return result;
}

  // uniformly distributed integer in {0, ..., ubound-1}
int Random2::nextInt(int ubound){
  return nextInt(0, ubound);
}
	
  // N(0,1) distributed double, Box Muller method
double Random2::nextNormGaussian(){
  double r1 = nextDouble();
  double r2 = nextDouble();
  
  double delta1 = sqrt (-2.0 * log(r1)) * cos(2.0 * PI * r2);
  //double delta2 = sqrt (-2.0 * log(r1)) * sin(2.0 * PI * r2);
  
  return delta1;
}

  // N(mean, sigma2) distributed double
double Random2::nextGaussian(double mean, double sigma){
  return sigma * nextNormGaussian() + mean;
}

  // uniformly distributed boolean
bool Random2::nextBoolean(){
  return (nextDouble() < 0.5);
}

// #endif
