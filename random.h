/* -*- Mode:C++ -*- */

#ifndef RANDOM_H
#define RANDOM_H

#include <stdlib.h>

using namespace std;

class Random {
private:
  /* constants for a pseudo-random number generator, 
     for details see numerical recipes in C */
  
  // IM = IA * IQ + IR
  static const int IA = 16807;      // 75      (prime root of IM)
  static const int IM = 2147483647; // 231 - 1 (biggest prime in 32bit)
  static const double AM = (1.0/IM);// 
  static const int IQ = 127773;     // quot      (IM / IA)
  static const int IR = 2836;       // remainder (IM mod IA)
  
  static double ran01(long *idum);
  
public:
  
  static long int seed;                      
  static void set_seed(long int s) {Random::seed = s;}
  
  // uniformly distributed double in [0,1)
  static double nextDouble() { return ran01(&seed);}
  
  // uniformly distributed double in [lbound, ubound)
  static double nextDouble(double lbound, double ubound);
  
  // uniformly distributed double in [0, ubound)
  static double nextDouble(double ubound);
  
  // uniformly distributed integer in {lbound, ..., ubound-1}
  static int nextInt(int lbound, int ubound);
  
  // uniformly distributed integer in {0, ..., ubound-1}
  static int nextInt(int ubound);
  
  // N(0,1) distributed double
  static double nextNormGaussian();
  
  // N(mean, sigma2) distributed double
  static double nextGaussian(double mean, double sigma);
  
  // uniformly distributed boolean
  static bool nextBoolean();
  
};

class Random2 {
private:
  /* constants for a pseudo-random number generator, 
     for details see numerical recipes in C */
  
  // IM = IA * IQ + IR
  static const int IA = 16807;      // 75      (prime root of IM)
  static const int IM = 2147483647; // 231 - 1 (biggest prime in 32bit)
  static const double AM = (1.0/IM);// 
  static const int IQ = 127773;     // quot      (IM / IA)
  static const int IR = 2836;       // remainder (IM mod IA)
  
  static double ran01(long *idum);
  
public:
  
  static long int seed;                      
  static void set_seed(long int s) {Random::seed = s;}
  
  // uniformly distributed double in [0,1)
  static double nextDouble() { return ran01(&seed);}
  
  // uniformly distributed double in [lbound, ubound)
  static double nextDouble(double lbound, double ubound);
  
  // uniformly distributed double in [0, ubound)
  static double nextDouble(double ubound);
  
  // uniformly distributed integer in {lbound, ..., ubound-1}
  static int nextInt(int lbound, int ubound);
  
  // uniformly distributed integer in {0, ..., ubound-1}
  static int nextInt(int ubound);
  
  // N(0,1) distributed double
  static double nextNormGaussian();
  
  // N(mean, sigma2) distributed double
  static double nextGaussian(double mean, double sigma);
  
  // uniformly distributed boolean
  static bool nextBoolean();
  
};


#endif
