#ifndef INDIVIDUAL_H_
#define INDIVIDUAL_H_

/******************************************************************************/
/******************************************************************************/

#define FITNESS_NOT_SET (-1)
#define BIT_X_GENE      8

#include <math.h>
#include <vector>
#include <list>
#include "general.h"
using namespace std;

/******************************************************************************/
/******************************************************************************/

class CIndividual
{
public:
    CIndividual(double* pf_chromosome, unsigned int un_chromosome_length);
    CIndividual(unsigned int* pt_chromosome, unsigned int un_chromosome_length);
    CIndividual(const CIndividual& original);
    virtual ~CIndividual();
    
    virtual bool          IsBinaryEncoded();
    virtual double*       GetChromosome();
    virtual unsigned int* GetBinaryChromosome();
    virtual unsigned int  GetChromosomeLength();
    virtual unsigned int  GetBinaryChromosomeLength();

    virtual void   SetFitness(double f_fitness);
    virtual double GetFitness();
	
	//DEBUGGING FUNCTION, TO BE REMOVED
	void Print();	

    bool operator< (const CIndividual& rhs);

    static bool FitnessLessThan(CIndividual* lhs, CIndividual* rhs);

protected:
    double        m_fFitness;
    double*       m_pfChromosome;
    unsigned int* m_ptBinaryChromosome;
    unsigned int  m_unChromosomeLength;
    bool          m_bBinaryEncoding;
};

/******************************************************************************/
/******************************************************************************/

#endif
