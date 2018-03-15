#include "population.h"
//#include "debug.h"

/******************************************************************************/
/******************************************************************************/

CPopulation::CPopulation() : m_nGenerationRandomSeed(0), 
                             m_unNumberOfElites(1), 
                             m_unNumberOfCrossovers(1),
                             m_unCrossoverDistance(1),
                             m_fMutationRate(0.1),
							 m_fCrossoverRate(0.0)
{
}

/******************************************************************************/
/******************************************************************************/

CPopulation::~CPopulation() 
{
}

/******************************************************************************/
/******************************************************************************/

void CPopulation::SetGenerationRandomSeed(int seed)
{
    m_nGenerationRandomSeed = seed;
}


/******************************************************************************/
/******************************************************************************/
    
int  CPopulation::GetGenerationRandomSeed()
{
    return m_nGenerationRandomSeed;
}

/******************************************************************************/
/******************************************************************************/

void CPopulation::SetElitism(unsigned int un_number_of_elites)
{
    m_unNumberOfElites = un_number_of_elites;
}

/******************************************************************************/
/******************************************************************************/

void CPopulation::SetNumberOfCrossovers(unsigned int un_number_of_crossovers)
{
    m_unNumberOfCrossovers = un_number_of_crossovers;    
}

/******************************************************************************/
/******************************************************************************/

void CPopulation::SetCrossoverDistance(unsigned int un_crossover_distance)
{
    m_unCrossoverDistance = un_crossover_distance;    
}

/******************************************************************************/
/******************************************************************************/

void CPopulation::SetMutationRate(double f_mutation_rate)
{
    m_fMutationRate = f_mutation_rate;    
}

/******************************************************************************/
/******************************************************************************/

void CPopulation::SetCrossoverRate(double f_crossover_rate){
	m_fCrossoverRate=f_crossover_rate;
}

/******************************************************************************/
/******************************************************************************/

void CPopulation::GetNBestChromosomes(unsigned int un_number_of_chromosomes,
                                      double*** pppf_chromosomes, 
                                      unsigned int* pun_chromosome_length)
{
    printf("GetNBestChromosomes is not implemented for the current population type");
	fflush(stdout);
}

/******************************************************************************/
/******************************************************************************/
