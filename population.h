#ifndef POPULATION_H_
#define POPULATION_H_

/******************************************************************************/
/******************************************************************************/

#include "individual.h"

/******************************************************************************/
/******************************************************************************/

class CPopulation
{
public:
    CPopulation();
    virtual ~CPopulation();

    // Creates a random population:
    virtual void CreateRandomPopulation(unsigned int un_number_of_chromosomes) = 0;

    // Creates a random population, starting from the given chromosome file and apply random mutations:
    virtual void CreateRandomPopulation(unsigned int un_number_of_chromosomes, const char*) = 0;

    // Return the number of the current generation, the first generation has
    // number 0 and so forth
    virtual unsigned int GetNumberOfCurrentGeneration()  = 0;

    // Get the number of chromosomes:
    virtual unsigned int GetNumberOfChromosomes()  = 0;

    // Get the number of chromosomes evaluated from the current generation:
    virtual unsigned int GetNumberOfChromosomesEvaluated()  = 0;


    // Gets the next chromosome to evaluate. Returns true if there were
    // one or more chromosomes to evaluate at the time of the call. In
    // that case the actual parameters are valid and have been assigned.
    // If there are no more chromosomes to evaluate, false is returned and
    // the actual parameters will be assigned the value NULL and 0.
    // Thus, keep calling this function until false is returned. If you want
    // more generations call CreateNextGeneration() when 
    // GetNextChromosomeToEvaluate() returns false, and then get the next 
    // chromosome till the function returns false, create the next generation
    // etc. 
    // The parameter pc_individual is a pointer used by the population object
    // as an ID of the chromosome. You can get any number of chromosomes to 
    // evaluate and then set the evaluation results in random order, so the 
    // ID identifies this chromosome. Once you have computed a fitness value
    // for a chromosome use SetEveluationResult(...).
    //
    // Note: Implementations of this function should set the generation 
    // random seed before returning each individual.
    virtual bool GetNextChromosomeToEvaluate(void** pc_individual, 
                                             double** pf_chromosome, 
                                             unsigned int* un_chromosome_length) = 0;

    // See comment for GetNextChromosomeToEvaluate(...)
    virtual void SetEvaluationResult(void*   pc_individual,
                                     double   f_fitness) = 0;

    // Once all of the chromosomes in a generation has been evaluated, use this
    // function to create the next generation. This function should do stuff 
    // like mutation and cross-over etc.
    virtual void CreateNextGeneration() = 0;

    // Set the random seed to be used for each individual in a generation:
    virtual void SetGenerationRandomSeed(int seed);
    
    // Get the random seed used for this generation:
    virtual int  GetGenerationRandomSeed();

    // Once all of the chromosomes in a generation has been evaluated, the
    // worst/average/best fitness can be obtained by callin this function.
    virtual void GetGenerationStats(double* pf_worst_fitness, 
                                    double* pf_average_fitness,
                                    double* pf_best_fitness) = 0;


    // Get the best chromosome of the generation:
    virtual void GetBestChromosome(double** ppf_chromosome, 
                                   unsigned int* pun_chromosome_length) = 0;


    // Get the n best chromosomes of the generation:
    virtual void GetNBestChromosomes(unsigned int un_number_of_chromosomes,
                                     double*** pppf_chromosomes, 
                                     unsigned int* pun_chromosome_length);
   
    virtual void SetMutationRate(double f_mutation_rate);

	virtual void SetCrossoverRate(double f_mutation_rate);

    virtual void SetElitism(unsigned int un_number_of_elites);

    virtual void SetNumberOfCrossovers(unsigned int un_number_of_crossovers);

    virtual void SetCrossoverDistance(unsigned int un_crossover_distance);

    // Save the current population to a file (or path):
    virtual bool SavePopulation(const char* pf_filename) = 0;

    // Load a previously saved population from a file (or path):
    virtual bool LoadPopulation(const char* pf_filename) = 0;

protected:
    int           m_nGenerationRandomSeed;
    unsigned int  m_unNumberOfElites;
	//This should be an upper bound for the real number of croccovers applied
    unsigned int  m_unNumberOfCrossovers;
    unsigned int  m_unCrossoverDistance;
    
    double        m_fMutationRate;
	double        m_fCrossoverRate;
};

/******************************************************************************/
/******************************************************************************/

#endif
