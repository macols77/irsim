#ifndef STANDARDGAPOPULATION_H_
#define STANDARDGAPOPULATION_H_

/******************************************************************************/
/******************************************************************************/

#include <math.h>
#include <vector>
#include <list>

#include <fstream>

using namespace std;

#include "population.h"
#include "individual.h"

/******************************************************************************/
/******************************************************************************/


class CStandardGAPopulation : public CPopulation
{
public:
    CStandardGAPopulation(unsigned int un_chromosome_length);

    virtual ~CStandardGAPopulation();

    virtual void CreateRandomPopulation(unsigned int un_number_of_individuals);
    virtual void CreateRandomPopulation(unsigned int un_number_of_individuals,const char* pch_filename); //creates randomized population starting from the given chromosome file
    virtual unsigned int GetNumberOfCurrentGeneration();
    virtual unsigned int GetNumberOfChromosomes();
    virtual unsigned int GetNumberOfChromosomesEvaluated();

    virtual bool GetNextChromosomeToEvaluate(void** pc_individual, 
                                             double** pf_chromosome, 
                                             unsigned int* un_chromosome_length);


    virtual void SetEvaluationResult(void*   pc_individual,
                                     double    f_fitness);

    virtual void CreateNextGeneration();

    virtual void GetBestChromosome(double** ppf_chromosome, 
                                   unsigned int* pun_chromosome_length);

    virtual void GetGenerationStats(double* pf_worst_fitness, 
                                    double* pf_average_fitness,
                                    double* pf_best_fitness);

    virtual bool SavePopulation(const char* pf_filename);
    virtual bool LoadPopulation(const char* pf_filename);

    virtual void SetRankBasedSelectionCoefficient(double f_selection_coefficient);

    virtual void SelectRandomIndividualAsBest();

	virtual void SetUpperBound(double);
	virtual void SetLowerBound(double);
	virtual double GetLowerBound();
	virtual double GetUpperBound();
	
	virtual void SetCrossoverOn(bool);
	
protected:
    void DeleteCurrentPopulation();
    void ResetGeneration();

protected:
    unsigned int    m_unChromosomeLength;
    unsigned int    m_unNumberOfChromosomes;
    unsigned int    m_unCurrentGeneration;

    unsigned int    m_unNumberOfChromosomesEvaluated;
    unsigned int    m_unNextChromosomeToEvaluate;

    vector<CIndividual*>* m_vecIndividuals;

    double          m_fMaxFitness;
    double          m_fAccumulatedFitness;
    double          m_fMinFitness;

    CIndividual*    m_pcBestIndividual;
    double          m_fRankBasedCoefficient;

	double m_fUpperBound;
	double m_fLowerBound;
	
	bool m_bIsCrossoverOn;
};

/******************************************************************************/
/******************************************************************************/

#endif
