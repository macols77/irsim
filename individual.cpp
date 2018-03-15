#include "individual.h"

/******************************************************************************/
/******************************************************************************/

CIndividual::CIndividual(double* pf_chromosome, unsigned int un_chromosome_length) :
    m_fFitness(FITNESS_NOT_SET), 
    m_bBinaryEncoding(false),
    m_unChromosomeLength(un_chromosome_length)
{
    m_pfChromosome = new double[m_unChromosomeLength];
    memcpy(m_pfChromosome, pf_chromosome, sizeof(double) * m_unChromosomeLength);
    m_ptBinaryChromosome = NULL;
}

/******************************************************************************/
/******************************************************************************/

CIndividual::CIndividual(unsigned int* pt_chromosome, unsigned int un_chromosome_length) :
    m_fFitness(FITNESS_NOT_SET), 
    m_bBinaryEncoding(true),
    m_unChromosomeLength(un_chromosome_length)
{
    m_pfChromosome       = new double[m_unChromosomeLength];
    m_ptBinaryChromosome = new unsigned int[m_unChromosomeLength*BIT_X_GENE];
    for( int i = 0; i < m_unChromosomeLength; i++ ) {
      double gene_value = 0;
      int    exp_factor = 1;
      for( int j = 0; j < BIT_X_GENE; j++ ) {
	int index = j + i*BIT_X_GENE;
	m_ptBinaryChromosome[index] = pt_chromosome[index];

	gene_value += pt_chromosome[index]*exp_factor;
	exp_factor *= 2;
      }
      m_pfChromosome[i] = gene_value/(exp_factor-1)*20.0 - 10.0;
    }
}

/******************************************************************************/
/******************************************************************************/

CIndividual::CIndividual(const CIndividual& original)
{
    // Copy all the fields and make a deep copy of the chromosome

    this->m_fFitness           = original.m_fFitness;
    this->m_unChromosomeLength = original.m_unChromosomeLength;
    this->m_pfChromosome       = new double[m_unChromosomeLength];
    this->m_bBinaryEncoding    = original.m_bBinaryEncoding;
    memcpy(this->m_pfChromosome, original.m_pfChromosome, m_unChromosomeLength * sizeof(double));

    if( this->m_bBinaryEncoding )
    {
        this->m_ptBinaryChromosome = new unsigned int[m_unChromosomeLength*BIT_X_GENE];
        memcpy(this->m_ptBinaryChromosome, original.m_ptBinaryChromosome, m_unChromosomeLength * BIT_X_GENE * sizeof(unsigned int));
    } else {
        m_ptBinaryChromosome = NULL;
    }
}

/******************************************************************************/
/******************************************************************************/

CIndividual::~CIndividual()
{
    delete[] m_pfChromosome;
    
    if( m_ptBinaryChromosome != NULL )
      delete[] m_ptBinaryChromosome;
}

/******************************************************************************/
/******************************************************************************/

bool CIndividual::IsBinaryEncoded()
{
    return m_bBinaryEncoding;
}

/******************************************************************************/
/******************************************************************************/

double* CIndividual::GetChromosome()
{
    return m_pfChromosome;
}

/******************************************************************************/
/******************************************************************************/

unsigned int* CIndividual::GetBinaryChromosome()
{
    if( m_ptBinaryChromosome == NULL ) {
		printf("the chromosome in not bynary!!");
		fflush(stdout);
	}
    return m_ptBinaryChromosome;
}

/******************************************************************************/
/******************************************************************************/

unsigned int CIndividual::GetChromosomeLength()
{
    return m_unChromosomeLength;
}

/******************************************************************************/
/******************************************************************************/

unsigned int CIndividual::GetBinaryChromosomeLength()
{
    return m_unChromosomeLength * BIT_X_GENE;
}

/******************************************************************************/
/******************************************************************************/

void CIndividual::SetFitness(double f_fitness)
{
    if (m_fFitness == FITNESS_NOT_SET)
    {
        m_fFitness = f_fitness;
    } else {

        printf("Fitness has already been set for this individual! You should\n" 
              "only set the fitness for an individual once!");
		fflush(stdout);
    }
}

/******************************************************************************/
/******************************************************************************/

double CIndividual::GetFitness()
{
	//printf("CIndividual::GetFitness %2f\n",m_fFitness);
    return m_fFitness;
}

/******************************************************************************/
/******************************************************************************/

bool CIndividual::operator< (const CIndividual& rhs)
{
    return (m_fFitness > rhs.m_fFitness);

}

/******************************************************************************/
/******************************************************************************/

bool CIndividual::FitnessLessThan(CIndividual* lhs, CIndividual* rhs) 
{

	//printf("CIndividual:: KK ");
    return lhs->GetFitness() < rhs->GetFitness(); 
} 

/******************************************************************************/
/******************************************************************************/

void CIndividual::Print(){
	for(int y=0;y<m_unChromosomeLength;y++){
		printf(" // %f  // ",m_pfChromosome[y]);
	}
	printf("\n");
}
