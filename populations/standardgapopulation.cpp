#include <algorithm>
#include <fstream>

#include "standardgapopulation.h"
#include "random.h"


/******************************************************************************/
/******************************************************************************/

CStandardGAPopulation::CStandardGAPopulation(unsigned int un_chromosome_length) : 
    CPopulation(), m_unChromosomeLength(un_chromosome_length), 
    m_unCurrentGeneration(0),
    m_pcBestIndividual(NULL),
	m_fLowerBound(-10.0),
	m_fUpperBound(10.0),
    m_fRankBasedCoefficient(0.9)
    
{
    m_vecIndividuals = new vector<CIndividual*>;
}


/******************************************************************************/
/******************************************************************************/

CStandardGAPopulation::~CStandardGAPopulation()
{
    DeleteCurrentPopulation();
    delete m_vecIndividuals;
}

/******************************************************************************/
/******************************************************************************/

void CStandardGAPopulation::CreateRandomPopulation(unsigned int un_number_of_individuals)
{
    // Just to be sure...
    DeleteCurrentPopulation();

    double* pfTempChromosome = (double*) malloc(m_unChromosomeLength * sizeof(double));
    for (int i = 0; i < un_number_of_individuals; i++)
    {
        for (int j = 0; j < m_unChromosomeLength; j++)
        {
            pfTempChromosome[j] = Random::nextDouble(0, 1);
        }
        
        CIndividual* pcNewIndividual = new CIndividual(pfTempChromosome, 
                                                       m_unChromosomeLength);
        m_vecIndividuals->push_back(pcNewIndividual);

    }

    m_unNumberOfChromosomes = un_number_of_individuals;

    free(pfTempChromosome);
    ResetGeneration();

}

/******************************************************************************/
/******************************************************************************/

void CStandardGAPopulation::DeleteCurrentPopulation()
{
    vector<CIndividual*>::iterator i;
    for (i = m_vecIndividuals->begin(); i != m_vecIndividuals->end(); i++)
        delete(*i);

    m_vecIndividuals->clear();
}

/******************************************************************************/
/******************************************************************************/

void CStandardGAPopulation::ResetGeneration()
{
    m_fMaxFitness          = -1e10;
    m_fAccumulatedFitness  = 0;
    m_fMinFitness          = 1e10;

    m_unNumberOfChromosomesEvaluated = 0;
    m_unNextChromosomeToEvaluate     = 0;
}

/******************************************************************************/
/******************************************************************************/

unsigned int CStandardGAPopulation::GetNumberOfCurrentGeneration()
{
    return m_unCurrentGeneration;
}

/******************************************************************************/
/******************************************************************************/

unsigned int CStandardGAPopulation::GetNumberOfChromosomes()
{
    return m_unNumberOfChromosomes;
}

/******************************************************************************/
/******************************************************************************/


bool CStandardGAPopulation::GetNextChromosomeToEvaluate(void** pc_individual, 
                                                      double** pf_chromosome, 
                                                      unsigned int* un_chromosome_length)
{

    if (m_unNextChromosomeToEvaluate < m_vecIndividuals->size())
    {
        (*pc_individual)        = (*m_vecIndividuals)[m_unNextChromosomeToEvaluate];
        (*pf_chromosome)        = (*m_vecIndividuals)[m_unNextChromosomeToEvaluate]->GetChromosome();
        (*un_chromosome_length) = (*m_vecIndividuals)[m_unNextChromosomeToEvaluate]->GetChromosomeLength();

        m_unNextChromosomeToEvaluate++;
        // Set the random seed for this generation:
				Random::set_seed(GetGenerationRandomSeed());
        return true;
    } else {
        (*pc_individual)        = NULL;
        (*pf_chromosome)        = NULL;
        (*un_chromosome_length) = 0;
        
        return false;
    }

}

/******************************************************************************/
/******************************************************************************/

void CStandardGAPopulation::SetEvaluationResult(void*   pc_individual,
                                              double   f_fitness)
{
    CIndividual* pcIndividual = (CIndividual*) pc_individual;
    if (pcIndividual->GetFitness() != FITNESS_NOT_SET) 
    {
        printf("Fitness has already been set for this chromosome!");
		fflush(stdout);
    } else {
        m_unNumberOfChromosomesEvaluated++;
        m_fAccumulatedFitness += f_fitness;        
    }
    pcIndividual->SetFitness(f_fitness);

    if (f_fitness > m_fMaxFitness)
    {
        m_pcBestIndividual = pcIndividual; 
        m_fMaxFitness      = f_fitness;         
    }
    
    if (f_fitness < m_fMinFitness)
        m_fMinFitness = f_fitness;
}

/******************************************************************************/
/******************************************************************************/

void CStandardGAPopulation::CreateNextGeneration()
{
  int i;

  m_pcBestIndividual = NULL;
  if (m_unNumberOfChromosomesEvaluated < m_vecIndividuals->size())
  {
    printf("Warning: There are %d chromosomes in the population and you have only evaluated %d", 
        m_vecIndividuals->size(), m_unNumberOfChromosomesEvaluated);
    fflush(stdout);
  }

  if (m_unNumberOfChromosomes == 0)
  {
    printf("m_unNumberOfChromosomes should be > 0 otherwise no new individuals will be created!");
    fflush(stdout);
  }


  // Sort the individuals in ascending order according to their fitness:
  sort(m_vecIndividuals->begin(), m_vecIndividuals->end(), &CIndividual::FitnessLessThan);

  vector<CIndividual*>* vecNewIndividuals = new vector<CIndividual*>;

  unsigned unIndividualsToCreate = m_vecIndividuals->size();

  // Copy elites:
  for (i = 0; i < m_unNumberOfElites; i++)
  {
    int nIndex = m_vecIndividuals->size() - i - 1;
    vecNewIndividuals->push_back(new CIndividual((*m_vecIndividuals)[nIndex]->GetChromosome(),    
          (*m_vecIndividuals)[nIndex]->GetChromosomeLength()));
    //printf("ELITE: %2f %2f\n", (*m_vecIndividuals)[nIndex]->GetChromosome()[0],(*m_vecIndividuals)[nIndex]->GetChromosome()[1]);
  }
  unIndividualsToCreate -= m_unNumberOfElites;

  /*	
  //TO BE OPTIMIZED!
  //Will contain "partial" sums of fitness values
  double pfOrderedFitness[m_unNumberOfChromosomes];
  for(int t=0;t<m_unNumberOfChromosomes;t++){
  pfOrderedFitness[t]=0.0;
  }
  int counter=1;

  for(int j=0;j<m_unNumberOfChromosomes;j++){
  for(int k=0;k<counter;k++){
  pfOrderedFitness[j]+=(*m_vecIndividuals)[k]->GetFitness();
  }
  counter++;
  } 
  */

  //TRY THIS ONE
  double pfOrderedFitness[m_unNumberOfChromosomes];
  pfOrderedFitness[0]=(*m_vecIndividuals)[0]->GetFitness();
  for ( int j = 1 ; j < m_unNumberOfChromosomes ; j++ ) {
    pfOrderedFitness[j]=pfOrderedFitness[j-1]+(*m_vecIndividuals)[j]->GetFitness();
  }
  /**/

  /* DEBUG */
  /* Print Ordered fitness */
  //for(int j=0;j<m_unNumberOfChromosomes;j++)
  //printf("Individual: %d -- pfOrderedFitness: %2f\n", j, pfOrderedFitness[j]);
  /* DEBUG */


  double* pfNewChromosome1 = (double*) malloc(m_unChromosomeLength * sizeof(double));
  double* pfNewChromosome2 = (double*) malloc(m_unChromosomeLength * sizeof(double));

  // Do the reproduction:
  while (unIndividualsToCreate > 0)
  {

    //double fSelectionValue = Random::nextDouble(0.0,pfOrderedFitness[m_unNumberOfChromosomes-1]-(*m_vecIndividuals)[0]->GetFitness());
    double fSelectionValue = Random2::nextDouble(0.0,pfOrderedFitness[m_unNumberOfChromosomes-1]);

    int nCurrentIndex=0;


    while ( !(fSelectionValue < pfOrderedFitness[nCurrentIndex]) ) {
      nCurrentIndex++;
    }
    CIndividual* pcMother = (*m_vecIndividuals)[nCurrentIndex];

    //printf("Mother: %2f, %d -- CHR %2f %2f\n", fSelectionValue, nCurrentIndex, pcMother->GetChromosome()[0], pcMother->GetChromosome()[1]);

    nCurrentIndex=0;
    fSelectionValue = Random2::nextDouble(0.0,pfOrderedFitness[m_unNumberOfChromosomes-1]-(*m_vecIndividuals)[0]->GetFitness());

    while(!(fSelectionValue < pfOrderedFitness[nCurrentIndex])){
      nCurrentIndex++;
    }

    CIndividual* pcFather = (*m_vecIndividuals)[nCurrentIndex];
    //printf("Father: %2f, %d -- CHR %2f %2f\n", fSelectionValue, nCurrentIndex, pcFather->GetChromosome()[0], pcFather->GetChromosome()[1]);

    /*        unsigned int unCrossoverPoints             = m_unChromosomeLength / m_unCrossoverDistance - 1;
              int          nNextPotentialCrossoverPoint  = m_unCrossoverDistance;
              unsigned int unNextCrossoverPoint          = 10000000;        
              for (int j = 0; j < m_unNumberOfCrossovers; j++)
              {
              int nCandidate = Random::nextInt(unCrossoverPoints);
              if (nCandidate < unNextCrossoverPoint)
              {
              unNextCrossoverPoint = nCandidate;
              }
              }
              */

    //      unsigned int unCrossoversSoFar = 0;
    bool bSwap = false;		
    //MY CROSSOVER STUFF
    int unCrossoverIndex=0; //0 just swaps the child chromosomes
    if(Random2::nextDouble() < m_fCrossoverRate){
      unCrossoverIndex=Random2::nextInt(1,m_unChromosomeLength); //nextInt(a,b) uniform int in [a,b-1]
    }
    //printf("unCrossoverIndex: %d, CrossoverRate: %2f\n", unCrossoverIndex, m_fCrossoverRate);

    for (int j = 0; j < m_unChromosomeLength; j++)
    {         
      double fGene;
      if (!bSwap)
      {
        fGene = pcMother->GetChromosome()[j]; 
        //printf("!Swarp - fGene Mother - :%2f\n", fGene);
      }
      else
      {
        fGene = pcFather->GetChromosome()[j]; 
        //printf("Swarp - fGene Father - :%2f\n", fGene);
      }

      //printf("MUT RATE: %2f\n", m_fMutationRate);
      if (Random2::nextDouble(0, 1) < m_fMutationRate)
      //if (  ((float)rand() / (float)RAND_MAX) < m_fMutationRate )
      {
        //fGene = fGene + Random2::nextNormGaussian();
        //fGene = fGene + Random2::nextGaussian(0,0.1);
        fGene = Random2::nextDouble();
        if (fGene < 0.0)
          fGene = 0.0;
        if (fGene > 1.0)
          fGene = 1.0;
        //printf("CHR1 - fGene %d Mutation: %2f\n", j, fGene);
      }

      pfNewChromosome1[j] = fGene;           

      if (bSwap)
      {
        fGene = pcMother->GetChromosome()[j]; 
        //printf("Swarp - fGene Mother - :%2f\n", fGene);
      }
      else
      {
        fGene = pcFather->GetChromosome()[j]; 
        //printf("!Swarp - fGene Father - :%2f\n", fGene);
      }

      if (Random2::nextDouble(0, 1) < m_fMutationRate)
      {
        //fGene = fGene + Random2::nextNormGaussian();
        //fGene = fGene + Random2::nextGaussian(0,0.1);
        fGene = Random2::nextDouble();
        if (fGene < 0.0)
          fGene = 0.0;
        if (fGene > 1.0)
          fGene = 1.0;
        //printf("CHR2 - fGene Mutation: %2f\n", fGene);
      }

      pfNewChromosome2[j] = fGene;           
      /*
         nNextPotentialCrossoverPoint--;
         if (nNextPotentialCrossoverPoint <= 0)
         {
         nNextPotentialCrossoverPoint = m_unCrossoverDistance;
         unNextCrossoverPoint--;

         if (unNextCrossoverPoint == 0 && unCrossoversSoFar < m_unNumberOfCrossovers && m_bIsCrossoverOn)
         {
         unNextCrossoverPoint = Random2::nextInt(unCrossoverPoints);
         unCrossoversSoFar++;
         bSwap = !bSwap;
         }
         }
         */

      //Do crossover
      if(m_bIsCrossoverOn && j==(unCrossoverIndex - 1)){
        bSwap = !bSwap;
      }

    }

    vecNewIndividuals->push_back(new CIndividual(pfNewChromosome1, m_unChromosomeLength));
    unIndividualsToCreate--;
    if (unIndividualsToCreate > 0)
    {
      vecNewIndividuals->push_back(new CIndividual(pfNewChromosome2, m_unChromosomeLength));           
      unIndividualsToCreate--;
    }
  }

  DeleteCurrentPopulation();
  delete m_vecIndividuals;
  m_vecIndividuals = vecNewIndividuals;

  free(pfNewChromosome1);
  free(pfNewChromosome2);
  ResetGeneration();

  m_unCurrentGeneration++;

  // Create a new random seed for this generation
  // (just use the random generators next int):
  //SetGenerationRandomSeed(Random::seed);

  //TO BE REMOVED	
  /*
  //PRINTS THE NEW GENERATION
  printf("Stadard ga population: end of method createnextgeneration------------------------\n\n\n");
  fflush(stdout);

  vector<CIndividual*>::iterator it=m_vecIndividuals->begin();
  while(it!=m_vecIndividuals->end()){
  (*it)->Print();
  printf("\n\n");
  fflush(stdout);
  it++;
  }

  printf("-------------------------------------------------\n\n");
  fflush(stdout);
  sleep(3);
  /**/
}

/******************************************************************************/
/******************************************************************************/

void CStandardGAPopulation::SetCrossoverOn(bool b){
	m_bIsCrossoverOn=b;	
}

/******************************************************************************/
/******************************************************************************/

void CStandardGAPopulation::GetGenerationStats(double* pf_worst_fitness, 
                                             double* pf_average_fitness,
                                             double* pf_best_fitness)
{
    if (m_unNumberOfChromosomesEvaluated == 0)
    {
        printf("You have not yet evaluated any individuals from this generation (this generation is %d)!", m_unCurrentGeneration);
		fflush(stdout);
    } else {       
        (*pf_worst_fitness)   = m_fMinFitness;
        (*pf_best_fitness)    = m_fMaxFitness;
        (*pf_average_fitness) = m_fAccumulatedFitness / m_unNumberOfChromosomesEvaluated;
    }
}


/******************************************************************************/
/******************************************************************************/


bool CStandardGAPopulation::SavePopulation(const char* pf_filename) 
{
    fstream fsFile;
    fsFile.open(pf_filename, ios::out);
    
    fsFile << m_unCurrentGeneration << " ";
    fsFile << m_unNumberOfChromosomes << " ";
    fsFile << m_unChromosomeLength << " ";
    fsFile << m_nGenerationRandomSeed << " ";
    fsFile << Random::seed << "\n";
    fsFile.precision(10);


    for (int i = 0; i < m_unNumberOfChromosomes; i++)
    {
        fsFile << i << " ";
        fsFile << (*m_vecIndividuals)[i]->GetFitness() << " ";

        double* pfChromosome = (*m_vecIndividuals)[i]->GetChromosome();
        for (int j = 0; j < m_unChromosomeLength; j++)
        {
            fsFile << pfChromosome[j] << " ";
        }            
        fsFile << "\n";
    }

    fsFile.close();
    return true;
}

/******************************************************************************/
/******************************************************************************/


bool CStandardGAPopulation::LoadPopulation(const char* pf_filename) 
{
    DeleteCurrentPopulation();

    fstream fsFile;
    fsFile.open(pf_filename);
    fsFile >> m_unCurrentGeneration;
    fsFile >> m_unNumberOfChromosomes;
    fsFile >> m_unChromosomeLength;
    fsFile >> m_nGenerationRandomSeed;
    fsFile >> Random::seed;

    double* pfChromosome = (double*) malloc(sizeof(double) * m_unChromosomeLength);

    m_unNumberOfChromosomesEvaluated = 0;
    m_unNextChromosomeToEvaluate     = 0;

    for (int i = 0; i < m_unNumberOfChromosomes; i++)
    {
        int nIndividual;

        fsFile >> nIndividual;
        if (nIndividual != i)
        {
            printf("Something is wrong with the file %s, %d should be %d", pf_filename, nIndividual, i);
			fflush(stdout);
            return false;
        }

        double fFitness;
        fsFile >> fFitness;
             
        for (int j = 0; j < m_unChromosomeLength; j++)
        {
            fsFile >> pfChromosome[j];
        }            
        
        
        CIndividual* pcIndividual = new CIndividual(pfChromosome, m_unChromosomeLength);
        if (fFitness == FITNESS_NOT_SET)
        {
            m_unNextChromosomeToEvaluate = m_vecIndividuals->size();
        } else {
            m_unNumberOfChromosomesEvaluated++;
            m_fAccumulatedFitness += fFitness;
            pcIndividual->SetFitness(fFitness);
            
            if (fFitness > m_fMaxFitness)
            {
                m_pcBestIndividual = pcIndividual; 
                m_fMaxFitness      = fFitness;         
            }
            
            if (fFitness < m_fMinFitness)
                m_fMinFitness = fFitness;
        }

        m_vecIndividuals->push_back(pcIndividual);
    }

    free(pfChromosome);
    fsFile.close();

    return true;
}

/******************************************************************************/
/******************************************************************************/

void CStandardGAPopulation::GetBestChromosome(double** ppf_chromosome, 
                                   unsigned int* pun_chromosome_length)
{
    if (m_pcBestIndividual == NULL)
    {
        printf("You have to get the best individual AFTER you have evaluated\n"
              "the generation.");            
		fflush(stdout);
    } else {
        (*ppf_chromosome)        = m_pcBestIndividual->GetChromosome();
        (*pun_chromosome_length) = m_pcBestIndividual->GetChromosomeLength();
    }
}

/******************************************************************************/
/******************************************************************************/

void CStandardGAPopulation::SetRankBasedSelectionCoefficient(double f_selection_coefficient)
{
    m_fRankBasedCoefficient = f_selection_coefficient;
}

/******************************************************************************/
/******************************************************************************/

unsigned int CStandardGAPopulation::GetNumberOfChromosomesEvaluated()
{
    return m_unNumberOfChromosomesEvaluated;
}

/******************************************************************************/
/******************************************************************************/

void CStandardGAPopulation::SelectRandomIndividualAsBest()
{
    if (m_vecIndividuals->size() == 0)
    {
        printf("Cannot select a random individual out of 0 individuals");
		fflush(stdout);
    }

    m_pcBestIndividual = (*m_vecIndividuals)[0];
}

/******************************************************************************/
/******************************************************************************/

void CStandardGAPopulation::SetUpperBound(double f_UpperBound){
	m_fUpperBound=f_UpperBound;
}

/******************************************************************************/
/******************************************************************************/

void CStandardGAPopulation::SetLowerBound(double f_LowerBound){
	m_fLowerBound=f_LowerBound;
}

/******************************************************************************/
/******************************************************************************/

double CStandardGAPopulation::GetUpperBound(){
	return m_fUpperBound;
}

/******************************************************************************/
/******************************************************************************/

double CStandardGAPopulation::GetLowerBound(){
	return m_fLowerBound;
}

/******************************************************************************/
/******************************************************************************/

void CStandardGAPopulation::CreateRandomPopulation(unsigned int un_number_of_individuals,const char* pch_filename)
{
    // Just to be sure...
    DeleteCurrentPopulation();
	
    ifstream in;
    in.open( pch_filename, ios::in );
    if( !in ) {
        printf("Cannot open chromosome file: %s", pch_filename);
		exit(1);
    }

	in >> m_unChromosomeLength;
	printf("Generating population from file: %s\n",pch_filename);
	printf("	chromosome length: %d\n",m_unChromosomeLength);
	
	double pfSampleChromosome[m_unChromosomeLength];
	
	for(int j=0;j<m_unChromosomeLength;j++){
		in >> pfSampleChromosome[j];
	//	printf("Current gene: %f\n",pfSampleChromosome[j]);
	}
	
    double* pfTempChromosome = (double*) malloc(m_unChromosomeLength * sizeof(double));
    for (int i = 0; i < un_number_of_individuals; i++)
    {
        for (int j = 0; j < m_unChromosomeLength; j++)
        {
            pfTempChromosome[j] = pfSampleChromosome[j]+Random::nextDouble(-0.1, 0.1);
			if(pfTempChromosome[j]<0.0){
				pfTempChromosome[j]=0.0;
			}else if(pfTempChromosome[j]>1.0){
				pfTempChromosome[j]=1.0;
			}
        }
        
        CIndividual* pcNewIndividual = new CIndividual(pfTempChromosome, 
                                                       m_unChromosomeLength);
        m_vecIndividuals->push_back(pcNewIndividual);

    }

    m_unNumberOfChromosomes = un_number_of_individuals;

    free(pfTempChromosome);
//	delete pfSampleChromosome;
    ResetGeneration();

}

/******************************************************************************/
/******************************************************************************/
