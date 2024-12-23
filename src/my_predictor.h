#include <cstdint>
#include <bitset>
#include <algorithm>

#define INT32  int32_t
#define UINT32 uint32_t

#define TAKEN 			  true
#define NOT_TAKEN 		  false	

#define BIMODAL_CTR_MAX		3
#define BIMODAL_CTR_INIT	2
#define TAGPRED_CTR_MAX		7
#define TAGPRED_CTR_INIT	0
#define BIMODALLOG   		14 // 2^14 entries in base predictor
#define NUMTAGTABLES 		4
#define TAGPREDLOG			12 // 2^12 entries in tage table
// TODO change variables names

// Entry in a TAGE compoenent
struct TagEntry 
{
    INT32 ctr;
    UINT32 tag;
    INT32 usefulBits; // TODO: change to useful bit
};

// Folded History implementation ... from GHR(geometric length) -> compressed(target)
struct CompressedHist
{
	// Objective is to convert geomLength of GHR into tagPredLog which is of length 
	// equal to the index of the corresponding bank
    // It can be also used for tag when the final length would be the Tag
    UINT32 geomLength;
    UINT32 targetLength;
    UINT32 compHist;
      
    void updateCompHist(std::bitset<131> ghr)
    {
        int mask = (1 << targetLength) - 1;
        int mask1 = ghr[geomLength] << (geomLength % targetLength);
        int mask2 = (1 << targetLength);
            compHist  = (compHist << 1) + ghr[0];
            compHist ^= ((compHist & mask2) >> targetLength);
            compHist ^= mask1;
            compHist &= mask;  
         
    }    
};

int satIncrement(UINT32 value, UINT32 max) {
    return (value < max) ? value + 1 : value;
}

int satDecrement(UINT32 value) {
    return (value > 0) ? value - 1 : value;
}

class my_update : public branch_update {
public:
	unsigned int index;
};

class my_predictor : public branch_predictor {
private:
	std::bitset<131> GHR;	// global history register
	int PHR;			// 16 bit path history
	
	// Bimodal Base Predictor
	UINT32  *bimodal;			// pattern history table (pht)
	UINT32  historyLength;		// history length
	UINT32  numBimodalEntries;	// entries in pht 
	UINT32  bimodalLog;
	
	//Tagged Predictors
	TagEntry *tagPred[NUMTAGTABLES];
	UINT32 numTagPredEntries;
	UINT32 tagPredLog;
	UINT32 geometric[NUMTAGTABLES];
	
	//Compressed Buffers
	CompressedHist indexComp[NUMTAGTABLES];
	CompressedHist tagComp[2][NUMTAGTABLES]; 

	// Predictions need to be global
	bool primePred;
	bool altPred;
	int primeBank;
	int altBank;

	// Index had to be made global else recalculate for update
	UINT32 indexTagPred[NUMTAGTABLES];
	UINT32 tag[NUMTAGTABLES];
	UINT32 clock;
	int clock_flip;
	INT32 altBetterCount;

public:
	my_update u;
	branch_info bi;

	my_predictor (void) { 

		// Initialize bimodal table (simple 2-bit counter table)
		// TODO: remove extra calculations(?)
		bimodalLog = BIMODALLOG;
		numBimodalEntries = (1 << bimodalLog);
		bimodal = new UINT32[numBimodalEntries];

		for(UINT32 i = 0; i < numBimodalEntries; i++)
			bimodal[i] = BIMODAL_CTR_INIT;
		
		// Initialize tagged predictors 
		tagPredLog = TAGPREDLOG;
		numTagPredEntries = (1 << tagPredLog);

		for(UINT32 i = 0; i < NUMTAGTABLES; i++) {
			tagPred[i] = new TagEntry[numTagPredEntries];

			for(UINT32 j = 0; j < numTagPredEntries; j++) {
				tagPred[i][j].ctr = 0;
				tagPred[i][j].tag = 0;
				tagPred[i][j].usefulBits = 0;
			}
		}

		// Geometric lengths of histry to consider, table 0 is longest
		geometric[0] = 130;
		geometric[1] = 44;
		geometric[2] = 15;
		geometric[3] = 5;

		// Initialize compressed buffers for index of tagged component 
		for(int i = 0; i < NUMTAGTABLES; i++) {
			indexComp[i].compHist = 0;
			indexComp[i].geomLength = geometric[i];
			indexComp[i].targetLength = TAGPREDLOG;
		}

		// Initialize compressed buffers for tags
        // The tables have different tag lengths
        // T2 and T3 have tag length -> 8
        // T0 and T1 have tag length -> 9
        // Second index indicates the Bank no.
        for(int j = 0; j < 2 ; j++) {
			// TODO: come back and look at this, read paper again
        	for(int i = 0; i < NUMTAGTABLES; i++) {
				tagComp[j][i].compHist = 0;
				tagComp[j][i].geomLength = geometric[i];
				if(j == 0) {
                	tagComp[j][i].targetLength = 9 ;    
            	} else {
                	tagComp[j][i].targetLength = 8 ;
            	}
        	}   
    	}

		// Preditions banks and prediction values 
		primePred = -1;
		altPred = -1;
		primeBank = NUMTAGTABLES;
		altBank = NUMTAGTABLES;
		
		for(int i=0; i < NUMTAGTABLES; i++)    
			indexTagPred[i] = 0;

		for(int i=0; i < NUMTAGTABLES; i++)
			tag[i] = 0;
		
		clock = 0;
		clock_flip = 1;
		PHR = 0;
		GHR.reset();
		altBetterCount = 8;
	}

	branch_update *predict (branch_info & b) {
		bi = b;
		if (b.br_flags & BR_CONDITIONAL) {
			// Base prediction
			bool basePrediction;
			UINT32 bimodalIndex = b.address % numBimodalEntries;
			UINT32 bimodalCounter = bimodal[bimodalIndex];

			basePrediction = (bimodalCounter > BIMODAL_CTR_MAX/2) ? TAKEN : NOT_TAKEN;

			// Hash to get tag includes info about bank, pc and global history compressed
			// formula given in PPM paper 
			// pc[9:0] xor CSR1 xor (CSR2 << 1)
			for (int i = 0; i < NUMTAGTABLES; i++) {
				tag[i] = b.address ^ tagComp[0][i].compHist ^ (tagComp[1][i].compHist << 1);
				tag[i] &= ((1 << 9) - 1);
			}
			
			// Get the index for each table
			indexTagPred[0] = b.address ^ (b.address >> TAGPREDLOG) ^ indexComp[0].compHist ^ PHR ^ (PHR >> TAGPREDLOG);
       		indexTagPred[1] = b.address ^ (b.address >> (TAGPREDLOG - 1)) ^ indexComp[1].compHist ^ (PHR);
       		indexTagPred[2] = b.address ^ (b.address >> (TAGPREDLOG - 2)) ^ indexComp[2].compHist ^ (PHR & 31);
       		indexTagPred[3] = b.address ^ (b.address >> (TAGPREDLOG - 3)) ^ indexComp[3].compHist ^ (PHR & 7);
			
			UINT32 index_mask = ((1 << TAGPREDLOG) - 1);
			for(int i = 0; i < NUMTAGTABLES; i++)
            	indexTagPred[i] &= index_mask;
			
			// Get two predictions prime and alt (alternate)
			primePred = -1;
			altPred = -1;
			primeBank = NUMTAGTABLES;
			altBank = NUMTAGTABLES;

			// See if any tags match for prime predictor
			// T0 would be best
			for(int iterator = 0; iterator < NUMTAGTABLES; iterator++) {
            	if(tagPred[iterator][indexTagPred[iterator]].tag == tag[iterator]) {
					primeBank = iterator;
					break;
				}  
       		}      
            
			// See if any tags match for alterante predictor
			for(int iterator = primeBank + 1; iterator < NUMTAGTABLES; iterator++) {
                if (tagPred[iterator][indexTagPred[iterator]].tag == tag[iterator]) {
                    altBank = iterator;
                    break;
                }  
            }
			
			
			if (primeBank < NUMTAGTABLES) { 
				// Prime predictor tag found
				  
				if(altBank == NUMTAGTABLES) {
					// alt predictor not found 
					altPred = basePrediction;
				} else {
					// alt predictor found 
					altPred = (tagPred[altBank][indexTagPred[altBank]].ctr >= TAGPRED_CTR_MAX/2) ? TAKEN : NOT_TAKEN;
					// if(tagPred[altBank][indexTagPred[altBank]].ctr >= TAGPRED_CTR_MAX/2)
					// 	altPred = TAKEN;
					// else 
					// 	altPred = NOT_TAKEN;
				}
        
				if ((tagPred[primeBank][indexTagPred[primeBank]].ctr != 3) || 
					(tagPred[primeBank][indexTagPred[primeBank]].ctr != 4 ) || 
					(tagPred[primeBank][indexTagPred[primeBank]].usefulBits != 0) || 
					(altBetterCount < 8)) {
						// Prime predictor is not weakly taken or weakly not taken, still useful

						primePred = (tagPred[primeBank][indexTagPred[primeBank]].ctr >= TAGPRED_CTR_MAX/2) ? TAKEN : NOT_TAKEN;
						// if(tagPred[primeBank][indexTagPred[primeBank]].ctr >= TAGPRED_CTR_MAX/2)
						// 	primePred = TAKEN;
						// else 
						// 	primePred = NOT_TAKEN;
						u.direction_prediction(primePred);
				}
				else {
					altPred = basePrediction;
					u.direction_prediction(altPred);
				}
			} else {
				// Prime predictor tag not found, use base predictor 

				altPred = basePrediction;
				u.direction_prediction(altPred);
			}
		} else {
			u.direction_prediction (true);
		}
		u.target_prediction (0);
		return &u;
	}

	// void UpdatePredictor(UINT32 PC (bi), 
	// bool resolveDir (taken), 
	// bool predDir (u.direction_prediction ()) );
	void update (branch_update *u, bool taken, unsigned int target) {
		if (bi.br_flags & BR_CONDITIONAL) {
			bool strong_old_present = false;
			bool new_entry = 0;

			if (primeBank < NUMTAGTABLES) {
				// had found a prime predictor

				// 1st update the useful counter
				if (u->direction_prediction () != altPred) {
					// Prime predictor and alt prediction were different

					if (u->direction_prediction () == taken) { // Correct prediction
						tagPred[primeBank][indexTagPred[primeBank]].usefulBits = satIncrement(tagPred[primeBank][indexTagPred[primeBank]].usefulBits, static_cast<UINT32>(BIMODAL_CTR_MAX));
					} else { // Misprediction
						tagPred[primeBank][indexTagPred[primeBank]].usefulBits = satDecrement(tagPred[primeBank][indexTagPred[primeBank]].usefulBits);
					}
				}

				// 2nd update the counters which provided the prediction  
				if (taken) {
					tagPred[primeBank][indexTagPred[primeBank]].ctr = satIncrement(tagPred[primeBank][indexTagPred[primeBank]].ctr, static_cast<UINT32>(TAGPRED_CTR_MAX));
				} else {
					tagPred[primeBank][indexTagPred[primeBank]].ctr = satDecrement(tagPred[primeBank][indexTagPred[primeBank]].ctr);
				}
			} else {
				// used base prediction as the prime predictor
				UINT32 bimodalIndex = bi.address % numBimodalEntries;
				if (taken)
					bimodal[bimodalIndex] = satIncrement(bimodal[bimodalIndex], static_cast<UINT32>(BIMODAL_CTR_MAX));
				else
					bimodal[bimodalIndex] = satDecrement(bimodal[bimodalIndex]);
			}

			// Check if the current entry that gave the prediction is a newly allocated entry
			if (primeBank < NUMTAGTABLES) {
				// Had found a prime predictor

				if ((tagPred[primeBank][indexTagPred[primeBank]].usefulBits == 0) && 
					((tagPred[primeBank][indexTagPred[primeBank]].ctr == 3) || 
					 (tagPred[primeBank][indexTagPred[primeBank]].ctr == 4))) {
						
					// prime predictor was not useful, and was weakly taken / weakly not taken						
					new_entry = true;
					
					if (primePred != altPred) {
						
						// Alternate prediction more useful is a counter to be of 4 bits
						if (altPred == taken && altBetterCount < 15)		
							altBetterCount++;
					} else if (altBetterCount > 0) {
						altBetterCount--;
					}
				}
			}

			// Allocation of the entry
			if((!new_entry) || (new_entry && (primePred != taken))) {    
				if (((u->direction_prediction () != taken) & (primeBank > 0))) {		
					for (int i = 0; i < primeBank; i++) {
						// Found at least one entry that is not useful

						// TODO: might need to follow the original
						if (tagPred[i][indexTagPred[i]].usefulBits == 0)
							strong_old_present = true;
					}
			
					if (strong_old_present == false) {
						// If no entry useful than decrease useful bits of all entries 
						// do not allocate
						for (int i = primeBank - 1; i >= 0; i--)
							tagPred[i][indexTagPred[i]].usefulBits--;
					} else {
						srand(time(NULL));
						int randNo = rand() % 100;
						int count = 0;
						int bank_store[NUMTAGTABLES - 1] = {-1, -1, -1};
						int matchBank = 0;

						// Count the number of tables that are useless?
						for (int i = 0; i < primeBank; i++) {
							if (tagPred[i][indexTagPred[i]].usefulBits == 0) {
								count++;
								bank_store[i] = i;
							}
						} 

						// only one useless bank
						if(count == 1)
							matchBank = bank_store[0];
						else if(count > 1) {
							// more than one useless bank so we need to choose one randomly with 2/3 preference
							if(randNo > 33 && randNo <= 99)
								matchBank = bank_store[(count-1)];
							else
								matchBank = bank_store[(count-2)];
						}

						// allocate one entry
						// start at the matched bank and go to longer histories
						for (int i = matchBank; i > -1; i--) {
							if ((tagPred[i][indexTagPred[i]].usefulBits == 0)) {
								if(taken)   
									tagPred[i][indexTagPred[i]].ctr = 4;
								else
									tagPred[i][indexTagPred[i]].ctr = 3;
	
								tagPred[i][indexTagPred[i]].tag = tag[i];
								tagPred[i][indexTagPred[i]].usefulBits = 0;
								break;
							}
						}
					}
				}
    		}  

			// Periodic Useful bit Reset Logic (Important so as to optimize compared to PPM paper)
			clock++;
        
			// For every 256 K instruction 1st MSB than LSB
			if (clock == (256*1024)) {
            	// reset clock
            	clock = 0;
            	
				clock_flip = (clock_flip == 1) ? 0 : 1;
				// if (clock_flip == 1)
                // 	clock_flip = 0;
				// else
				// 	clock_flip = 1;

	    		if (clock_flip == 1) {// MSB turn
					for (int jj = 0; jj < NUMTAGTABLES; jj++){    
						for (UINT32 ii = 0; ii < numTagPredEntries; ii++)
							tagPred[jj][ii].usefulBits = tagPred[jj][ii].usefulBits & 1;
					}
            	} else {// LSB turn
					for (int jj = 0; jj < NUMTAGTABLES; jj++) {    
						for (UINT32 ii = 0; ii < numTagPredEntries; ii++)
							tagPred[jj][ii].usefulBits = tagPred[jj][ii].usefulBits & 2;
					}
				}
			}
	
			// update the GHR
			GHR = (GHR << 1);

			if (taken)
				GHR.set(0, 1); 

			for (int i = 0; i < NUMTAGTABLES; i++) {
				indexComp[i].updateCompHist(GHR);
				tagComp[0][i].updateCompHist(GHR);
				tagComp[1][i].updateCompHist(GHR);
			}
  
  			// PHR update is simple, jus take the last bit
    		// Always Limited to 16 bits as per paper.
			PHR = (PHR << 1);

			if (bi.address & 1)
				PHR = PHR + 1;
			
			PHR = (PHR & ((1 << 16) - 1));  
		}
	}
};