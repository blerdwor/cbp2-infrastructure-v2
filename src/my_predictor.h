#include <cstdint>
#include <bitset>
#include <algorithm>

#define INT32	int32_t
#define UINT32	uint32_t

#define TAKEN		true
#define NOT_TAKEN	false

#define NUM_TAGE_TABLES 	4	// Total number of TAGE components (tables)
#define BIMODAL_CTR_MAX		3	// 2bit counter (following paper); 00 ... 11;  
#define BIMODAL_CTR_INIT	2	// TODO: Check paper
#define BIMODAL_LOG_SIZE   	14	// 2^14 entries in base predictor
#define TAGEPRED_CTR_MAX	7	// 3bit counter (following paper); 000 ... 111
#define TAGEPRED_CTR_INIT	0	// TODO: Check paper
#define TAGEPRED_LOG_SIZE	12	// 2^12 entries in tage table

#define ALT_BETTER_COUNT_MAX	15 // 4bit counter for the max number of times that the alternate predictor was better

// Entry in a TAGE compoenent
struct TagEntry {
    INT32 ctr;	// 3bit predictor
    UINT32 tag;	// Unique tag
    INT32 u;	// 2bit useful counter
};

// Folded history implementation; GHR(geometric length) -> Compressed(target)
struct CompressedHist {
    UINT32 geomLength;		// Geometric history length
    UINT32 targetLength;	// Cropped size
    UINT32 compHist;		// Compressed history
      
    void updateCompHist(std::bitset<131> ghr) {
        int mask = (1 << targetLength) - 1;
        int mask1 = ghr[geomLength] << (geomLength % targetLength);
        int mask2 = (1 << targetLength);
		compHist  = (compHist << 1) + ghr[0];
		compHist ^= ((compHist & mask2) >> targetLength);
		compHist ^= mask1;
		compHist &= mask;     
    }    
};

int satIncrement(UINT32 value, UINT32 max) { return (value < max) ? value + 1 : value; }

int satDecrement(UINT32 value) { return (value > 0) ? value - 1 : value; }

class my_predictor : public branch_predictor {
private:
	// Histories
	std::bitset<131> GHR;	// Global history register
	int PHR;				// 16bit path history
	
	// Bimodal Base Predictor
	UINT32  *bimodal;			// Pattern history table (pht)
	UINT32  numBimodalEntries;	// Total entries in pht 
	
	//Tagged Predictors
	TagEntry *tagePred[NUM_TAGE_TABLES];	// TAGE tables; T[4]
	UINT32 geometric[NUM_TAGE_TABLES];		// Geometric history length of T[i]
	UINT32 numTagPredEntries;				// Total entries in TAGE table
	UINT32 tageIndex[NUM_TAGE_TABLES];		// Calculated index for T[i]
	UINT32 tag[NUM_TAGE_TABLES];			// Calculated tag for that index in T[i]
	
	//Compressed Buffers
	CompressedHist indexComp[NUM_TAGE_TABLES];
	CompressedHist tagComp[2][NUM_TAGE_TABLES]; 

	// Predictions
	bool providerPred;		// Prediction of the provider component
	bool altPred;			// Prediction of the alternate component
	int providerComp;		// Provider component
	int altComp;			// Alternate component
	INT32 altBetterCount;	// Times that the alternate prediction was better

	// Clock for resetting
	UINT32 clock;
	int clock_flip;

public:
	branch_update u;
	branch_info bi;

	my_predictor (void) { 

		// Initialize bimodal predictors
		numBimodalEntries = (1 << BIMODAL_LOG_SIZE);
		bimodal = new UINT32[numBimodalEntries];

		for(UINT32 i = 0; i < numBimodalEntries; i++)
			bimodal[i] = BIMODAL_CTR_INIT;
		
		// Initialize tagged predictors 
		numTagPredEntries = (1 << TAGEPRED_LOG_SIZE);

		for(UINT32 i = 0; i < NUM_TAGE_TABLES; i++) {
			tagePred[i] = new TagEntry[numTagPredEntries];

			for(UINT32 j = 0; j < numTagPredEntries; j++) {
				tagePred[i][j].ctr = TAGEPRED_CTR_INIT;
				tagePred[i][j].tag = 0;
				tagePred[i][j].u = 0;
			}
		}

		for(int i=0; i < NUM_TAGE_TABLES; i++) {
			tageIndex[i] = 0;
			tag[i] = 0;
		}

		// Geometric lengths of history, T0 is longest
		geometric[0] = 130;
		geometric[1] = 44;
		geometric[2] = 15;
		geometric[3] = 5;

		// Initialize compressed buffers for tage components 
		for(int i = 0; i < NUM_TAGE_TABLES; i++) {
			indexComp[i].geomLength = geometric[i];
			indexComp[i].targetLength = TAGEPRED_LOG_SIZE;
			indexComp[i].compHist = 0;
		}

		// Initialize compressed buffers for tags
        // T0/T1 have tag length of 9; T2/T3 have tag length of 8
        for(int j = 0; j < 2 ; j++) {
        	for(int i = 0; i < NUM_TAGE_TABLES; i++) {
				tagComp[j][i].geomLength = geometric[i];
				tagComp[j][i].targetLength = (j == 0) ? 9 : 8;
				tagComp[j][i].compHist = 0;
        	}   
    	}

		// Predictions banks and values 
		providerPred = -1;
		altPred = -1;
		providerComp = NUM_TAGE_TABLES;
		altComp = NUM_TAGE_TABLES;
			
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
			for (int i = 0; i < NUM_TAGE_TABLES; i++) {
				tag[i] = b.address ^ tagComp[0][i].compHist ^ (tagComp[1][i].compHist << 1);
				tag[i] &= ((1 << 9) - 1);
			}
			
			// Get the index for each table
			tageIndex[0] = b.address ^ (b.address >> TAGEPRED_LOG_SIZE) ^ indexComp[0].compHist ^ PHR ^ (PHR >> TAGEPRED_LOG_SIZE);
       		tageIndex[1] = b.address ^ (b.address >> (TAGEPRED_LOG_SIZE - 1)) ^ indexComp[1].compHist ^ (PHR);
       		tageIndex[2] = b.address ^ (b.address >> (TAGEPRED_LOG_SIZE - 2)) ^ indexComp[2].compHist ^ (PHR & 31);
       		tageIndex[3] = b.address ^ (b.address >> (TAGEPRED_LOG_SIZE - 3)) ^ indexComp[3].compHist ^ (PHR & 7);
			
			UINT32 index_mask = ((1 << TAGEPRED_LOG_SIZE) - 1);
			for(int i = 0; i < NUM_TAGE_TABLES; i++)
            	tageIndex[i] &= index_mask;
			
			// Get the provider and alternate predictions
			providerPred = -1;
			altPred = -1;
			providerComp = NUM_TAGE_TABLES;
			altComp = NUM_TAGE_TABLES;

			// See if the tags match for the provider component; T0 would be best
			for(int i = 0; i < NUM_TAGE_TABLES; i++) {
            	if(tagePred[i][tageIndex[i]].tag == tag[i]) {
					providerComp = i;
					break;
				}  
       		}      
            
			// See if the tags match for alternate predictor
			for(int i = providerComp + 1; i < NUM_TAGE_TABLES; i++) {
                if (tagePred[i][tageIndex[i]].tag == tag[i]) {
                    altComp = i;
                    break;
                }  
            }
			
			if (providerComp < NUM_TAGE_TABLES) {	/* Provider component found */
				if(altComp == NUM_TAGE_TABLES)
					altPred = basePrediction;	// Alternate component not found; use base predictor
				else
					altPred = (tagePred[altComp][tageIndex[altComp]].ctr >= TAGEPRED_CTR_MAX/2) ? TAKEN : NOT_TAKEN;	// Alt component found
        
				// Use provider prediction
				if ((tagePred[providerComp][tageIndex[providerComp]].ctr != 3) || 
					(tagePred[providerComp][tageIndex[providerComp]].ctr != 4 ) || 
					(tagePred[providerComp][tageIndex[providerComp]].u != 0) || 
					(altBetterCount <= ALT_BETTER_COUNT_MAX/2)) { 
						providerPred = (tagePred[providerComp][tageIndex[providerComp]].ctr >= TAGEPRED_CTR_MAX/2) ? TAKEN : NOT_TAKEN;
						u.direction_prediction(providerPred);
				}
				else	// Use alternate prediction
					u.direction_prediction(altPred);

			} else {	/* Provider component not found */
				altPred = basePrediction;
				u.direction_prediction(altPred);
			}
		} else
			u.direction_prediction (true);	// Other non-conditional branches

		u.target_prediction (0);
		return &u;
	}

	void update (branch_update *u, bool taken, unsigned int target) {
		if (bi.br_flags & BR_CONDITIONAL) {
			bool strong_old_present = false;
			bool new_entry = 0;

			// First update the counters of the appropriate predictor
			if (providerComp < NUM_TAGE_TABLES) {
				/* Provider component found previously */

				// If the provider prediction != alt prediction, increment useful ctr on a correct prediction, decrement otherwise
				if (u->direction_prediction () != altPred) {
					if (u->direction_prediction () == taken)
						tagePred[providerComp][tageIndex[providerComp]].u = satIncrement(tagePred[providerComp][tageIndex[providerComp]].u, static_cast<UINT32>(BIMODAL_CTR_MAX));
					else
						tagePred[providerComp][tageIndex[providerComp]].u = satDecrement(tagePred[providerComp][tageIndex[providerComp]].u);
				}

				// Then update the provider component ctr  
				if (taken)
					tagePred[providerComp][tageIndex[providerComp]].ctr = satIncrement(tagePred[providerComp][tageIndex[providerComp]].ctr, static_cast<UINT32>(TAGEPRED_CTR_MAX));
				else
					tagePred[providerComp][tageIndex[providerComp]].ctr = satDecrement(tagePred[providerComp][tageIndex[providerComp]].ctr);

			} else {
				/* Provider component not found previously; used base predictor */
				UINT32 bimodalIndex = bi.address % numBimodalEntries;
				if (taken)
					bimodal[bimodalIndex] = satIncrement(bimodal[bimodalIndex], static_cast<UINT32>(BIMODAL_CTR_MAX));
				else
					bimodal[bimodalIndex] = satDecrement(bimodal[bimodalIndex]);
			}

			// Is current entry that gave the prediction a newly allocated entry?
			if (providerComp < NUM_TAGE_TABLES) {
				/* Provider component found previously */

				// Provider predictor was not useful, and was weakly not taken / weakly taken
				if ((tagePred[providerComp][tageIndex[providerComp]].u == 0) && 
					((tagePred[providerComp][tageIndex[providerComp]].ctr == 3) || 
					 (tagePred[providerComp][tageIndex[providerComp]].ctr == 4))) {
												
					new_entry = true;
					
					// Alternate prediction is more useful
					if (providerPred != altPred) {
						if (altPred == taken && altBetterCount < ALT_BETTER_COUNT_MAX)		
							altBetterCount++;
					} else if (altBetterCount > 0)
						altBetterCount--;
				}
			}

			// Allocate a new entry if we haven't already or the provider prediction was wrong
			if((!new_entry) || (new_entry && (providerPred != taken))) {

				// Misprediction; didn't use T0
				if ((u->direction_prediction () != taken) && (providerComp > 0)) {		
					for (int i = 0; i < providerComp; i++) {
						// Find at least one entry that is not useful

						// TODO: might need to follow the original, read paper here
						if (tagePred[i][tageIndex[i]].u == 0)
							strong_old_present = true;
					}
			
					if (strong_old_present == false) {
						// If all entries are useful, than decrease useful bits for all and do not allocate
						for (int i = providerComp - 1; i >= 0; i--)
							tagePred[i][tageIndex[i]].u = satDecrement(tagePred[i][tageIndex[i]].u);
					} else {
						srand(time(NULL));
						int randNo = rand() % 100;
						int count = 0;
						int bank_store[NUM_TAGE_TABLES - 1] = {-1, -1, -1};
						int matchBank = 0;

						// Count the number of components with a useless entry at the calculated index
						for (int i = 0; i < providerComp; i++) {
							if (tagePred[i][tageIndex[i]].u == 0) {
								count++;
								bank_store[i] = i;
							}
						} 

						// Only one useless component
						if(count == 1)
							matchBank = bank_store[0];
						else if(count > 1) {
							// More than one useless bank; choose one randomly with 2/3 preference for the component with longer history
							if(randNo > 33 && randNo <= 99)
								matchBank = bank_store[(count-1)];
							else
								matchBank = bank_store[(count-2)];
						}

						// Allocate one entry; start at the matched component and go to longer histories
						for (int i = matchBank; i > -1; i--) {
							if ((tagePred[i][tageIndex[i]].u == 0)) {
								if(taken)   
									tagePred[i][tageIndex[i]].ctr = 4;
								else
									tagePred[i][tageIndex[i]].ctr = 3;
	
								tagePred[i][tageIndex[i]].tag = tag[i];
								tagePred[i][tageIndex[i]].u = 0;
								break;
							}
						}
					}
				}
    		}  

			// Periodic useful bit reset (important for optimizing over PPM paper)
			clock++;
        
			// Every 256K instruction, clear MSB and then LSB
			if (clock == (256*1024)) {
            	// Reset clock
            	clock = 0;

				clock_flip = (clock_flip == 1) ? 0 : 1;

				if (clock_flip == 1) { // MSB turn
					for (int j = 0; j < NUM_TAGE_TABLES; j++){    
						for (UINT32 i = 0; i < numTagPredEntries; i++)
							tagePred[j][i].u = tagePred[j][i].u & 1;
					}
            	} else { // LSB turn
					for (int j = 0; j < NUM_TAGE_TABLES; j++) {    
						for (UINT32 i = 0; i < numTagPredEntries; i++)
							tagePred[j][i].u = tagePred[j][i].u & 2;
					}
				}
			}
	
			// Update GHR
			GHR = (GHR << 1);
			if (taken)
				GHR.set(0, 1); 

			for (int i = 0; i < NUM_TAGE_TABLES; i++) {
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