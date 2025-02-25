// Predictor 2: TAGE

#include <cstdint>
#include <bitset>
#include <algorithm>

#define INT32	int32_t
#define UINT32	uint32_t

#define TAKEN		true
#define NOT_TAKEN	false

#define BIMODAL_CTR_MAX		3	// 2bit counter (as per paper); 00 ... 11;  
#define BIMODAL_CTR_INIT	2	// Initialize to weakly taken
#define BIMODAL_LOG_SIZE   	14	// 2^14 entries in base predictor

#define NUM_TAGE_TABLES 	4	// Total number of TAGE components (tables)
#define TAGEPRED_CTR_MAX	7	// 3bit counter (as per paper); 000 ... 111
#define TAGEPRED_CTR_INIT	4	// Initialize to weakly taken
#define TAGEPRED_LOG_SIZE	12	// 2^12 entries in a TAGE component

#define ALT_BETTER_COUNT_MAX	15 			// 4bit counter
#define CLOCK_RESET_PERIOD		256*1024	// Useful bit resets after 256K branches (as per paper)

// Entry in a TAGE component
struct TagEntry {
    INT32 ctr;	// 3bit predictor
    UINT32 tag;	// Unique tag
    INT32 u;	// 2bit useful counter
};

// Folded history compression; GHR(geometric length) -> Compressed(target)
struct FoldedHist {
    UINT32 geomLength;		// Geometric history length
    UINT32 targetLength;	// Cropped size
    UINT32 compHist;		// Compressed history
      
    void updateCompHist(std::bitset<256> ghr) {
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

class tage_predictor : public branch_predictor {
private:
	// Histories
	std::bitset<256> GHR;	// Global history register
	int PHR;				// 16bit path history register
	
	// Bimodal Base Predictor
	UINT32 *bimodal;			// Pattern history table (pht)
	UINT32 numBimodalEntries;	// Total entries in pht 
	
	// Tagged Predictors
	TagEntry *tagePred[NUM_TAGE_TABLES];	// TAGE tables; T[4]
	UINT32 numTagPredEntries;				// Total entries in TAGE table
	UINT32 index[NUM_TAGE_TABLES];			// Calculated index for T[i]
	UINT32 tag[NUM_TAGE_TABLES];			// Calculated tag for that index in T[i]
	
	// Compressed Buffers
	FoldedHist indexComp[NUM_TAGE_TABLES];
	FoldedHist tagComp[2][NUM_TAGE_TABLES]; 

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

	tage_predictor (void) { 

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

		// Initialize stored indices and tags
		for(int i=0; i < NUM_TAGE_TABLES; i++) {
			index[i] = 0;
			tag[i] = 0;
		}

		// { 130, 44, 15, 5 }
		// Geometric lengths of history, T0 is longest
		UINT32 geometric[4] = { 81, 27, 9, 3 };

		// Initialize compressed buffers for indices 
		for(int i = 0; i < NUM_TAGE_TABLES; i++) {
			indexComp[i].geomLength = geometric[i];
			indexComp[i].targetLength = TAGEPRED_LOG_SIZE;
			indexComp[i].compHist = 0;
		}

		// Initialize compressed buffers for tags
        // From PPM paper, tagComp[0] has 8bits and tagComp[1] has 7 bits
        for(int j = 0; j < 2 ; j++) {
        	for(int i = 0; i < NUM_TAGE_TABLES; i++) {
				tagComp[j][i].geomLength = geometric[i];
				tagComp[j][i].targetLength = (j == 0) ? 8 : 7;
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

			// Compute tag according to PPM paper: pc[9:0] ⊕ CSR1 ⊕ (CSR2 << 1)
			for (int i = 0; i < NUM_TAGE_TABLES; i++) {
				tag[i] = b.address ^ tagComp[0][i].compHist ^ (tagComp[1][i].compHist << 1);
				tag[i] &= ((1 << 9) - 1);
			}
			
			// Compute index for each table according to PPM paper: pc[9:0] ⊕ pc[19:10] ⊕ ghist ⊕ phist
			index[0] = b.address ^ (b.address >> TAGEPRED_LOG_SIZE) ^ indexComp[0].compHist ^ PHR ^ (PHR >> TAGEPRED_LOG_SIZE);
       		index[1] = b.address ^ (b.address >> TAGEPRED_LOG_SIZE) ^ indexComp[1].compHist ^ (PHR);
       		index[2] = b.address ^ (b.address >> TAGEPRED_LOG_SIZE) ^ indexComp[2].compHist ^ (PHR & 31);
       		index[3] = b.address ^ (b.address >> TAGEPRED_LOG_SIZE) ^ indexComp[3].compHist ^ (PHR & 7);
			
			UINT32 index_mask = ((1 << TAGEPRED_LOG_SIZE) - 1);
			for(int i = 0; i < NUM_TAGE_TABLES; i++)
            	index[i] &= index_mask;
			
			// Set the provider and alternate predictions
			providerPred = -1;
			altPred = -1;
			providerComp = NUM_TAGE_TABLES;
			altComp = NUM_TAGE_TABLES;

			// See if any tags match for the provider component; T0 would be best
			for(int i = 0; i < NUM_TAGE_TABLES; i++) {
            	if(tagePred[i][index[i]].tag == tag[i]) {
					providerComp = i;
					break;
				}  
       		}      
            
			// See if any tags match for alternate predictor
			for(int i = providerComp + 1; i < NUM_TAGE_TABLES; i++) {
                if (tagePred[i][index[i]].tag == tag[i]) {
                    altComp = i;
                    break;
                }  
            }

			if (providerComp < NUM_TAGE_TABLES) {	// Provider component found

				if(altComp == NUM_TAGE_TABLES)
					altPred = basePrediction;	// Alt pred not found; use base predictor
				else
					altPred = (tagePred[altComp][index[altComp]].ctr >= TAGEPRED_CTR_MAX/2) ? TAKEN : NOT_TAKEN;	// Alt pred found
			
				// Use provider component if it wasn't newly allocated and is useful
				if ((tagePred[providerComp][index[providerComp]].ctr != 3) || 
					(tagePred[providerComp][index[providerComp]].ctr != 4 ) || 
					(tagePred[providerComp][index[providerComp]].u != 0) || 
					(altBetterCount <= ALT_BETTER_COUNT_MAX/2)) { 
						providerPred = (tagePred[providerComp][index[providerComp]].ctr >= TAGEPRED_CTR_MAX/2) ? TAKEN : NOT_TAKEN;
						u.direction_prediction(providerPred);
				} else
					u.direction_prediction(altPred);

			} else {	// Provider component not found
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
			bool useless_entries_found = false;
			bool allocate = false;

			// First, update the provider component's useful bit and prediction counter
			if (providerComp < NUM_TAGE_TABLES) {

				if (u->direction_prediction () != altPred) {
					if (u->direction_prediction () == taken)
						tagePred[providerComp][index[providerComp]].u = satIncrement(tagePred[providerComp][index[providerComp]].u, static_cast<UINT32>(BIMODAL_CTR_MAX));
					else
						tagePred[providerComp][index[providerComp]].u = satDecrement(tagePred[providerComp][index[providerComp]].u);
				}

				if (taken)
					tagePred[providerComp][index[providerComp]].ctr = satIncrement(tagePred[providerComp][index[providerComp]].ctr, static_cast<UINT32>(TAGEPRED_CTR_MAX));
				else
					tagePred[providerComp][index[providerComp]].ctr = satDecrement(tagePred[providerComp][index[providerComp]].ctr);

			} else {	// Update the base predictor's counter
				UINT32 bimodalIndex = bi.address % numBimodalEntries;
				if (taken)
					bimodal[bimodalIndex] = satIncrement(bimodal[bimodalIndex], static_cast<UINT32>(BIMODAL_CTR_MAX));
				else
					bimodal[bimodalIndex] = satDecrement(bimodal[bimodalIndex]);
			}

			// Was the current entry that gave the prediction useful?
			if (providerComp < NUM_TAGE_TABLES) {

				if ((tagePred[providerComp][index[providerComp]].u == 0) && 
					((tagePred[providerComp][index[providerComp]].ctr == 3) || 
					 (tagePred[providerComp][index[providerComp]].ctr == 4))) {
												
					allocate = true;
					
					// Alternate prediction might be more useful
					if (providerPred != altPred) {
						if (altPred == taken && altBetterCount < ALT_BETTER_COUNT_MAX)		
							altBetterCount++;
					} else if (altBetterCount > 0)
						altBetterCount--;
				}
			}

			// Allocate a new entry if necessary or the provider component mispredicted
			if((!allocate) || (allocate && (providerPred != taken))) {

				// Misprediction
				if (u->direction_prediction () != taken && providerComp > 0) {		
					for (int i = 0; i < providerComp; i++) {
						// Find at least one entry that is not useful
						if (tagePred[i][index[i]].u == 0) {
							useless_entries_found = true;
							break;
						}
					}
					
					// All entries useful; decrease useful bits for all and do not allocate
					if (!useless_entries_found) {
						for (int i = providerComp - 1; i >= 0; i--)
							tagePred[i][index[i]].u = satDecrement(tagePred[i][index[i]].u);
					} else {
						srand(time(NULL));
						int randNo = rand() % 100;
						int count = 0;
						int bank_store[NUM_TAGE_TABLES - 1] = {-1, -1, -1};
						int matchBank = 0;

						// Count number of components with a useless entry
						for (int i = 0; i < providerComp; i++) {
							if (tagePred[i][index[i]].u == 0) {
								count++;
								bank_store[i] = i;
							}
						} 

						if(count == 1)
							matchBank = bank_store[0];
						else if (count > 1) {
							// More than one useless bank; choose one randomly with 2/3 preference for component with longer history
							if (randNo > 33 && randNo <= 99)
								matchBank = bank_store[(count-1)];
							else
								matchBank = bank_store[(count-2)];
						}

						// Allocate one entry
						for (int i = matchBank; i > -1; i--) {
							if ((tagePred[i][index[i]].u == 0)) { 
								tagePred[i][index[i]].ctr = taken ? 4 : 3;	
								tagePred[i][index[i]].tag = tag[i];
								tagePred[i][index[i]].u = 0;
								break;
							}
						}
					}
				}
    		}  

			// Periodic useful bit reset (optimizes over PPM paper)
			clock++;
        
			// Every 256K instruction, clear MSB and then LSB
			if (clock == CLOCK_RESET_PERIOD) {
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
	
			// Append the branch result to the GHR
			GHR = (GHR << 1);
			if (taken)
				GHR.set(0, 1); 

			for (int i = 0; i < NUM_TAGE_TABLES; i++) {
				indexComp[i].updateCompHist(GHR);
				tagComp[0][i].updateCompHist(GHR);
				tagComp[1][i].updateCompHist(GHR);
			}
  
  			// Append the LSB of the address to the PHR
			PHR = (PHR << 1);

			if (bi.address & 1)
				PHR = PHR + 1;
			
			PHR = (PHR & ((1 << 16) - 1));  
		}
	}
};