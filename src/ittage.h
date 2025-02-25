// Predictor 3: ITTAGE

#include <cstdint>
#include <bitset>
#include <algorithm>
#include "tools.h"

#define BIMODAL_LOG_SIZE   	14	// 2^14 entries in base predictor

#define NUM_ITTAGE_TABLES 	    4	// Total number of ITTAGE components (tables)
#define ITTAGE_COMP_LOG_SIZE	12	// 2^12 entries in  table
#define U_CTR_MAX			    3	// 2bit counter (as per paper); 00 ... 11;
#define C_CTR_MAX			    3	// 2bit counter (as per paper); 00 ... 11;

// Entry in an ITTAGE component
struct IttageEntry {
    unsigned int target;  // Prediction target address
    UINT32 tag;           // Unique tag
    INT32 c;              // 2bit confidence counter
    INT32 u;              // 2bit useful counter
};

class ittage_predictor : public branch_predictor {
private:
	// Histories
	std::bitset<GHIST_SIZE> GHR;    // Global history register
	int PHR;				        // 16bit path history
	
	// Bimodal Base Predictor
	unsigned int *bimodal;		// Pattern history table (pht)
	UINT32 numBimodalEntries;	// Total entries in pht 
	
	// Tagged Predictors
	IttageEntry *ittagePred[NUM_ITTAGE_TABLES];	// ITTAGE tables; T[4]
	// UINT32 geometric[NUM_ITTAGE_TABLES];		// Geometric history length of T[i]
	UINT32 numTagPredEntries;				    // Total entries in TAGE table
	UINT32 index[NUM_ITTAGE_TABLES];		    // Calculated index for T[i]
	UINT32 tag[NUM_ITTAGE_TABLES];			    // Calculated tag for that index in T[i]
	
	// Compressed Buffers
	FoldedHist indexComp[NUM_ITTAGE_TABLES];
	FoldedHist tagComp[2][NUM_ITTAGE_TABLES]; 

	// Predictions
	unsigned int providerPred;  // Prediction of the provider component
	unsigned int altPred;		// Prediction of the alternate component
	int providerComp;		    // Provider component
	int altComp;			    // Alternate component
	INT32 altBetterCount;	    // Times that the alternate prediction was better

	// Clock for resetting
	UINT32 clock;
	int clock_flip;

public:
	branch_update u;
	branch_info bi;

	ittage_predictor (void) { 

        // Initialize bimodal predictor
        numBimodalEntries = (1 << BIMODAL_LOG_SIZE);
        bimodal = new unsigned int[numBimodalEntries];
    
        for(UINT32 i = 0; i < numBimodalEntries; i++)
            bimodal[i] = 0;
        
        // Initialize tagged predictors 
        numTagPredEntries = (1 << ITTAGE_COMP_LOG_SIZE);
    
        for(UINT32 i = 0; i < NUM_ITTAGE_TABLES; i++) {
            ittagePred[i] = new IttageEntry[numTagPredEntries];
    
            for(UINT32 j = 0; j < numTagPredEntries; j++) {
                ittagePred[i][j].target = 0; 
                ittagePred[i][j].tag = 0;     
                ittagePred[i][j].u = 0;
                ittagePred[i][j].c = 0;
            }
        }
    
        // Initialize stored indices and tags
        for(int i = 0; i < NUM_ITTAGE_TABLES; i++) {
            index[i] = 0;
            tag[i] = 0;
        }

        // 130, 44, 15, 5
        // Geometric lengths of history, T0 is longest
        UINT32 geometric[NUM_ITTAGE_TABLES] = { 128, 32, 8, 2 };
    
        // Initialize compressed buffers for indices 
        for(int i = 0; i < NUM_ITTAGE_TABLES; i++) {
            indexComp[i].geomLength = geometric[i];
            indexComp[i].targetLength = ITTAGE_COMP_LOG_SIZE;
            indexComp[i].compHist = 0;
        }
    
        // Initialize compressed buffers for tags
        // From PPM paper, tagComp[0] has 8bits and tagComp[1] has 7 bits
        for(int j = 0; j < 2 ; j++) {
            for(int i = 0; i < NUM_ITTAGE_TABLES; i++) {
                tagComp[j][i].geomLength = geometric[i];
                tagComp[j][i].targetLength = (j == 0) ? 8 : 7;
                tagComp[j][i].compHist = 0;
            }   
        }
    
        // Predictions banks and values 
        providerPred = 0;
        altPred = 0;
        providerComp = NUM_ITTAGE_TABLES;
        altComp = NUM_ITTAGE_TABLES;
            
        clock = 0;
        clock_flip = 1;
        PHR = 0;
        GHR.reset();
        altBetterCount = 8;
    }    

	branch_update *predict (branch_info & b) {
        bi = b;

        // Base prediction
        UINT32 bimodalIndex = b.address % numBimodalEntries;
        unsigned int baseTarget = bimodal[bimodalIndex];

        // Compute tag according to PPM paper: pc[9:0] ⊕ CSR1 ⊕ (CSR2 << 1)
        for (int i = 0; i < NUM_ITTAGE_TABLES; i++) {
            tag[i] = b.address ^ tagComp[0][i].compHist ^ (tagComp[1][i].compHist << 1);
            tag[i] &= ((1 << 9) - 1);
        }

        // Compute index for each table according to PPM paper: pc[9:0] ⊕ pc[19:10] ⊕ ghist ⊕ phist
        index[0] = b.address ^ (b.address >> ITTAGE_COMP_LOG_SIZE) ^ indexComp[0].compHist ^ PHR ^ (PHR >> ITTAGE_COMP_LOG_SIZE);
        index[1] = b.address ^ (b.address >> (ITTAGE_COMP_LOG_SIZE - 1)) ^ indexComp[1].compHist ^ (PHR);
        index[2] = b.address ^ (b.address >> (ITTAGE_COMP_LOG_SIZE - 2)) ^ indexComp[2].compHist ^ (PHR & 31);
        index[3] = b.address ^ (b.address >> (ITTAGE_COMP_LOG_SIZE - 3)) ^ indexComp[3].compHist ^ (PHR & 7);

        UINT32 index_mask = ((1 << ITTAGE_COMP_LOG_SIZE) - 1);
        for (int i = 0; i < NUM_ITTAGE_TABLES; i++)
            index[i] &= index_mask;

        // Set the provider and alternate predictions
        providerPred = -1;
        altPred = -1;
        providerComp = NUM_ITTAGE_TABLES;
        altComp = NUM_ITTAGE_TABLES;
        
        // See if any tags match for the provider component; T0 would be best
        for (int i = 0; i < NUM_ITTAGE_TABLES; i++) {
            if (ittagePred[i][index[i]].tag == tag[i]) {
                providerComp = i;
                break;
            }
        }

        // See if any tags match for alternate predictor
        for (int i = providerComp + 1; i < NUM_ITTAGE_TABLES; i++) {
            if (ittagePred[i][index[i]].tag == tag[i]) {
                altComp = i;
                break;
            }
        }

        // Determine final prediction using confidence
        if (providerComp < NUM_ITTAGE_TABLES) { // Provider component found
            
            if (altComp == NUM_ITTAGE_TABLES)
                altPred = baseTarget; // Alt pred not found; use base predictor
            else
                altPred = ittagePred[altComp][index[altComp]].target;

            
            INT32 confidence = ittagePred[providerComp][index[providerComp]].c;

            if (confidence > 1 || altBetterCount <= ALT_BETTER_COUNT_MAX/2) {
                providerPred = ittagePred[providerComp][index[providerComp]].target;
                u.target_prediction(providerPred);
            }
            else
                u.target_prediction(altPred);
        } else  // Provider component not found
            u.target_prediction(baseTarget);
    
        return &u;
    }

	void update (branch_update *u, bool taken, unsigned int target) {
        bool useless_entries_found = false;
        
        // First, update the provider component's useful bit and target prediction
        if (providerComp < NUM_ITTAGE_TABLES) {

            if (u->target_prediction () != altPred) {
                if (u->target_prediction () == target)
                    ittagePred[providerComp][index[providerComp]].u = satIncrement(ittagePred[providerComp][index[providerComp]].u, static_cast<UINT32>(U_CTR_MAX));
                else
                    ittagePred[providerComp][index[providerComp]].u = satDecrement(ittagePred[providerComp][index[providerComp]].u);
            }

            if (u->target_prediction() != target) {
                satDecrement(ittagePred[providerComp][index[providerComp]].c);

                if (ittagePred[providerComp][index[providerComp]].c == 0)
                    ittagePred[providerComp][index[providerComp]].target = target;    
            } else
                satIncrement(ittagePred[providerComp][index[providerComp]].c, C_CTR_MAX);
        } else {    // Update base predictor's target
            UINT32 bimodalIndex = bi.address % numBimodalEntries;
            bimodal[bimodalIndex] = target;
        }

        // Was the alternate prediction more useful?
        if (providerComp < NUM_ITTAGE_TABLES && ittagePred[providerComp][index[providerComp]].u == 0) {					
            if (providerPred != altPred) {
                if (altPred == target && altBetterCount < ALT_BETTER_COUNT_MAX)		
                    altBetterCount++;
            } else if (altBetterCount > 0)
                altBetterCount--;
        }

        // Allocate new entry if there was a misprediction 
        if (u->target_prediction() != target) {

            // Look for an unused entry in smaller history tables
            if (providerComp > 0) {
                for (int i = 0; i < providerComp; i++) {
                    if (ittagePred[i][index[i]].u == 0) {
                        useless_entries_found = true;
                        break;
                    }
                }

                if (!useless_entries_found) {
                    // All entries are useful; decrease useful bits for all and do not allocate
                    for (int i = providerComp - 1; i >= 0; i--)
                        ittagePred[i][index[i]].u = satDecrement(ittagePred[i][index[i]].u);
                } else {
                    srand(time(NULL));
                    int randNo = rand() % 100;
                    int count = 0;
                    int bank_store[NUM_ITTAGE_TABLES - 1] = {-1, -1, -1};
                    int matchBank = 0;

                    // Count the number of components with a useless entry
                    for (int i = 0; i < providerComp; i++) {
                        if (ittagePred[i][index[i]].u == 0) {
                            count++;
                            bank_store[i] = i;
                        }
                    } 

                    if (count == 1)
                        matchBank = bank_store[0];
                    else if (count > 1) {
                        // More than one useless bank; choose one randomly with 2/3 preference for the component with longer history
                        if (randNo > 33 && randNo <= 99)
                            matchBank = bank_store[(count-1)];
                        else
                            matchBank = bank_store[(count-2)];
                    }

                    // Allocate an entry in the chosen bank
                    for (int i = matchBank; i >= 0; i--) {
                        if (ittagePred[i][index[i]].u == 0) {
                            ittagePred[i][index[i]].target = target;
                            ittagePred[i][index[i]].tag = tag[i];
                            ittagePred[i][index[i]].c = 1;
                            ittagePred[i][index[i]].u = 0;
                            break;
                        }
                    }
                }
            }
        }

        // Periodic useful bit reset
        clock++;

        if (clock == CLOCK_RESET_PERIOD) {
            clock = 0;
            clock_flip = !clock_flip;

            if (clock_flip) { // Reset MSB
                for (int j = 0; j < NUM_ITTAGE_TABLES; j++) {
                    for (UINT32 i = 0; i < numTagPredEntries; i++)
                        ittagePred[j][i].u &= 1;
                }
            } else { // Reset LSB
                for (int j = 0; j < NUM_ITTAGE_TABLES; j++) {
                    for (UINT32 i = 0; i < numTagPredEntries; i++)
                        ittagePred[j][i].u &= 2;
                }
            }
        }

        // Append branch target to GHR
        GHR = (GHR << 1);
        GHR.set(0, (target & 1));

        for (int i = 0; i < NUM_ITTAGE_TABLES; i++) {
            indexComp[i].updateCompHist(GHR);
            tagComp[0][i].updateCompHist(GHR);
            tagComp[1][i].updateCompHist(GHR);
        }

        // Append the LSB of the address to the PHR
        PHR = (PHR << 1);
        if (bi.address & 1)
            PHR += 1;
        PHR &= ((1 << 16) - 1);
    }
};