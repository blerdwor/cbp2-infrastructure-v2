// Predictor 3: ITTAGE

#include <cstdint>
#include <bitset>
#include <algorithm>

#include <fstream>

std::ofstream outFile("output.txt");

#define INT32	int32_t
#define UINT32	uint32_t

#define TAKEN		true
#define NOT_TAKEN	false

#define NUM_TAGE_TABLES 	4	// Total number of TAGE components (tables)
#define BIMODAL_CTR_MAX		3	// 2bit counter (as per paper); 00 ... 11;  
#define BIMODAL_CTR_INIT	2	// TODO: Check paper
#define BIMODAL_LOG_SIZE   	14	// 2^14 entries in base predictor
#define TAGEPRED_CTR_MAX	7	// 3bit counter (as per paper); 000 ... 111
#define TAGEPRED_CTR_INIT	4	// TODO: Check paper
#define TAGEPRED_LOG_SIZE	12	// 2^12 entries in tage table

#define ALT_BETTER_COUNT_MAX	15 			// 4bit counter for the max number of times that the alternate predictor was better
#define CLOCK_RESET_PERIOD		256*1024	// Useful bit resets after 256K branches (as per paper)

// Entry in an ITTAGE component
struct IttageEntry {
    unsigned int target;  // Full target address for indirect branch prediction
    UINT32 tag;           // Unique tag
    INT32 c;              // 2-bit confidence counter
    INT32 u;              // 2-bit useful counter
};

class ittage_predictor : public branch_predictor {
private:
	// Histories
	std::bitset<131> GHR;	// Global history register
	int PHR;				// 16bit path history
	
	// Bimodal Base Predictor
	unsigned int *bimodal;			// Pattern history table (pht)
	UINT32 numBimodalEntries;	// Total entries in pht 
	
	// Tagged Predictors
	IttageEntry *tagePred[NUM_TAGE_TABLES];	// TAGE tables; T[4]
	UINT32 geometric[NUM_TAGE_TABLES];		// Geometric history length of T[i]
	UINT32 numTagPredEntries;				// Total entries in TAGE table
	UINT32 tageIndex[NUM_TAGE_TABLES];		// Calculated index for T[i]
	UINT32 tag[NUM_TAGE_TABLES];			// Calculated tag for that index in T[i]
	
	// Compressed Buffers
	CompressedHist indexComp[NUM_TAGE_TABLES];
	CompressedHist tagComp[2][NUM_TAGE_TABLES]; 

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

        // Initialize bimodal predictor (now stores targets instead of counters)
        numBimodalEntries = (1 << BIMODAL_LOG_SIZE);
        bimodal = new unsigned int[numBimodalEntries];  // Stores full target addresses
    
        for(UINT32 i = 0; i < numBimodalEntries; i++)
            bimodal[i] = 0;  // No target stored initially
        
        // Initialize tagged predictors 
        numTagPredEntries = (1 << TAGEPRED_LOG_SIZE);
    
        for(UINT32 i = 0; i < NUM_TAGE_TABLES; i++) {
            tagePred[i] = new IttageEntry[numTagPredEntries];
    
            for(UINT32 j = 0; j < numTagPredEntries; j++) {
                tagePred[i][j].target = 0;  // No target stored initially
                tagePred[i][j].tag = 0;     
                tagePred[i][j].u = 0;       // Confidence starts at 0
                tagePred[i][j].c = 0;       // Useful bit starts at 0
            }
        }
    
        for(int i = 0; i < NUM_TAGE_TABLES; i++) {
            tageIndex[i] = 0;
            tag[i] = 0;
        }
    
        // Geometric lengths of history, T0 is longest
        geometric[0] = 130;
        geometric[1] = 44;
        geometric[2] = 15;
        geometric[3] = 5;
    
        // Initialize compressed buffers for TAGE components 
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
        providerPred = 0;
        altPred = 0;
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
        if (b.br_flags & BR_INDIRECT) {
            // Base prediction: fetch the target from the bimodal table
            UINT32 bimodalIndex = b.address % numBimodalEntries;
            unsigned int baseTarget = bimodal[bimodalIndex];
    
            // Hashing to generate tag values for each table
            for (int i = 0; i < NUM_TAGE_TABLES; i++) {
                tag[i] = b.address ^ tagComp[0][i].compHist ^ (tagComp[1][i].compHist << 1);
                tag[i] &= ((1 << 9) - 1);
            }
    
            // Compute indices for each TAGE table
            tageIndex[0] = b.address ^ (b.address >> TAGEPRED_LOG_SIZE) ^ indexComp[0].compHist ^ PHR ^ (PHR >> TAGEPRED_LOG_SIZE);
            tageIndex[1] = b.address ^ (b.address >> (TAGEPRED_LOG_SIZE - 1)) ^ indexComp[1].compHist ^ (PHR);
            tageIndex[2] = b.address ^ (b.address >> (TAGEPRED_LOG_SIZE - 2)) ^ indexComp[2].compHist ^ (PHR & 31);
            tageIndex[3] = b.address ^ (b.address >> (TAGEPRED_LOG_SIZE - 3)) ^ indexComp[3].compHist ^ (PHR & 7);
    
            UINT32 index_mask = ((1 << TAGEPRED_LOG_SIZE) - 1);
            for (int i = 0; i < NUM_TAGE_TABLES; i++)
                tageIndex[i] &= index_mask;
    
            // Search for provider and alternate predictor
            providerComp = NUM_TAGE_TABLES;
            altComp = NUM_TAGE_TABLES;
            for (int i = 0; i < NUM_TAGE_TABLES; i++) {
                if (tagePred[i][tageIndex[i]].tag == tag[i]) {
                    outFile << "HIT\n";
                    providerComp = i;
                    break;
                }
            }
            for (int i = providerComp + 1; i < NUM_TAGE_TABLES; i++) {
                if (tagePred[i][tageIndex[i]].tag == tag[i]) {
                    outFile << "ALT HIT\n";
                    altComp = i;
                    break;
                }
            }
    
            // Get the alternate target prediction
            if (altComp == NUM_TAGE_TABLES)
                altPred = baseTarget; // No alternate component found
            else
                altPred = tagePred[altComp][tageIndex[altComp]].target;
    
            // Determine final prediction using confidence
            if (providerComp < NUM_TAGE_TABLES) { // Found provider component
                providerPred = tagePred[providerComp][tageIndex[providerComp]].target;
                INT32 confidence = tagePred[providerComp][tageIndex[providerComp]].c;
    
                if (confidence > 1 || altBetterCount < 8)
                    u.target_prediction(providerPred); // Use provider target
                else
                    u.target_prediction(altPred); // Use alternate target
            } else // No provider component found
                u.target_prediction(baseTarget); // Default to bimodal target
        } else
            u.target_prediction(0); // Default for non-conditional branches
    
        return &u;
    }

	void update (branch_update *u, bool taken, unsigned int target) {
        if (bi.br_flags & BR_INDIRECT) {

            bool new_entry = false;
            
            // First, check if the provider table correctly predicted the target
            if (providerComp < NUM_TAGE_TABLES) {
                // If prediction was incorrect, update the stored target
                if (u->target_prediction() != target) {
                    outFile << "Incorrect\n";
                    tagePred[providerComp][tageIndex[providerComp]].target = target;
                    satDecrement(tagePred[providerComp][tageIndex[providerComp]].c);
                } else {
                    satIncrement(tagePred[providerComp][tageIndex[providerComp]].c, BIMODAL_CTR_MAX);
                    outFile << "Correct\n";
                }
            } else {
                // Provider component not found; update base predictor (bimodal table)
                outFile << "Bimodal correct\n";
                UINT32 bimodalIndex = bi.address % numBimodalEntries;
                bimodal[bimodalIndex] = target;
            }
    
            // Allocate new entry if:
            // There was no provider entry OR the provider prediction was incorrect
            if (providerComp == NUM_TAGE_TABLES || u->target_prediction() != target)
                new_entry = true;
    
            if (new_entry) {
                bool strong_old_present = false;
                
                // Look for an unused entry in smaller history tables
                if (providerComp > 0) {
                    for (int i = 0; i < providerComp; i++) {
                        if (tagePred[i][tageIndex[i]].u == 0)
                            strong_old_present = true;
                    }
    
                    if (!strong_old_present) {
                        // All entries are useful; decrement usefulness counters to make room
                        for (int i = providerComp - 1; i >= 0; i--)
                            tagePred[i][tageIndex[i]].u = satDecrement(tagePred[i][tageIndex[i]].u);
                    } else {
                        // Choose a random bank to allocate from among the useless entries
                        srand(time(NULL));
                        int randNo = rand() % 100;
                        int count = 0;
                        int bank_store[NUM_TAGE_TABLES - 1] = {-1, -1, -1};
                        int matchBank = 0;
    
                        for (int i = 0; i < providerComp; i++) {
                            if (tagePred[i][tageIndex[i]].u == 0)
                                bank_store[count++] = i;
                        }
    
                        if (count == 1)
                            matchBank = bank_store[0];
                        else if (count > 1) {
                            matchBank = (randNo > 33) ? bank_store[count - 1] : bank_store[count - 2];
                        }
    
                        // Allocate an entry in the chosen bank
                        for (int i = matchBank; i >= 0; i--) {
                            if (tagePred[i][tageIndex[i]].u == 0) {
                                tagePred[i][tageIndex[i]].target = target; // Store the new target
                                tagePred[i][tageIndex[i]].tag = tag[i];    // Store the new tag
                                tagePred[i][tageIndex[i]].u = 0;
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
                    for (int j = 0; j < NUM_TAGE_TABLES; j++) {
                        for (UINT32 i = 0; i < numTagPredEntries; i++)
                            tagePred[j][i].u &= 1;
                    }
                } else { // Reset LSB
                    for (int j = 0; j < NUM_TAGE_TABLES; j++) {
                        for (UINT32 i = 0; i < numTagPredEntries; i++)
                            tagePred[j][i].u &= 2;
                    }
                }
            }
    
            // Append branch target to GHR
            GHR = (GHR << 1);
            GHR.set(0, (target & 1)); // Store the least significant bit of the target
    
            for (int i = 0; i < NUM_TAGE_TABLES; i++) {
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
    }
};