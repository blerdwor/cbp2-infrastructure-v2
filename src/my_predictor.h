//Predictor 2- TAGE

#include <bitset>
#include <iostream>
#include <algorithm>
#include <iostream>
#include <iomanip> // For formatting

#define TAKEN		0	// Branch taken
#define NOT_TAKEN	1	// Branch not taken

#define BIMODAL_PRED_SIZE	13	// Number of rows in the bimodel table
#define BIMODAL_PRED_MAX	3	// 2 bit predictor maxsize
#define BIMODAL_PRED_INIT	2	// Weakly taken, 0b10 out of {00, 01, 10, 11}

#define NUM_TAGE_COMPONENTS 4	// Total amount of TAGE components
#define TAGE_PRED_MAX		7	// 3 bit predictor
#define TAGE_U_MAX			3	// 2 bit useful counter

#define WEAKLY_TAKEN		4	// 0b100 out of {000, 001, ..., 110, 111} 
#define WEAKLY_NOT_TAKEN	3	// 0b101 out of {000, 001, ..., 110, 111} 

#define ALTPRED_BETTER_MAX	15
#define ALTPRED_BETTER_INIT	8

#define GLOBAL_HISTORY_LENGTH	100
#define PATH_HISTORY_LENGTH		16

#define CLOCK_MAX	20

const uint32_t HIST_LENGTHS[] = {2, 3, 8, 12, 17, 33, 35, 67, 97, 138, 195, 330, 517, 1193, 1741, 1930};
const uint32_t TAGE_TABLE_SIZE[] = {9,9,10,10,10,10,11,11,11,11,12,12,11,11,10,10};
const uint32_t TAGE_TAG_SIZE[] = {16, 15, 14, 14, 13, 13, 12, 12, 11, 10, 9, 9, 9, 8, 8, 7};

// Print function for branch_info
void print_branch_info(const branch_info &b) {
    std::cout << "Address: 0x" << std::bitset<32>(b.address) << std::dec << std::endl;
    std::cout << "Opcode: " << b.opcode << std::endl << std::endl;
}

struct bimodalEntry{
    unsigned int pred;	// 2 bit saturating counter
};

struct tageEntry {
	unsigned int pred;	// 3 bit saturating counter
	unsigned int tag;	// Tag for associativity
	unsigned int u;		// 2 bit usefulness counter

	void reset() {
		pred = 0;
		tag = 0;
		u = 0;
	}
};

class my_update : public branch_update {
public:
	unsigned int index;
};

class my_predictor : public branch_predictor {
public:
	// #define HISTORY_LENGTH	15
	// #define TABLE_BITS	15

	my_update u;
    branch_info bi;

    unsigned int globalHistory;
	unsigned int pathHistory; 						// Global history register
    
    tageEntry **tagePredictor; // TAGE components
	bimodalEntry *bimodal;                // Bimodal Table

	// Track predictions
	bool providerPred;       
	bool altPred;
	int tableOfPred;
	int altTableOfPred;
	uint32_t indexPred;
	uint32_t altIndexPred;
	int32_t	altBetterCount;
	uint8_t predDir;

    my_predictor() : 
		globalHistory(0), 
		pathHistory(0),
		providerPred(0),       
		altPred(0),
		tableOfPred(NUM_TAGE_COMPONENTS),
		altTableOfPred(NUM_TAGE_COMPONENTS),
		altBetterCount(ALTPRED_BETTER_INIT) {

        // Initialize the default Bimodal Predictor
		uint32_t numBimodalEntries = (1 << BIMODAL_PRED_SIZE);
		bimodal = new bimodalEntry[numBimodalEntries];
		for (uint32_t i = 0; i < numBimodalEntries; i++)
			bimodal[i].pred = BIMODAL_PRED_INIT;

		// Initialize the TAGE predictor
		tagePredictor = new tageEntry*[NUM_TAGE_COMPONENTS];
        for (int i = 0; i < NUM_TAGE_COMPONENTS; i++) {
			uint32_t tableSize = (1 << TAGE_TABLE_SIZE[i]);
			
			tagePredictor[i] = new tageEntry[tableSize];
			for (uint32_t j = 0; j < tableSize; j++)
				tagePredictor[i][j].reset();
        }
    }

    branch_update *predict(branch_info &b) {
		if (b.br_flags & BR_CONDITIONAL) {
			print_branch_info(b);

			u.index = 0;           // Default index
			int hit_table = -1;    // Track the table with a matching tag
			int alt_table = -1;    // Track the alternate table
			unsigned int alt_pred = 0; // Alternate prediction

			// Search TAGE tables
			for (int i = NUM_COMPONENTS - 1; i >= 0; i--) {
				unsigned int index = indexForTable(history, i);
				unsigned int tag = tagForTable(history, i);

				if (tage[i][index].tag == tag) {
					if (hit_table == -1) {
						hit_table = i; // First matching table becomes the primary
					} else if (alt_table == -1) {
						alt_table = i; // Second matching table becomes the alternate
						alt_pred = tage[i][index].pred > 3; // Majority-taken threshold for alt_pred
					}

					// Check if the current table provides a "useful" prediction
					if (tage[i][index].pred != 0 && tage[i][index].pred != ((1 << PRED_BITS) - 1)) {
						break; // Stop searching when we find a useful prediction
					}
				}
			}

			// Determine the final prediction
			if (hit_table == -1) {
				// No matching tag; fall back to the base predictor
				u.direction_prediction(base_predictor[history & ((1 << BASE_BITS) - 1)] > 1);
			} else {
				// Use the primary prediction
				unsigned int primary_pred = tage[hit_table][indexForTable(history, hit_table)].pred > 3;

				// Use alt_pred if the primary prediction is not "strong"
				if (tage[hit_table][indexForTable(history, hit_table)].pred == 0 ||
					tage[hit_table][indexForTable(history, hit_table)].pred == ((1 << PRED_BITS) - 1)) {
					u.direction_prediction(alt_pred); // Alternate prediction
				} else {
					u.direction_prediction(primary_pred); // Primary prediction
				}
			}
		}
		else {
			u.direction_prediction(true);
		}

		u.target_prediction (0);
		return &u;
	}

    void update(branch_update *u, bool taken, unsigned int target) {
    // Update base predictor
    unsigned int base_index = history & ((1 << BASE_BITS) - 1);
    base_predictor[base_index] = std::min(base_predictor[base_index] + taken, 3);

    // Update TAGE predictors
    int hit_table = -1;

    for (int i = NUM_COMPONENTS - 1; i >= 0; i--) {
        unsigned int index = indexForTable(history, i);
        unsigned int tag = tagForTable(history, i);

        if (tage[i][index].tag == tag) {
            hit_table = i;
            if (taken) {
                tage[i][index].pred = std::min(tage[i][index].pred + 1, static_cast<unsigned int>(1 << PRED_BITS) - 1);
            } else {
                tage[i][index].pred = std::max(tage[i][index].pred - 1, static_cast<unsigned int>(0));
            }
            break;
        }
    }

    // Allocate new entries in TAGE if needed
    if (hit_table == -1) {
        for (int i = NUM_COMPONENTS - 1; i >= 0; i--) {
            unsigned int index = indexForTable(history, i);
            if (tage[i][index].u == 0) {
                tage[i][index].tag = tagForTable(history, i);
                tage[i][index].pred = taken ? (1 << (PRED_BITS - 1)) : 0;
                tage[i][index].u = 1;
                break;
            }
        }
    }
	history = ((history << 1) | taken) & ((1 << HISTORY_LENGTH) - 1);

	}

private:
    unsigned int indexForTable(unsigned int history, int table) {
    	return (history ^ (history >> tage_hist_lengths[table])) & ((1 << TABLE_BITS) - 1);
	}

	unsigned int tagForTable(unsigned int history, int table) {
    	return history & ((1 << TAG_BITS) - 1); // Simple example; modify as needed
	}
};