//Predictor 2- TAGE

#include <bitset>
#include <iostream>
#include <algorithm>
#include <iostream>
#include <iomanip> // For formatting

#define TAKEN 0
#define NOT_TAKEN 1

#define NUM_COMPONENTS 4 // Number of TAGE components
#define BASE_BITS 15 // Base predictor bits
#define TAG_BITS 10 // Number of bits for tags
#define PRED_BITS 3 // Predictor counter bits
#define U_BITS 2 // Utility counter bits

// PARAMETER 1
unsigned int tage_hist_lengths[NUM_COMPONENTS] = {1, 2, 4, 8};

// Print function for branch_info
void print_branch_info(const branch_info &b) {
    std::cout << "Address: 0x" << std::bitset<32>(b.address) << std::dec << std::endl;
    std::cout << "Opcode: " << b.opcode << std::endl << std::endl;
}

struct tageEntry {
	unsigned int pred;	// 3b saturating counter
	unsigned int tag;	// Tag for associativity
	unsigned int u;		// 2b usefulness counter

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
	#define HISTORY_LENGTH	15
	#define TABLE_BITS	15

	my_update u;
    branch_info bi;
    unsigned int history; 							// Global history register
    unsigned char base_predictor[1 << BASE_BITS];	// Base predictor
    tageEntry tage[NUM_COMPONENTS][1 << TABLE_BITS]; // TAGE components

    my_predictor() : history(0) {
        memset(base_predictor, 0, sizeof(base_predictor));
        for (int i = 0; i < NUM_COMPONENTS; i++) {
            memset(tage[i], 0, sizeof(tage[i]));
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