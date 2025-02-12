#include <cstdint>

#define TAKEN		true
#define NOT_TAKEN	false

#define UINT8   uint8_t
#define UINT16  uint16_t
#define UINT32	uint32_t

#define ENTRIES         256 // Number of entries in the predictor table
#define WAY             4   // Associativity of the predictor table
#define LOGIND          6   // Number of bits required to index into the table
#define LOGWAY          2   // Number of bits required to determine the way at an index
#define TAGSIZE         14  // Number of bits to represent tag in the table
#define ITERSIZE        14  // Max size of the loop that the predictor can predict properly
#define AGE             255  // Initial age of the entry
#define CONFIDENCE_MAX  4   // Recognize a branch as a loop after 3 successful executions
#define NO_HIT          -1  // A symbol for not finding a hit

struct LoopEntry {
    UINT16 tag;          // Stores the 14-bit tag for the entry
    UINT16 past_iter;    // Stores the 14-bit count for the number of iterations seen in past
    UINT16 current_iter; // Stores the 14-bit count for the number of iterations seen currently
    UINT8 age;           // 8-bit counter signifying age of entry
    UINT8 confidence;    // 2-bit counter signifying confidence in prediction
};

class loop_predictor {
private:
    LoopEntry table[ENTRIES];   // Predictor table
    int ind;                // Index in loop
    int hit;                // The way in the loop where we get a hit else -1
    int tag;                // The tag calculated
    UINT8 seed;

public:
    branch_update u;
	branch_info bi;
    
    bool is_valid;  // Validity of prediction
    bool loop_pred; // The prediction returned for current PC

    loop_predictor (void) {
        seed = 0;
        for (int i = 0; i < ENTRIES; i++) {
            table[i].tag = 0;
            table[i].past_iter = 0;
            table[i].current_iter = 0;
            table[i].age = 0;
            table[i].confidence = 0;
        }
    }

    branch_update *predict (branch_info & b) {
        hit = NO_HIT;
        ind = (b.address & ((1 << LOGIND) - 1)) << LOGWAY;  // Calculate index
        tag = (b.address >> LOGIND) & ((1 << TAGSIZE) - 1); // Calculate tag
        u.target_prediction (0);

        // Try to find a matching entry
        for (int i = ind; i < ind + WAY; i++) {        
            if (table[i].tag == tag) {
                hit = i;
                is_valid = (table[i].confidence == CONFIDENCE_MAX);  // Only want high confidence
                
                // Loop is on last iteration TODO MIGHT NEED TO CHANGE THIS
                if (table[i].current_iter + 1 == table[i].past_iter) {
                    loop_pred = NOT_TAKEN;
                    u.direction_prediction(NOT_TAKEN);
                } else {
                    loop_pred = TAKEN;
                    u.direction_prediction(TAKEN);
                }
                return &u;
            }
        }

        // No matching entry found in table
        is_valid = false;
        loop_pred = NOT_TAKEN;
        u.direction_prediction(NOT_TAKEN);
        return &u;
    }

    void update (branch_update *u, bool taken, unsigned int target, bool tage_pred);
};

// Updates the predictor table based on the prediction and actually taken/not taken branch
void loop_predictor::update (branch_update *u, bool taken, unsigned int target, bool tage_pred) {
    if (hit > NO_HIT) {
        LoopEntry &entry = table[hit];

        if (is_valid) {
            // If the predicton was wrong, free the entry
            if (taken != loop_pred) {
                entry.current_iter = 0;
                entry.past_iter = 0;
                entry.confidence = 0;
                entry.age = 0;
                return;
            }

            // TODO check why age is only 31
            // If TAGE is wrong and the entry was valid, then age it
            if (taken != tage_pred && entry.age < AGE)
                entry.age++;
        }
        
        entry.current_iter++;
        entry.current_iter &= ((1 << ITERSIZE) - 1);

        // If the iteration is greater than what was seen last time, free the entry
        if (entry.current_iter > entry.past_iter)
        {
            entry.confidence = 0;
            if (entry.past_iter != 0) {
                entry.current_iter = 0;
                entry.past_iter = 0;
                entry.age = 0;
            }
        }

        if (!taken) {
            if (entry.current_iter == entry.past_iter) {
                // Increase the confidence if correct
                if (entry.confidence < 3)
                    entry.confidence++;
                
                // We do not care for loops with < 3 iterations
                if (entry.past_iter > 0 && entry.past_iter < 3) {
                    entry.past_iter = 0;
                    entry.age = 0;
                    entry.confidence = 0;
                }
            } else {
                // Set the newly allocated entry
                if (entry.past_iter == 0) {
                    entry.confidence = 0;
                    entry.past_iter = entry.current_iter;
                } else { // else free the entry
                    entry.past_iter = 0;
                    entry.age = 0;
                    entry.confidence = 0;
                }
            }
            entry.current_iter = 0;
        }
    } else if (taken) {
        // If the branch is taken but there is no entry, we must allocate one entry in the table
        seed = (seed + 1) & 3;

        for (int i = 0; i < WAY; i++) {
            int j = ind + ((seed + i) & 3);

            if (table[j].age == 0) {
                table[j].tag = tag;
                table[j].past_iter = 0;
                table[j].current_iter = 1;
                table[j].age = AGE;
                table[j].confidence = 0;
                break;
            }
            else if (table[j].age > 0)
                table[j].age--;
        }
    }
}