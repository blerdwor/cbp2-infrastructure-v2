//Predictor 3

#include <bitset>
#include <cstdlib>
#include <time.h>
#include <fstream>

#define TAKEN 			  1 	// Branch Taken True
#define NOT_TAKEN 		  0		// Branch Taken False

#define BIMODAL_SIZE      13  	// Number of rows in Bimodal Table

#define BIMODAL_PRED_MAX  3   	// 2 bit prediction Bimodal
#define TAGE_PRED_MAX     7   	// 3 bit prediction TAGE
#define PRED_U_MAX        3   	// 2 useful bit counter

#define BIMODAL_PRED_INIT 2   	// choose 10 out of {00, 01, 10, 11} [Weakly Taken]

#define WEAKLY_TAKEN      4   	// 100 out of {000,...,111} [Weakly Taken for TAGE]
#define WEAKLY_NOT_TAKEN  3		// 011 out of {000,...,111} [Weakly Not Taken for TAGE]

#define NUM_TAGE_TABLES   16  	// Number of TAGE tables

#define ALTPRED_BET_MAX   15    // Alternate Prediction cap of 4 bits
#define ALTPRED_BET_INIT  8     // Init at 1000 out of {0000,...,1111} [Weakly Taken]

#define PHR_LEN           16    // Length of Path History Register

#define CLOCK_MAX         20    // Number of cycles before reset/flush -> 2^20 cycles

// Our parameters tuned to a local optimum
// const uint32_t HIST[] = {2, 3, 8, 12, 17, 33, 35, 67, 97, 138, 195, 330, 517, 1193, 1741, 1930};
const uint32_t HIST[] = {1930, 1741, 1193, 517, 330, 195, 138, 97, 67, 35, 33, 17, 12, 8, 3, 2};
const uint32_t TAGE_TABLE_SIZE[] = {9,9,10,10,10,10,11,11,11,11,12,12,11,11,10,10};
const uint32_t TAGE_TAG_SIZE[] = {16, 15, 14, 14, 13, 13, 12, 12, 11, 10, 9, 9, 9, 8, 8, 7};

//#############################################################################
// For folding overflow
typedef struct CircularShiftRegister { 
    uint16_t val;
    uint16_t oldlen;
    uint16_t newlen;
} CircularShiftRegister_t;
//#############################################################################

//#############################################################################
// 2 prediction bits
typedef struct bimodVal{
    uint32_t pred;  
} bimodVal_t;
//#############################################################################

//#############################################################################
// Entries of TAGE Table
typedef struct tagVal {
    uint16_t pred;
    uint16_t tag;
    uint16_t u;

    void reset();
} tagVal_t;

void tagVal_t::reset() {
    pred = 0;
    tag = 0;
    u = 0;
}
//#############################################################################

//#############################################################################
std::bitset<1001> *GHR;                  // Global History Register       
uint32_t PHR;                       // Path History Register 
//#############################################################################

//#############################################################################
bimodVal_t *bimodal;                // Bimodal Table
uint32_t numBimodalEntries;         // Number of Entries in Bimodal Table
//#############################################################################

//#############################################################################
tagVal_t **tagTables;               // TAGE Tables
uint32_t *tageTableSize;            // Size of each TAGE Table
uint32_t *tageHistory;              // History length of each TAGE Table
//#############################################################################
    
//#############################################################################
CircularShiftRegister_t *csrIndex;  // Circular Shift Register for indexing
CircularShiftRegister_t **csrTag;   // Circular Shift Registers for tags [2 of them]
//#############################################################################

//#~ Variables to Track all Prediction quantities ~############################
bool pred_pred;       
bool altPred_pred;
int table_pred;
int altTable_pred;
uint32_t bimodalIndex;
uint32_t index_pred;
uint32_t altIndex_pred;
//#############################################################################
    
//#############################################################################
uint32_t *tageIndex;                // Indices for all tables
uint32_t *tageTag;                  // Tags for all tables
//#############################################################################

//#############################################################################
uint32_t clockk;                    // Clock to count cycles for reset/flush      
bool clockState;                    // Reset state
//#############################################################################

//#############################################################################
int32_t altBetterCount;             // Number of times Alternate was better
uint8_t predDir;                    // Direction of pred (Taken or Not-taken?)
//#############################################################################

class my_update : public branch_update {
public:
	unsigned int index;
};

class my_predictor : public branch_predictor {
public:
    my_update u;

    my_predictor() {
        GHR = new std::bitset<1001>;
    
        // Tables to stores the entries of the TAGE Tables
        tagTables = new tagVal_t*[NUM_TAGE_TABLES];
        for(uint32_t i = 0; i < NUM_TAGE_TABLES; i++) {
            uint32_t tableSize = (1<<TAGE_TABLE_SIZE[i]);

            tagTables[i] = new tagVal_t[tableSize];
            for(uint32_t j =0; j < tableSize; j++)
                tagTables[i][j].reset();     
        }

        tageIndex = new uint32_t[NUM_TAGE_TABLES];      // Indexing array for TAGE Tables
        for(uint32_t i=0; i < NUM_TAGE_TABLES; i++)   
            tageIndex[i] = 0;
        
        tageTag = new uint32_t[NUM_TAGE_TABLES];        // Tag array for each TAGE Table
        for(uint32_t i=0; i < NUM_TAGE_TABLES; i++)
            tageTag[i] = 0;
        
        // Bimodal Inits 
        numBimodalEntries = (1 << BIMODAL_SIZE);        // Number of entries in the Bimodal Table
        bimodal = new bimodVal_t[numBimodalEntries];    // Entries of Bimodal table
        for(uint32_t i=0; i< numBimodalEntries; i++)
            bimodal[i].pred = BIMODAL_PRED_INIT;        // Init the table to Weakly Taken

        // Create Circular Shift Registers for Index and Tags
        csrIndex = new CircularShiftRegister_t[NUM_TAGE_TABLES];
        csrTag = new CircularShiftRegister_t*[2];
        csrTag[0] = new CircularShiftRegister_t[NUM_TAGE_TABLES];
        csrTag[1] = new CircularShiftRegister_t[NUM_TAGE_TABLES];

        // Init the CSR 
        for(uint32_t i = 0; i<NUM_TAGE_TABLES; i++){
            csrIndex[i].val = 0;
            csrIndex[i].oldlen = HIST[i];
            csrIndex[i].newlen = TAGE_TAG_SIZE[i];

            csrTag[0][i].val = 0;
            csrTag[0][i].oldlen = HIST[i];
            csrTag[0][i].newlen = TAGE_TAG_SIZE[i];

            csrTag[1][i].val = 0;
            csrTag[1][i].oldlen = HIST[i];
            csrTag[1][i].newlen = TAGE_TAG_SIZE[i]-1;
        }
    
        // Init the Global Prediction Counters
        pred_pred = -1;
        altPred_pred = -1;
        table_pred = NUM_TAGE_TABLES;
        altTable_pred = NUM_TAGE_TABLES;
        //~
        
        // Init the clock and clock state
        clockk = 0;
        clockState = 0;
        
        // Init the Path History, Global History and Alternate Better Count (to Weakly Better)
        PHR = 0;
        GHR->reset();
        altBetterCount = ALTPRED_BET_INIT;
	}

    branch_update *predict(branch_info &b) {
		if (b.br_flags & BR_CONDITIONAL) {
			bimodalIndex = (b.address) % (numBimodalEntries); 

            // Get TAGE Tags
            for(int i = 0; i < NUM_TAGE_TABLES; i++)
                tageTag[i] = (b.address ^ csrTag[0][i].val ^ (csrTag[1][i].val << 1)) & ((1 << TAGE_TAG_SIZE[i]) -1);

            // Get TAGE Indices
            for(int i = 0; i < NUM_TAGE_TABLES; i++) 
                tageIndex[i] = (b.address ^ (b.address >> TAGE_TABLE_SIZE[i]) ^ csrIndex[i].val ^ PHR) & ((1 << TAGE_TABLE_SIZE[i])-1);

            // Prepare for TAGE prediction
            pred_pred = -1;
            altPred_pred = -1;
            table_pred = NUM_TAGE_TABLES;
            altTable_pred = NUM_TAGE_TABLES;
            
            // Get the first matching TAGE Table
            for(uint32_t i = 0; i < NUM_TAGE_TABLES; i++) 
                if(tagTables[i][tageIndex[i]].tag == tageTag[i]) {  
                    table_pred = i;
                    index_pred = tageIndex[i];
                    break;
                }  
            
            // Check if any other tables with longer history matches the tag
            for(uint32_t i = table_pred + 1; i < NUM_TAGE_TABLES; i++) {
                if(tagTables[i][tageIndex[i]].tag == tageTag[i]) {  
                    altTable_pred = i;
                    altIndex_pred = tageIndex[i];
                    break;
                }
            }

            if(table_pred < NUM_TAGE_TABLES) {                  // If table was found
                if(altTable_pred == NUM_TAGE_TABLES) {          // No alternate table found
                    altPred_pred = (bimodal[bimodalIndex].pred > BIMODAL_PRED_MAX/2);  
                } 
                else {                                          // Alternate table found
                    if(tagTables[altTable_pred][altIndex_pred].pred >= TAGE_PRED_MAX/2)  
                            altPred_pred = TAKEN;
                        else 
                            altPred_pred = NOT_TAKEN;
                }
                
                if((tagTables[table_pred][index_pred].pred  != WEAKLY_NOT_TAKEN) ||  
                    (tagTables[table_pred][index_pred].pred != WEAKLY_TAKEN) ||     
                    (tagTables[table_pred][index_pred].u != 0) ||                     
                    (altBetterCount < ALTPRED_BET_INIT)) {            // Decide between altPred and Pred                
                    
                    pred_pred = tagTables[table_pred][index_pred].pred >= TAGE_PRED_MAX/2;
                    u.direction_prediction (pred_pred);
                } 
                else {
                    u.direction_prediction (altPred_pred);  
                }
            } 
            else {                                              // If no table found return Bimodal Prediction only
                altPred_pred = (bimodal[bimodalIndex].pred > BIMODAL_PRED_MAX/2);  
                u.direction_prediction (altPred_pred); 
            }
        }

		u.target_prediction (0);
		return &u;
    }

    void update(branch_update *u, bool taken, unsigned int target) {
        // uint32_t bimodalIndex = (u.address) % (numBimodalEntries);  

        // Update Bimodal Table
        int predictionVal = -1;
        int altPredVal = -1;
        if(table_pred < NUM_TAGE_TABLES) {  
            
            predictionVal = tagTables[table_pred][index_pred].pred; 

            if(taken && predictionVal < TAGE_PRED_MAX)          // If taken and not max
                ++(tagTables[table_pred][index_pred].pred);     // Increment Counter
            else if(!taken && predictionVal > 0)                
                --(tagTables[table_pred][index_pred].pred);  
            
            altPredVal = -1;
            if(altTable_pred != NUM_TAGE_TABLES)                // If alternate prediction present
                altPredVal = tagTables[altTable_pred][altIndex_pred].pred;
            
            if(tagTables[table_pred][index_pred].u == 0 && altPredVal != -1) {
                if(taken && altPredVal < TAGE_PRED_MAX)
                    ++(tagTables[altTable_pred][altIndex_pred].pred);
                else if(!taken && altPredVal > 0)
                    --(tagTables[altTable_pred][altIndex_pred].pred);
            } 
        } 
        else {  
            predictionVal = bimodal[bimodalIndex].pred;
            if(taken && predictionVal < BIMODAL_PRED_MAX)   // If taken and not max
                ++(bimodal[bimodalIndex].pred);             // Increment prediction counter
            else if(!taken && predictionVal > 0) 
                --(bimodal[bimodalIndex].pred);
        }
        //~
        
        // Update altBetterCount
        if(table_pred < NUM_TAGE_TABLES) {                  // Table hit
            if((tagTables[table_pred][index_pred].u == 0) &&                     
                ((tagTables[table_pred][index_pred].pred  == WEAKLY_NOT_TAKEN) ||  
                    (tagTables[table_pred][index_pred].pred  == WEAKLY_TAKEN))) {    // Entry unused and weakly confused            

                // Its new in the table
                if (pred_pred != altPred_pred) {                    // If predictions are different
                    if (altPred_pred == taken) {                    
                        if (altBetterCount < ALTPRED_BET_MAX)       // Increment if alternate was correct
                            altBetterCount++;                    
                    } 
                    else if (altBetterCount > 0)                    // Decrement if alternate was wrong
                        altBetterCount--;                            
                }
            }
        }
        //~
        
        if (((predDir != taken) && (table_pred > 0))) {          // Prediction wrong and no tag hit
            bool alloc = false;
            for (int i = 0; i < table_pred; i++) 
                if (tagTables[i][tageIndex[i]].u == 0)          // Find useful tables
                    alloc = true;

            if (!alloc) {                                       // If none
                for (int i = table_pred - 1; i >= 0; i--)
                    tagTables[i][tageIndex[i]].u--;             // Decrement usefulness
            } 
            else {  
                for(int i = table_pred-1; i>=0; i--){
                    if((tagTables[i][tageIndex[i]].u == 0 && !(rand()%10))) {   // Do uniform rejection sampling
                        if(taken) {  
                            tagTables[i][tageIndex[i]].pred = WEAKLY_TAKEN; 
                        } 
                        else {  
                            tagTables[i][tageIndex[i]].pred = WEAKLY_NOT_TAKEN;
                        }    

                        tagTables[i][tageIndex[i]].tag = tageTag[i];  // Reset tag
                        tagTables[i][tageIndex[i]].u = 0;             // Reset usefulness
                        break; 
                    }
                }
            }
        }
        
        if(table_pred < NUM_TAGE_TABLES) {  // If table hit 
            if ((predDir != altPred_pred)) {  
                if (predDir == taken && tagTables[table_pred][index_pred].u < PRED_U_MAX) // altpred not used   
                    tagTables[table_pred][index_pred].u += 1;  // increment usefulness
                else if(predDir != taken && tagTables[table_pred][index_pred].u > 0)
                    tagTables[table_pred][index_pred].u -= 1;  // decrement usefulness
            }  
        }
        
        clockk++;                           // Add cycle
        if(clockk == (1<<CLOCK_MAX)) { 	    // Reset after every 2^CLOCK_MAX cycles
            clockk = 0;   
            clockState = 1 - clockState;               
            
            for(uint32_t i = 0; i < NUM_TAGE_TABLES; i++)
                for(uint32_t j = 0; j < (1<<TAGE_TABLE_SIZE[i]); j++)
                    tagTables[i][j].u &= (clockState+1);  // If clockstate=0, reset lower bit else upper bit
        }
        
        *GHR = (*GHR << 1); // Shift GHR
        if(taken == TAKEN)
            GHR->set(0,1); 
        
        for (int i = 0; i < NUM_TAGE_TABLES; i++) { // Perform folding on Circular Shift Registers
            csrIndex[i].val = (csrIndex[i].val << 1) + (*GHR)[0];
            csrIndex[i].val ^= ((csrIndex[i].val & (1 << csrIndex[i].newlen)) >> csrIndex[i].newlen);
            csrIndex[i].val ^= ((*GHR)[csrIndex[i].oldlen] << (csrIndex[i].oldlen % csrIndex[i].newlen));
            csrIndex[i].val &= ((1 << csrIndex[i].newlen) -1);

            csrTag[0][i].val = (csrTag[0][i].val << 1) + (*GHR)[0];
            csrTag[0][i].val ^= ((csrTag[0][i].val & (1 << csrTag[0][i].newlen)) >> csrTag[0][i].newlen);
            csrTag[0][i].val ^= ((*GHR)[csrTag[0][i].oldlen] << (csrTag[0][i].oldlen % csrTag[0][i].newlen));
            csrTag[0][i].val &= ((1 << csrTag[0][i].newlen) -1);

            csrTag[1][i].val = (csrTag[1][i].val << 1) + (*GHR)[0];
            csrTag[1][i].val ^= ((csrTag[1][i].val & (1 << csrTag[1][i].newlen)) >> csrTag[1][i].newlen);
            csrTag[1][i].val ^= ((*GHR)[csrTag[1][i].oldlen] << (csrTag[1][i].oldlen % csrTag[1][i].newlen));
            csrTag[1][i].val &= ((1 << csrTag[1][i].newlen) -1);
        }
        
        // Update Path History
        PHR = (PHR << 1);
        if(target & 1) 
            PHR = PHR + 1;
        PHR = (PHR & ((1 << PHR_LEN) - 1));
	}
};