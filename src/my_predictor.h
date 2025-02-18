#include "tage.h"
#include "loop_predictor.h"
#include "ittage.h"
#include <iostream>
#include <fstream>

#include <fstream>

// std::ofstream out("a.txt");

class my_update : public branch_update {
    public:
        unsigned int index;
};
    
class my_predictor : public branch_predictor {
public:
    tage_predictor tage;
    loop_predictor loop;
    ittage_predictor ittage;
    
    branch_update* tage_pred;
    branch_update* loop_pred;
    branch_update* ittage_pred;
    
    int loop_correct;

    my_predictor (void): loop_correct(0) {}

    void update_ctr (bool taken) {
        if (taken == loop_pred->direction_prediction()) {
            if (loop_correct < 127) 
                loop_correct++;
        }
        else if (loop_correct > -126) 
            loop_correct--;
    }

    branch_update *predict (branch_info & b) {
        tage_pred = tage.predict(b);
        // loop_pred = loop.predict(b);
        ittage_pred = ittage.predict(b);

        // // if (loop.is_valid && loop_correct >= 0) {
        // //     return loop_pred;
        // // }

        if (b.br_flags & BR_INDIRECT)
            return ittage_pred;
        else
            return tage_pred;
    }

    void update (branch_update *u, bool taken, unsigned int target) {
        tage.update(u, taken, target);
        // loop.update(u, taken, target, tage_pred->direction_prediction());
        ittage.update(u, taken, target);

        // if (loop.is_valid && tage_pred->direction_prediction() != loop_pred->direction_prediction()) 
        //     update_ctr(taken);
    }
};