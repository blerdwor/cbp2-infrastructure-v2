#include "tage.h"

class my_update : public branch_update {
    public:
        unsigned int index;
    };
    
    class my_predictor : public branch_predictor {
    public:
        tage_predictor tage;
        branch_update* u;
    
        my_predictor (void) {}
    
        branch_update *predict (branch_info & b) {
            u = tage.predict(b);
            return u;
        }
    
        void update (branch_update *u, bool taken, unsigned int target) {
            tage.update(u, taken, target);
        }
    };