#ifndef TOOLS_H
#define TOOLS_H

#include <bitset>

// Common constants between TAGE and ITTAGE
#define INT32	int32_t
#define UINT32	uint32_t

#define TAKEN		true
#define NOT_TAKEN	false

#define GHIST_SIZE	129

#define ALT_BETTER_COUNT_MAX	15 			// 4bit counter for the max number of times that the alternate predictor was better
#define CLOCK_RESET_PERIOD		256*1024	// Useful bit resets after 256K branches (as per paper)

// Folded history compression; GHR(geometric length) -> Compressed(target)
struct FoldedHist {
    UINT32 geomLength;		// Geometric history length
    UINT32 targetLength;	// Cropped size
    UINT32 compHist;		// Compressed history
      
    void updateCompHist(std::bitset<GHIST_SIZE> ghr) {
        int mask = (1 << targetLength) - 1;
        int mask1 = ghr[geomLength] << (geomLength % targetLength);
        int mask2 = (1 << targetLength);
		compHist  = (compHist << 1) + ghr[0];
		compHist ^= ((compHist & mask2) >> targetLength);
		compHist ^= mask1;
		compHist &= mask;     
    }    
};

#endif  // TOOLS_H