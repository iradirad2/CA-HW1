/* 046267 Computer Architecture - HW #1                                 */
/* This file should hold your implementation of the predictor simulator */


#include "bp_api.h"


typedef struct {
    uint32_t tag;
    uint32_t target;
    uint32_t local_history;  // only if local
	uint8_t* local_fsmTable;   
} BTB_entry;

typedef struct {
	BTB_entry* entries;
	
	uint32_t global_history;
	uint8_t* global_fsmTable; 
	
	unsigned btbSize; 
	unsigned historySize; 
	unsigned tagSize; 
	unsigned fsmState;
	bool isGlobalHist; 
	bool isGlobalTable; 
	int Shared;

} BTB;

static BTB btb;
static SIM_stats stats;

int BP_init(unsigned btbSize, unsigned historySize, unsigned tagSize, unsigned fsmState,
			bool isGlobalHist, bool isGlobalTable, int Shared){

	// initialize parameter
	btb.btbSize = btbSize; 
	btb.historySize = historySize; 
	btb.tagSize = tagSize; 
	btb.fsmState = fsmState;
	btb.isGlobalHist = isGlobalHist; 
	btb.isGlobalTable = isGlobalTable; 
	btb.Shared = Shared;

	
	// allocate memory to entries
	btb.entries = malloc(btb.btbSize * sizeof(BTB_entry));
	if(!btb.entries)
		return -1;

	// reset history registers
    if(!isGlobalHist) {
        for (int i = 0; i < btbSize; i++) {
            btb.entries[i].local_history = 0;
        }
    } else {
        btb.global_history = 0;
    }

	if(isGlobalTable){
		// one global fsm
		int fsm_size = 1 << btb.historySize;
    	btb.entries = calloc(btbSize, sizeof(BTB_entry));
		if(!btb.global_fsmTable)
			return -1;

		// initialize the fsm state
		for(int i=0; i<fsmSize; i++)
            btb.global_fsmTable[i] = fsmState;
	} else{
		//local fsm for each entry
		int fsm_size = 1 << btb.historySize;
		for(int i=0; i < btb.btbSize; i++){
			btb.global_fsmTable = calloc(fsmSize, sizeof(uint8_t));
			if(!btb.entries[i].local_fsmTable)
				return -1;
			
			// initialize the fsm state
			for(int j=0; i<fsmSize; i++)
            btb.global_fsmTable[j] = fsmState;
		}
	}
	return 0;
}

bool BP_predict(uint32_t pc, uint32_t *dst){
	return false;
}

void BP_update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst){
	return;
}

void BP_GetStats(SIM_stats *curStats){
	return;
}


