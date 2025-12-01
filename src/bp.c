/* 046267 Computer Architecture - HW #1                                 */
/* This file should hold your implementation of the predictor simulator */

#include "bp_api.h"
#include <stddef.h>
#include <stdlib.h>

typedef uint32_t tag_t;
typedef uint32_t addr_t;
typedef uint8_t histbuf_t;

typedef enum { SNT, WNT, WT, ST } predictor_t;

typedef struct {
    tag_t m_tag;
    addr_t IP_addr;
    addr_t m_target_addr;

    predictor_t *m_local_fsm_arr;
    histbuf_t m_local_history;

    bool m_used;
} BTB_entry;

typedef struct {
    uint8_t m_size;
    uint8_t m_tag_size;
    uint8_t m_history_size;
    predictor_t m_default_stat;
    BTB_entry *m_ent;

    predictor_t *m_fsm_arr;
    histbuf_t m_history;

    bool is_global_table;
    bool is_global_history;
    bool share;
} BTB;

static BTB btb;

int BP_init(unsigned btbSize, unsigned historySize, unsigned tagSize,
            unsigned fsmState, bool isGlobalHist, bool isGlobalTable,
            int Shared) {

    btb.m_size = btbSize;
    btb.m_history_size = historySize;
    btb.m_tag_size = tagSize;
    btb.m_default_stat = fsmState;
    btb.is_global_history = isGlobalHist;
    btb.is_global_table = isGlobalTable;

    if (Shared && !isGlobalTable)
        return -1; // failure

    btb.share = Shared;

    btb.m_ent = (BTB_entry *)malloc(sizeof(BTB_entry) * btb.m_size);

    if (!btb.is_global_history) {
        for (int ent_idx = 0; ent_idx < btb.m_size; ent_idx++) {
            btb.m_ent[ent_idx].m_local_history = 0;
        }
    } else {
        btb.m_history = 0;
    }

    const int fsm_arr_size = 2 ^ btb.m_history_size;
    if (!btb.is_global_table) {
        for (int ent_idx = 0; ent_idx < btb.m_size; ent_idx++) {
            predictor_t *entr_fsm_arr = btb.m_ent[ent_idx].m_local_fsm_arr;
            entr_fsm_arr =
                (predictor_t *)malloc(sizeof(predictor_t) * (fsm_arr_size));
            for (int fsm_idx = 0; fsm_idx < fsm_arr_size; fsm_idx++) {
                entr_fsm_arr[fsm_idx] = btb.m_default_stat;
            }
        }
    } else {
        btb.m_fsm_arr =
            (predictor_t *)malloc(sizeof(predictor_t) * (fsm_arr_size));
        for (int fsm_idx = 0; fsm_idx < fsm_arr_size; fsm_idx++) {
            btb.m_fsm_arr[fsm_idx] = btb.m_default_stat;
        }
    }

    return 0; // success
}

bool BP_predict(uint32_t pc, uint32_t *dst) { return false; }

void BP_update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst) {
    return;
}

void BP_GetStats(SIM_stats *curStats) { return; }
