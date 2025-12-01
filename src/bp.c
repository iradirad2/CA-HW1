/* 046267 Computer Architecture - HW #1                                 */
/* This file should hold your implementation of the predictor simulator */

#include "bp_api.h"
#include <stddef.h>
#include <stdlib.h>

#define B_ADDR_LEN 32
#define B_MEM_ALIGN_LEN 2
#define B_HIST_MAX_SIZE 8
#define B_FSM_SIZE 2

typedef uint32_t tag_t;
typedef uint32_t addr_t;
typedef uint8_t histbuf_t;

typedef enum { MIN_BOUND, SNT, WNT, WT, ST, MAX_BOUND } predictor_t;

typedef struct {
    tag_t m_tag;
    addr_t IP_addr;
    addr_t m_target_addr;

    predictor_t *m_local_fsm_arr;
    histbuf_t m_local_history;

    bool m_used;
} BTB_entry;

typedef struct {
    BTB_entry *m_ent;

    uint8_t m_size;
    uint8_t m_tag_size;
    uint8_t m_history_size;
    histbuf_t m_history;
    predictor_t m_default_stat;

    predictor_t *m_fsm_arr;

    bool is_global_table;
    bool is_global_history;
    bool share;

    unsigned int m_flush_num;  // Machine flushes
    unsigned int m_br_num;     // Number of branch instructions
    unsigned int m_b_mem_size; // Theoretical allocated BTB and branch pred size
} BTB;

static BTB btb;

short my_log2(uint8_t size);
size_t calc_mem_usage();
short get_entry(uint32_t pc);
tag_t get_tag(uint32_t pc);
uint8_t get_fsm_arr_idx(const histbuf_t hist_buf, const BTB_entry entery);
void update_hist_buf(histbuf_t *hist_buf, bool taken);
uint8_t get_history(const histbuf_t hist_buf);
short ttp(uint8_t exponent); // ttp = two to the power

int BP_init(unsigned btbSize, unsigned historySize, unsigned tagSize,
            unsigned fsmState, bool isGlobalHist, bool isGlobalTable,
            int Shared) {

    btb.m_size = btbSize;
    btb.m_history_size = historySize;
    btb.m_tag_size = tagSize;
    btb.m_default_stat = fsmState;
    btb.is_global_history = isGlobalHist;
    btb.is_global_table = isGlobalTable;
    btb.m_flush_num = 0;
    btb.m_br_num = 0;
    btb.m_b_mem_size = calc_mem_usage();

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

    const int fsm_arr_size = ttp(btb.m_history_size);
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

// true for taken - false for not taken
bool BP_predict(uint32_t pc, uint32_t *dst) {
    short ent = get_entry(pc);
    tag_t tag = get_tag(pc);
    BTB_entry *cur_ent = &btb.m_ent[ent];

    if (cur_ent->m_tag != tag)
        return false;

    histbuf_t *cur_hist_buf = NULL;
    predictor_t *cur_pred_arr = NULL;

    if (!btb.is_global_history)
        cur_hist_buf = &cur_ent->m_local_history;
    else
        cur_hist_buf = &btb.m_history;

    if (!btb.is_global_table)
        cur_pred_arr = cur_ent->m_local_fsm_arr;
    else
        cur_pred_arr = btb.m_fsm_arr;

    uint8_t arr_idx = get_fsm_arr_idx(*cur_hist_buf, *cur_ent);

    switch (cur_pred_arr[arr_idx]) {
    case ST:
    case WT:
        *dst = cur_ent->m_target_addr;
        return true;
    case WNT:
    case SNT:
        *dst += 4;
        return false;
    default:
        return false; // this shouldn't happen
    }
}

void BP_update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst) {
    short ent = get_entry(pc);
    tag_t tag = get_tag(pc);
    BTB_entry *cur_ent = &btb.m_ent[ent];

    cur_ent->m_tag = tag;
    cur_ent->m_target_addr = targetPc;

    histbuf_t *cur_hist_buf = NULL;
    predictor_t *cur_pred_arr = NULL;

    if (!btb.is_global_history)
        cur_hist_buf = &cur_ent->m_local_history;
    else
        cur_hist_buf = &btb.m_history;
    if (!btb.is_global_table)
        cur_pred_arr = cur_ent->m_local_fsm_arr;
    else
        cur_pred_arr = btb.m_fsm_arr;
    uint8_t arr_idx = get_fsm_arr_idx(*cur_hist_buf, *cur_ent);

    bool prediction = BP_predict(pc, &pred_dst);
    if (prediction != taken) {
        btb.m_flush_num++;
    }
    btb.m_br_num++;

    update_hist_buf(cur_hist_buf, taken);
    predictor_t next_state;
    if (taken == true) {
        next_state = (predictor_t)((int)cur_pred_arr[arr_idx] + 1);
        if (next_state == MAX_BOUND)
            next_state = ST;
    } else {
        next_state = (predictor_t)((int)cur_pred_arr[arr_idx] - 1);
        if (next_state == MIN_BOUND)
            next_state = SNT;
    }
    cur_pred_arr[arr_idx] = next_state;
}

void BP_GetStats(SIM_stats *curStats) {
    curStats->flush_num = btb.m_flush_num;
    curStats->br_num = btb.m_br_num;
    curStats->size = btb.m_b_mem_size;
}

// -------------------- helper functions --------------------

short my_log2(uint8_t size) {
    short result = 0;
    while (size >>= 1)
        result++;
    return result;
}

short ttp(uint8_t exponent) { return (2 << exponent); }

short get_entry(uint32_t pc) {
    const int b_idx_len = my_log2(btb.m_size);
    uint32_t ent = pc >> B_MEM_ALIGN_LEN;
    ent <<= B_ADDR_LEN - b_idx_len;
    ent >>= B_ADDR_LEN - b_idx_len;
    return ent;
}

tag_t get_tag(uint32_t pc) {
    const int b_idx_len = my_log2(btb.m_size);
    tag_t tag = pc >> (b_idx_len + B_MEM_ALIGN_LEN);
    tag <<= B_ADDR_LEN - btb.m_tag_size;
    tag >>= B_ADDR_LEN - btb.m_tag_size;
    return tag;
}

uint8_t get_history(const histbuf_t hist_buf) {
    if (btb.m_history_size == B_HIST_MAX_SIZE)
        return hist_buf; // cause hist_buf is a uint8_t anyway

    histbuf_t history = hist_buf << (B_HIST_MAX_SIZE - btb.m_history_size);
    history >>= B_HIST_MAX_SIZE - btb.m_history_size;
    return history;
}

uint8_t get_fsm_arr_idx(const histbuf_t hist_buf, const BTB_entry entery) {
    uint8_t arr_idx;
    if (btb.share) {
        arr_idx = get_history(hist_buf) ^
                  entery.IP_addr; // probably need to change this
    }

    arr_idx = get_history(hist_buf);
    return arr_idx;
}

void update_hist_buf(histbuf_t *hist_buf, bool taken) {
    *hist_buf <<= 1;
    if (!taken)
        return;
    *hist_buf |= 1; // 1 is a mask
}

size_t calc_mem_usage() {
    uint8_t n_entries = btb.m_size;
    uint8_t tag_size = btb.m_tag_size;
    uint8_t target_size = my_log2(btb.m_size);
    uint8_t history_size = btb.m_history_size;
    uint8_t fsm_tbl_size = B_FSM_SIZE * (ttp(history_size));

    size_t mem_size;
    if (!btb.is_global_history && !btb.is_global_table) {
        mem_size = n_entries         //
                   * (tag_size       //
                      + target_size  //
                      + history_size //
                      + fsm_tbl_size);

    } else if (btb.is_global_history && !btb.is_global_table) {
        mem_size = n_entries              //
                       * (tag_size        //
                          + target_size   //
                          + fsm_tbl_size) //
                   + history_size;

    } else if (!btb.is_global_history && btb.is_global_table) {
        mem_size = n_entries              //
                       * (tag_size        //
                          + target_size   //
                          + history_size) //
                   + fsm_tbl_size;

    } else {
        mem_size = n_entries * (tag_size + target_size) //
                   + (history_size + fsm_tbl_size);
    }

    return mem_size;
}
