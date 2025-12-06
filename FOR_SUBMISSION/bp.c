/* 046267 Computer Architecture - HW #1                                 */
/* This file should hold your implementation of the predictor simulator */

// submission of Areg 315031203 and Irad 207197500
// version 1.1

#include "bp_api.h"
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

#define B_ADDR_LEN 32
#define B_MEM_ALIGN_LEN 2
#define B_HIST_MAX_SIZE 8
#define B_FSM_SIZE 2
#define SHARE_LSB 2
#define SHARE_MID 16

typedef uint32_t tag_t;
typedef uint32_t addr_t;
typedef uint8_t histbuf_t;

typedef enum { MIN_BOUND, SNT, WNT, WT, ST, MAX_BOUND } predictor_t;

// struct for each entry
typedef struct {
    tag_t m_tag;
    addr_t IP_addr;
    addr_t m_target_addr;
    predictor_t *m_local_fsm_arr; // local fsm array

    histbuf_t m_local_history; // local history buffer

    bool m_last_check;
    bool m_used;
    bool m_valid;
} BTB_entry;

// struct for the btb
typedef struct {
    BTB_entry *m_ent; // array of entries

    uint8_t m_size; // size of btb
    uint8_t m_tag_size;
    uint8_t m_history_size;
    histbuf_t m_history;        // global history buffer
    predictor_t m_default_stat; // default fsm state

    predictor_t *m_fsm_arr; // global fsm array

    bool is_global_table;
    bool is_global_history;
    uint8_t share;

    unsigned int m_fsm_arr_size;
    unsigned int m_flush_num;  // Machine flushes
    unsigned int m_br_num;     // Number of branch instructions
    unsigned int m_b_mem_size; // Theoretical allocated BTB and branch pred size
} BTB;

static BTB btb; // global static instance of the btb

// helper function prototypes
short my_log2(uint8_t size);
size_t calc_mem_usage();
short get_entry(uint32_t pc);
tag_t get_tag(uint32_t pc);
uint8_t get_fsm_arr_idx(const histbuf_t hist_buf, addr_t pc);
void update_hist_buf(histbuf_t *hist_buf, bool taken);
uint8_t get_history(const histbuf_t hist_buf);
uint8_t align_for_history(const uint32_t addr);
short ttp(uint8_t exponent); // ttp = two to the power

int BP_init(unsigned btbSize, unsigned historySize, unsigned tagSize,
            unsigned fsmState, bool isGlobalHist, bool isGlobalTable,
            int Shared) {

    // initialize all structure fields
    btb.m_size = btbSize;
    btb.m_history_size = historySize;
    btb.m_tag_size = tagSize;
    btb.m_default_stat = (predictor_t)((int)fsmState + 1);
    btb.is_global_history = isGlobalHist;
    btb.is_global_table = isGlobalTable;
    btb.m_flush_num = 0;
    btb.m_br_num = 0;
    btb.m_b_mem_size = calc_mem_usage();
    btb.m_fsm_arr_size = ttp(btb.m_history_size);

    // if shared and not global then make shared 0, cause it doesn't make sense
    // otherwise
    if (Shared && !isGlobalTable) {
        btb.share = 0;
    } else {
        btb.share = Shared;
    }

    // create entries
    btb.m_ent = (BTB_entry *)malloc(sizeof(BTB_entry) * btb.m_size);

    // create global or local history
    if (!btb.is_global_history) {
        for (int ent_idx = 0; ent_idx < btb.m_size; ent_idx++) {
            btb.m_ent[ent_idx].m_local_history = 0;
        }
    } else {
        btb.m_history = 0;
    }

    // allocate global or local fsm table
    const unsigned int fsm_arr_size = btb.m_fsm_arr_size;
    if (!btb.is_global_table) {
        for (int ent_idx = 0; ent_idx < btb.m_size; ent_idx++) {
            btb.m_ent[ent_idx].m_local_fsm_arr =
                (predictor_t *)malloc(sizeof(predictor_t) * (fsm_arr_size));
            predictor_t *entr_fsm_arr = btb.m_ent[ent_idx].m_local_fsm_arr;
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

    // initialize valid bits to 0
    for (int ent_idx = 0; ent_idx < btb.m_size; ent_idx++) {
        btb.m_ent[ent_idx].m_valid = false;
    }

    return 0; // success
}

// true for taken - false for not taken
bool BP_predict(uint32_t pc, uint32_t *dst) {
    short ent = get_entry(pc);
    tag_t tag = get_tag(pc);
    BTB_entry *cur_ent = &btb.m_ent[ent];

    // if the entry doesn't exist, return false and set dest to pc+4
    if (cur_ent->m_valid == false || cur_ent->m_tag != tag) {
        cur_ent->m_last_check = false;
        *dst = pc + 4;
        return false;
    }

    histbuf_t *cur_hist_buf = NULL;
    predictor_t *cur_pred_arr = NULL;

    // choose if we work with global or local history
    if (!btb.is_global_history)
        cur_hist_buf = &cur_ent->m_local_history;
    else
        cur_hist_buf = &btb.m_history;

    if (!btb.is_global_table)
        cur_pred_arr = cur_ent->m_local_fsm_arr;
    else
        cur_pred_arr = btb.m_fsm_arr;

    // find the index inside the fsm table with the history
    uint8_t arr_idx = get_fsm_arr_idx(*cur_hist_buf, pc);

    // reutrn the prediction according to the fsm table
    switch (cur_pred_arr[arr_idx]) {
    case ST:
    case WT:
        cur_ent->m_last_check = true;
        *dst = cur_ent->m_target_addr;
        return true;
    case WNT:
    case SNT:
        cur_ent->m_last_check = false;
        *dst = pc + 4;
        return false;
    default:
        return false; // this shouldn't happen
    }
}

void BP_update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst) {
    short ent = get_entry(pc);
    tag_t tag = get_tag(pc);
    BTB_entry *cur_ent = &btb.m_ent[ent];

    // if the entry doesn't have the same tag, then reset the history and the
    // fsm array, if they are not global
    if (cur_ent->m_tag != tag) {
        if (!btb.is_global_history) {
            cur_ent->m_local_history = 0;
        }
        if (!btb.is_global_table) {
            predictor_t *fsm_arr = cur_ent->m_local_fsm_arr;
            for (int i = 0; i < btb.m_fsm_arr_size; i++) {
                fsm_arr[i] = btb.m_default_stat;
            }
        }
    }
    cur_ent->m_tag = tag;
    cur_ent->m_target_addr = targetPc;
    cur_ent->m_valid = true;
    cur_ent->IP_addr = pc;

    histbuf_t *cur_hist_buf = NULL;
    predictor_t *cur_pred_arr = NULL;

    // choose what to work with
    if (!btb.is_global_history)
        cur_hist_buf = &cur_ent->m_local_history;
    else
        cur_hist_buf = &btb.m_history;
    if (!btb.is_global_table)
        cur_pred_arr = cur_ent->m_local_fsm_arr;
    else
        cur_pred_arr = btb.m_fsm_arr;

    // get the index
    uint8_t arr_idx = get_fsm_arr_idx(*cur_hist_buf, pc);

    // if the predition was wrong, then flush
    if (cur_ent->m_last_check != taken) {
        btb.m_flush_num++;
        // if the prediction was right, and was taken, but the target and
        // destination are not the same, then flush (can be caused because of
        // aliasing)
    } else if (taken && (cur_ent->m_target_addr != pred_dst)) {
        btb.m_flush_num++;
    }
    btb.m_br_num++;

    update_hist_buf(cur_hist_buf, taken);

    // keep the next state of the fsm in bounds
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
    // pass all statistics to the main function
    curStats->flush_num = btb.m_flush_num;
    curStats->br_num = btb.m_br_num;
    curStats->size = btb.m_b_mem_size;

    // release memory
    if (!btb.is_global_table) {
        for (int ent_idx = 0; ent_idx < btb.m_size; ent_idx++) {
            free(btb.m_ent[ent_idx].m_local_fsm_arr);
        }
    } else {
        free(btb.m_fsm_arr);
    }

    free(btb.m_ent);
}

// -------------------- helper functions --------------------

// simple log function
short my_log2(uint8_t size) {
    short result = 0;
    while (size >>= 1)
        result++;
    return result;
}

// return 2^exponent
short ttp(uint8_t exponent) { return (1 << exponent); }

short get_entry(uint32_t pc) {
    const int b_idx_len = my_log2(btb.m_size);
    if (b_idx_len == 0)
        return 0;
    // get the entry bits, with masking the other bits
    uint32_t ent = pc >> B_MEM_ALIGN_LEN;
    ent <<= B_ADDR_LEN - b_idx_len;
    ent >>= B_ADDR_LEN - b_idx_len;
    return ent;
}

tag_t get_tag(uint32_t pc) {
    if (btb.m_tag_size == 0)
        return 0;

    const int b_idx_len = my_log2(btb.m_size);
    // get the tag bits with masking the other bits
    tag_t tag = pc >> (b_idx_len + B_MEM_ALIGN_LEN);
    tag <<= B_ADDR_LEN - btb.m_tag_size;
    tag >>= B_ADDR_LEN - btb.m_tag_size;
    return tag;
}

uint8_t get_history(const histbuf_t hist_buf) {
    if (btb.m_history_size == B_HIST_MAX_SIZE)
        return hist_buf; // cause hist_buf is a uint8_t anyway

    // return the relevant bits of the history buffer with masking the other
    // bits
    histbuf_t history = hist_buf << (B_HIST_MAX_SIZE - btb.m_history_size);
    history >>= B_HIST_MAX_SIZE - btb.m_history_size;
    return history;
}

uint8_t align_for_history(const uint32_t addr) {
    // get the relevant bits from the address to do a xor with the history
    // buffer
    uint8_t aligned = addr << (B_HIST_MAX_SIZE - btb.m_history_size);
    aligned >>= B_HIST_MAX_SIZE - btb.m_history_size;
    return aligned;
}

uint8_t get_fsm_arr_idx(const histbuf_t hist_buf, addr_t pc) {
    uint8_t arr_idx;
    if (btb.share == 1) {
        arr_idx = get_history(hist_buf) ^ align_for_history(pc >> SHARE_LSB);
    } else if (btb.share == 2) {
        arr_idx = get_history(hist_buf) ^ align_for_history(pc >> SHARE_MID);
    } else {
        arr_idx = get_history(hist_buf);
    }
    return arr_idx;
}

void update_hist_buf(histbuf_t *hist_buf, bool taken) {
    // add the last outcome to the history
    *hist_buf <<= 1;
    if (!taken)
        return;
    *hist_buf |= 1; // 1 is a mask
}

size_t calc_mem_usage() {
    const uint8_t target_size = 30;
    const uint8_t valid_size = 1;

    uint8_t n_entries = btb.m_size;
    uint8_t tag_size = btb.m_tag_size;
    uint8_t history_size = btb.m_history_size;
    short fsm_tbl_size = B_FSM_SIZE * (ttp(history_size));

    size_t mem_size;
    if (!btb.is_global_history && !btb.is_global_table) {
        mem_size = n_entries         //
                   * (tag_size       //
                      + target_size  //
                      + valid_size   //
                      + history_size //
                      + fsm_tbl_size);

    } else if (btb.is_global_history && !btb.is_global_table) {
        mem_size = n_entries              //
                       * (tag_size        //
                          + target_size   //
                          + valid_size    //
                          + fsm_tbl_size) //
                   + history_size;

    } else if (!btb.is_global_history && btb.is_global_table) {
        mem_size = n_entries              //
                       * (tag_size        //
                          + target_size   //
                          + valid_size    //
                          + history_size) //
                   + fsm_tbl_size;

    } else {
        mem_size = n_entries * (valid_size + tag_size + target_size) //
                   + (history_size + fsm_tbl_size);
    }

    return mem_size;
}
