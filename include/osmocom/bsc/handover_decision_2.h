/* Handover Decision Algorithm 2 for intra-BSC (inter-BTS) handover, public API for OsmoBSC */

#pragma once
struct gsm_bts;

void handover_decision_2_init();
void handover_decision_2_reinit_congestion_timer(struct gsm_bts *bts);
