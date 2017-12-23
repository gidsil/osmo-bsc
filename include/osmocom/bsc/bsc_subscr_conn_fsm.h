#pragma once
#include <osmocom/core/fsm.h>

enum gscon_fsm_event {
	/* local SCCP stack tells us incoming conn from MSC */
	GSCON_EV_A_CONN_IND,
	/* RSL side requests CONNECT to MSC */
	GSCON_EV_A_CONN_REQ,
	/* MSC confirms the SCCP connection */
	GSCON_EV_A_CONN_CFM,
	/* MSC requests assignment */
	GSCON_EV_A_ASSIGNMENT_CMD,
	/* MSC has sent BSSMAP CLEAR CMD */
	GSCON_EV_A_CLEAR_CMD,
	/* MSC SCCP disconnect indication */
	GSCON_EV_A_DISC_IND,
	/* MSC sends Handover Request (in CR) */
	GSCON_EV_A_HO_REQ,

	/* RR ASSIGNMENT COMPLETE received */
	GSCON_EV_RR_ASS_COMPL,
	/* RR ASSIGNMENT FAIL received */
	GSCON_EV_RR_ASS_FAIL,
	/* RR MODE MODIFY ACK received */
	GSCON_EV_RR_MODE_MODIFY_ACK,
	/* RR HO ACC (access burst on ext HO) */
	GSCON_EV_RR_HO_ACC,
	/* RR HANDOVER COMPLETE received */
	GSCON_EV_RR_HO_COMPL,
	/* RSL RLL Release Indication */
	GSCON_EV_RLL_REL_IND,
	/* RSL CONNection FAILure Indication */
	GSCON_EV_RSL_CONN_FAIL,

	/* RSL/lchan tells us clearing is complete */
	GSCON_EV_RSL_CLEAR_COMPL,

	/* Mobile-originated DTAP (from MS) */
	GSCON_EV_MO_DTAP,
	/* Mobile-terminated DTAP (from MSC) */
	GSCON_EV_MT_DTAP,
};

struct gsm_subscriber_connection;
struct gsm_network;

/* Allocate a subscriber connection and its associated FSM */
struct gsm_subscriber_connection *bsc_subscr_con_allocate(struct gsm_network *net);
