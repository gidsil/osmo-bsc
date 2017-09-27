/* (C) 2017 by sysmocom s.f.m.c. GmbH
 * All Rights Reserved
 *
 * Author: Philipp Maier
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <osmocom/mgcp_client/mgcp_client.h>
#include <osmocom/bsc/gsm_data.h>
#include <osmocom/bsc/osmo_bsc_mgcp.h>
#include <osmocom/bsc/debug.h>
#include <osmocom/bsc/osmo_bsc.h>
#include <osmocom/core/logging.h>
#include <osmocom/core/utils.h>
#include <osmocom/core/timer.h>
#include <osmocom/core/fsm.h>
#include <osmocom/bsc/osmo_bsc_sigtran.h>
#include <osmocom/core/byteswap.h>
#include <arpa/inet.h>

#define CONN_ID_BTS 1
#define CONN_ID_NET 2

#define MGCP_MGW_TIMEOUT 4	/* sek */
#define MGCP_MGW_TIMEOUT_TIMER_NO 7411
#define MGCP_BSS_TIMEOUT 4	/* sek */
#define MGCP_BSS_TIMEOUT_TIMER_NO 7412

#define MGCP_ENDPOINT_FORMAT "%i@mgw"

extern struct gsm_network *bsc_gsmnet;

enum fsm_states {
	/* Initalization state to start the FSM */
	ST_INIT,

	/* Send CRCX for BTS and wait for response */
	ST_CRCX_BTS,

	/* Wait for the BSS to setup the connection */
	ST_WAIT_BSS,

	/* Send MDCX for BTS and wait for response */
	ST_MDCX_BTS,

	/* Send CRCX (single phase) for NET and wait for response */
	ST_CRCX_NET,

	/* Call is now active, wait for call end */
	ST_WAIT_END,

	/* Send DLCX for BTS/NET and wait for response */
	ST_DLCX_ALL,
};

static const struct value_string fsm_state_names[] = {
	{ST_INIT, "ST_INIT (initalize FSM)"},
	{ST_CRCX_BTS, "ST_CRCX_BTS (create BTS connection)"},
	{ST_WAIT_BSS, "ST_WAIT_BSS (wait for BSS)"},
	{ST_MDCX_BTS, "ST_MDCX_BTS (update BTS connection)"},
	{ST_CRCX_NET, "ST_CRCX_NET (create NET connection)"},
	{ST_CRCX_NET, "ST_DLCX_ALL (delete BTS/ connection)"},
	{ST_WAIT_END, "ST_WAIT_END (wait for call end)"},
	{0, NULL},
};

enum fsm_evt {
	/* Initial event: start off the state machine */
	EV_INIT,

	/* External event: Assignment complete, event is issued shortly before
	 * the assignment complete message is sent via the A-Interface */
	EV_ASS_COMPLETE,

	/* External event: Teardown event, this event is used to notify the end
	 * of a. It is also issued in case of errors to teardown a half open
	 * connection. */
	EV_TEARDOWN,

	/* Internal event: The mgcp_gw has sent its CRCX response for
	 * the BTS side */
	EV_CRCX_BTS_RESP,

	/* Internal event: The mgcp_gw has sent its MDCX response for
	 * the BTS side */
	EV_MDCX_BTS_RESP,

	/* Internal event: The mgcp_gw has sent its CRCX response for
	 * the NET side */
	EV_CRCX_NET_RESP,

	/* Internal event: The mgcp_gw has sent its DLCX response for
	 * the NET and BTS side */
	EV_DLCX_ALL_RESP,
};

static const struct value_string fsm_evt_names[] = {
	{EV_INIT, "EV_INIT (start state machine (send CRCX for BTS)"},
	{EV_ASS_COMPLETE, "EV_ASS_COMPLETE (assignment complete)"},
	{EV_TEARDOWN, "EV_TEARDOWN (teardown all connections)"},
	{EV_CRCX_BTS_RESP, "EV_CRCX_BTS_RESP (got CRCX reponse for BTS)"},
	{EV_MDCX_BTS_RESP, "EV_MDCX_BTS_RESP (got MDCX reponse for BTS)"},
	{EV_CRCX_NET_RESP, "EV_CRCX_NET_RESP (got CRCX reponse for NET)"},
	{EV_DLCX_ALL_RESP, "EV_DLCX_ALL_RESP (got DLCX reponse for BTS/NET)"},
	{0, NULL},
};

/* On error, go directly to the DLCX phase. */
static void on_error_goto_dlcx(struct mgcp_ctx *mgcp_ctx)
{
	/* This function forces the FSM into the DLCX phase. The FSM will just
	 * behave like the call were ended normally. */

	struct osmo_fsm_inst *fi;
	struct osmo_bsc_sccp_con *conn;

	OSMO_ASSERT(mgcp_ctx);
	conn = mgcp_ctx->conn;
	OSMO_ASSERT(conn);

	fi = mgcp_ctx->fsm;
	OSMO_ASSERT(fi);

	LOGP(DMSC, LOGL_NOTICE, "MGCPGW: (%s) fsm-state: %s\n",
	     mgcp_ctx->name, get_value_string(fsm_state_names, fi->state));

	LOGP(DMGCP, LOGL_ERROR,
	     "(%s) MGCPGW error proceeding request, graceful shutdown...\n",
	     mgcp_ctx->name);

	/* Set the VM into the state where it waits for the call end */
	osmo_fsm_inst_state_chg(fi, ST_WAIT_END, 0, 0);

	/* Simulate the call end by sending a teardown event, so that
	 * the FSM proceeds directly with the DLCX */
	osmo_fsm_inst_dispatch(mgcp_ctx->fsm, EV_TEARDOWN, mgcp_ctx);
}

/* Forward declaration to keep the function in logical order */
static void crcx_for_bts_resp_cb(struct mgcp_response *r, void *priv);

/* Callback for ST_INIT: startup state machine send out CRCX for BTS side */
static void fsm_init_cb(struct osmo_fsm_inst *fi, uint32_t event, void *data)
{
	struct mgcp_ctx *mgcp_ctx = (struct mgcp_ctx *)data;
	struct gsm_network *network;
	struct osmo_bsc_sccp_con *conn;
	struct msgb *msg;
	struct mgcp_msg mgcp_msg;
	struct mgcp_client *mgcp;
	uint16_t rtp_endpoint;
	unsigned int call_id;

	OSMO_ASSERT(mgcp_ctx);
	network = mgcp_ctx->network;
	OSMO_ASSERT(network);
	conn = mgcp_ctx->conn;
	OSMO_ASSERT(conn);
	mgcp = network->mgw.client;
	OSMO_ASSERT(mgcp);

	LOGP(DMSC, LOGL_NOTICE,
	     "MGCPGW: (%s) fsm-state: %s, fsm-event: %s\n",
	     mgcp_ctx->name, get_value_string(fsm_state_names, fi->state),
	     get_value_string(fsm_evt_names, event));

	rtp_endpoint = mgcp_client_next_endpoint(mgcp);
	conn->rtp_endpoint = rtp_endpoint;
	call_id = conn->conn_id;

	LOGP(DMGCP, LOGL_NOTICE,
	     "MGCPGW: (%s) creating connection for the BTS side on "
	     "MGCPGW endpoint:%x...\n", mgcp_ctx->name, rtp_endpoint);

	/* Generate MGCP message string */
	mgcp_msg.verb = MGCP_VERB_CRCX;
	mgcp_msg.presence =
	    (MGCP_MSG_PRESENCE_ENDPOINT | MGCP_MSG_PRESENCE_CALL_ID |
	     MGCP_MSG_PRESENCE_CONN_ID | MGCP_MSG_PRESENCE_CONN_MODE);
	snprintf(mgcp_msg.endpoint, MGCP_ENDPOINT_MAXLEN, MGCP_ENDPOINT_FORMAT,
		 rtp_endpoint);
	mgcp_msg.call_id = call_id;
	mgcp_msg.conn_id = CONN_ID_BTS;
	mgcp_msg.conn_mode = MGCP_CONN_LOOPBACK;
	msg = mgcp_msg_gen(mgcp, &mgcp_msg);
	OSMO_ASSERT(msg);

	/* Note: if transmitting fails, receiving will also fail which eventually
	 * will cause the error be handled by the timeout callback */
	mgcp_client_tx(mgcp, msg, crcx_for_bts_resp_cb, mgcp_ctx);

	osmo_fsm_inst_state_chg(mgcp_ctx->fsm, ST_CRCX_BTS, MGCP_MGW_TIMEOUT,
				MGCP_MGW_TIMEOUT_TIMER_NO);
}

/* Callback for MGCP-Client: handle response for BTS associated CRCX */
static void crcx_for_bts_resp_cb(struct mgcp_response *r, void *priv)
{
	struct mgcp_ctx *mgcp_ctx = (struct mgcp_ctx *)priv;
	int rc;
	struct osmo_bsc_sccp_con *conn;

	OSMO_ASSERT(mgcp_ctx);
	conn = mgcp_ctx->conn;
	OSMO_ASSERT(conn);

	if (r->head.response_code != 200) {
		LOGP(DMGCP, LOGL_ERROR,
		     "MGCPGW: (%s) CRCX response yields error: %d %s\n",
		     mgcp_ctx->name, r->head.response_code, r->head.comment);
		on_error_goto_dlcx(mgcp_ctx);
		return;
	}

	rc = mgcp_response_parse_params(r);
	if (rc) {
		LOGP(DMGCP, LOGL_ERROR,
		     "MGCPGW: (%s) Cannot parse CRCX response\n",
		     mgcp_ctx->name);
		on_error_goto_dlcx(mgcp_ctx);
		return;
	}

	LOGP(DMGCP, LOGL_DEBUG,
	     "MGCPGW: (%s) CRCX responded with address %s:%u\n",
	     mgcp_ctx->name, r->audio_ip, r->audio_port);

	/* Set the connection details in the conn struct. The code that
	 * controls the BTS via RSL will take these values and signal them
	 * to the BTS via RSL/IPACC */
	conn->rtp_port = r->audio_port;
	conn->rtp_ip = osmo_ntohl(inet_addr(r->audio_ip));

	/* Notify the FSM that we got the response. */
	osmo_fsm_inst_dispatch(mgcp_ctx->fsm, EV_CRCX_BTS_RESP, mgcp_ctx);
}

/* Callback for ST_CRCX_BTS: An mgcp response has been received, proceed
 * with the assignment request */
static void fsm_proc_assignmnent_req_cb(struct osmo_fsm_inst *fi,
					uint32_t event, void *data)
{
	struct mgcp_ctx *mgcp_ctx = (struct mgcp_ctx *)data;
	struct osmo_bsc_sccp_con *conn;
	int chan_mode;
	int full_rate;
	int rc;

	OSMO_ASSERT(mgcp_ctx);
	conn = mgcp_ctx->conn;
	OSMO_ASSERT(conn);

	LOGP(DMSC, LOGL_NOTICE,
	     "MGCPGW: (%s) fsm-state: %s, fsm-event: %s\n",
	     mgcp_ctx->name, get_value_string(fsm_state_names, fi->state),
	     get_value_string(fsm_evt_names, event));

	/* Bail on teardown */
	if (event == EV_TEARDOWN) {
		on_error_goto_dlcx(mgcp_ctx);
		return;
	}

	OSMO_ASSERT(conn->conn);
	chan_mode = mgcp_ctx->chan_mode;
	full_rate = mgcp_ctx->full_rate;

	LOGP(DMGCP, LOGL_NOTICE,
	     "MGCPGW: (%s) MGCPGW proceeding assignment request...\n",
	     mgcp_ctx->name);
	rc = gsm0808_assign_req(conn->conn, chan_mode, full_rate);

	if (rc < 0) {
		on_error_goto_dlcx(mgcp_ctx);
		return;
	}

	osmo_fsm_inst_state_chg(fi, ST_WAIT_BSS, MGCP_BSS_TIMEOUT,
				MGCP_BSS_TIMEOUT_TIMER_NO);
}

/* Forward declaration to keep the function in logical order */
static void mdcx_for_bts_resp_cb(struct mgcp_response *r, void *priv);

/* Callback for ST_WAIT_BSS: When the BSS has completed the assignment,
 * proceed with updating the connection for the BTS side */
static void fsm_mdcx_bts_cb(struct osmo_fsm_inst *fi, uint32_t event,
			    void *data)
{
	struct mgcp_ctx *mgcp_ctx = (struct mgcp_ctx *)data;
	struct gsm_network *network;
	struct osmo_bsc_sccp_con *conn;
	struct gsm_lchan *lchan;
	struct msgb *msg;
	struct mgcp_msg mgcp_msg;
	struct mgcp_client *mgcp;
	uint16_t rtp_endpoint;
	unsigned int call_id;
	struct in_addr addr;

	OSMO_ASSERT(mgcp_ctx);
	conn = mgcp_ctx->conn;
	OSMO_ASSERT(conn);

	LOGP(DMSC, LOGL_NOTICE,
	     "MGCPGW: (%s) fsm-state: %s, fsm-event: %s\n",
	     mgcp_ctx->name, get_value_string(fsm_state_names, fi->state),
	     get_value_string(fsm_evt_names, event));

	/* Bail on teardown */
	if (event == EV_TEARDOWN) {
		on_error_goto_dlcx(mgcp_ctx);
		return;
	}

	network = mgcp_ctx->network;
	OSMO_ASSERT(network);
	mgcp = network->mgw.client;
	OSMO_ASSERT(mgcp);
	lchan = mgcp_ctx->lchan;
	OSMO_ASSERT(lchan);

	LOGP(DMGCP, LOGL_NOTICE,
	     "MGCPGW: (%s) MGCPGW BSS has completed the assignment, now prceed with MDCX towards BTS...\n",
	     mgcp_ctx->name);

	rtp_endpoint = conn->rtp_endpoint;
	call_id = conn->conn_id;

	LOGP(DMGCP, LOGL_NOTICE,
	     "MGCPGW: (%s) completing connection for the BTS side on "
	     "MGCPGW endpoint:%x...\n", mgcp_ctx->name, rtp_endpoint);

	addr.s_addr = osmo_ntohl(lchan->abis_ip.bound_ip);
	LOGP(DMGCP, LOGL_NOTICE,
	     "MGCPGW: (%s) BTS expects RTP input on address %s:%u\n",
	     mgcp_ctx->name, inet_ntoa(addr), lchan->abis_ip.bound_port);

	/* Generate MGCP message string */
	mgcp_msg.verb = MGCP_VERB_MDCX;
	mgcp_msg.presence =
	    (MGCP_MSG_PRESENCE_ENDPOINT | MGCP_MSG_PRESENCE_CALL_ID |
	     MGCP_MSG_PRESENCE_CONN_ID | MGCP_MSG_PRESENCE_CONN_MODE |
	     MGCP_MSG_PRESENCE_AUDIO_IP | MGCP_MSG_PRESENCE_AUDIO_PORT);
	snprintf(mgcp_msg.endpoint, sizeof(mgcp_msg.endpoint),
		 MGCP_ENDPOINT_FORMAT, rtp_endpoint);
	mgcp_msg.call_id = call_id;
	mgcp_msg.conn_id = CONN_ID_BTS;
	mgcp_msg.conn_mode = MGCP_CONN_RECV_SEND;
	mgcp_msg.audio_ip = inet_ntoa(addr);
	mgcp_msg.audio_port = lchan->abis_ip.bound_port;
	msg = mgcp_msg_gen(mgcp, &mgcp_msg);
	OSMO_ASSERT(msg);

	/* See note in fsm_init_cb() */
	mgcp_client_tx(mgcp, msg, mdcx_for_bts_resp_cb, mgcp_ctx);

	osmo_fsm_inst_state_chg(mgcp_ctx->fsm, ST_MDCX_BTS, MGCP_MGW_TIMEOUT,
				MGCP_MGW_TIMEOUT_TIMER_NO);
}

/* Callback for MGCP-Client: handle response for BTS associated MDCX */
static void mdcx_for_bts_resp_cb(struct mgcp_response *r, void *priv)
{
	struct mgcp_ctx *mgcp_ctx = (struct mgcp_ctx *)priv;
	int rc;
	struct in_addr addr;
	struct osmo_bsc_sccp_con *conn;
	struct gsm_lchan *lchan;

	OSMO_ASSERT(mgcp_ctx);
	conn = mgcp_ctx->conn;
	OSMO_ASSERT(conn);
	lchan = mgcp_ctx->lchan;
	OSMO_ASSERT(lchan);

	if (r->head.response_code != 200) {
		LOGP(DMGCP, LOGL_ERROR,
		     "MGCPGW: (%s) MDCX response yields error: %d %s\n",
		     mgcp_ctx->name, r->head.response_code, r->head.comment);
		on_error_goto_dlcx(mgcp_ctx);
		return;
	}

	rc = mgcp_response_parse_params(r);
	if (rc) {
		LOGP(DMGCP, LOGL_ERROR,
		     "MGCPGW: (%s) Cannot parse MDCX response\n",
		     mgcp_ctx->name);
		on_error_goto_dlcx(mgcp_ctx);
		return;
	}

	LOGP(DMGCP, LOGL_DEBUG,
	     "MGCPGW: (%s) MDCX responded with address %s:%u\n",
	     mgcp_ctx->name, r->audio_ip, r->audio_port);

	addr.s_addr = lchan->abis_ip.bound_ip;
	LOGP(DMGCP, LOGL_ERROR,
	     "MGCPGW: (%s) MDCX corresponding lchan has been bound to address %s:%u\n",
	     mgcp_ctx->name, inet_ntoa(addr), lchan->abis_ip.bound_port);

	/* Notify the FSM that we got the response. */
	osmo_fsm_inst_dispatch(mgcp_ctx->fsm, EV_MDCX_BTS_RESP, mgcp_ctx);
}

/* Forward declaration to keep the function in logical order */
static void crcx_for_net_resp_cb(struct mgcp_response *r, void *priv);

/* Callback for ST_MDCX_BTS: An mgcp response has been received, proceed... */
static void fsm_crcx_net_cb(struct osmo_fsm_inst *fi, uint32_t event,
			    void *data)
{
	struct mgcp_ctx *mgcp_ctx = (struct mgcp_ctx *)data;
	struct gsm_network *network;
	struct osmo_bsc_sccp_con *conn;
	struct msgb *msg;
	struct mgcp_msg mgcp_msg;
	struct mgcp_client *mgcp;
	uint16_t rtp_endpoint;
	unsigned int call_id;
	struct sockaddr_in *sin;
	char *addr;
	uint16_t port;

	OSMO_ASSERT(mgcp_ctx);
	network = mgcp_ctx->network;
	OSMO_ASSERT(network);
	conn = mgcp_ctx->conn;
	OSMO_ASSERT(conn);
	mgcp = network->mgw.client;
	OSMO_ASSERT(mgcp);

	LOGP(DMSC, LOGL_NOTICE,
	     "MGCPGW: (%s) fsm-state: %s, fsm-event: %s\n",
	     mgcp_ctx->name, get_value_string(fsm_state_names, fi->state),
	     get_value_string(fsm_evt_names, event));

	rtp_endpoint = conn->rtp_endpoint;
	call_id = conn->conn_id;

	LOGP(DMGCP, LOGL_NOTICE,
	     "MGCPGW: (%s) creating connection for the NET side on "
	     "MGCPGW endpoint:%x...\n", mgcp_ctx->name, rtp_endpoint);

	/* Currently we only have support for IPv4 in our MGCP software, the
	 * AoIP part is ready to support IPv6 in theory, because the IE
	 * parser/generator uses sockaddr_storage for the AoIP transport
	 * identifier. However, the MGCP-GW does not support IPv6 yet. This is
	 * why we stop here in case some MSC tries to signal IPv6 AoIP
	 * transport identifiers */
	if (conn->aoip_rtp_addr_remote.ss_family != AF_INET) {
		LOGP(DMGCP, LOGL_ERROR,
		     "MGCPGW (%s) endpoint:%x MSC uses unsupported address format in AoIP transport identifier -- aborting...\n",
		     mgcp_ctx->name, rtp_endpoint);
		on_error_goto_dlcx(mgcp_ctx);
		return;
	}

	sin = (struct sockaddr_in *)&conn->aoip_rtp_addr_remote;
	addr = inet_ntoa(sin->sin_addr);
	port = osmo_ntohs(sin->sin_port);
	LOGP(DMGCP, LOGL_NOTICE,
	     "MGCPGW: (%s) MSC expects RTP input on address %s:%u\n",
	     mgcp_ctx->name, addr, port);

	/* Generate MGCP message string */
	mgcp_msg.verb = MGCP_VERB_CRCX;
	mgcp_msg.presence =
	    (MGCP_MSG_PRESENCE_ENDPOINT | MGCP_MSG_PRESENCE_CALL_ID |
	     MGCP_MSG_PRESENCE_CONN_ID | MGCP_MSG_PRESENCE_CONN_MODE |
	     MGCP_MSG_PRESENCE_AUDIO_IP | MGCP_MSG_PRESENCE_AUDIO_PORT);
	snprintf(mgcp_msg.endpoint, sizeof(mgcp_msg.endpoint),
		 MGCP_ENDPOINT_FORMAT, rtp_endpoint);
	mgcp_msg.call_id = call_id;
	mgcp_msg.conn_id = CONN_ID_NET;
	mgcp_msg.conn_mode = MGCP_CONN_RECV_SEND;
	mgcp_msg.audio_ip = addr;
	mgcp_msg.audio_port = port;
	msg = mgcp_msg_gen(mgcp, &mgcp_msg);
	OSMO_ASSERT(msg);

	/* See note in fsm_init_cb() */
	mgcp_client_tx(mgcp, msg, crcx_for_net_resp_cb, mgcp_ctx);

	osmo_fsm_inst_state_chg(mgcp_ctx->fsm, ST_CRCX_NET, MGCP_MGW_TIMEOUT,
				MGCP_MGW_TIMEOUT_TIMER_NO);
}

/* Callback for MGCP-Client: handle response for NET associated CRCX */
static void crcx_for_net_resp_cb(struct mgcp_response *r, void *priv)
{
	struct mgcp_ctx *mgcp_ctx = (struct mgcp_ctx *)priv;
	int rc;
	struct osmo_bsc_sccp_con *conn;
	struct gsm_lchan *lchan;
	struct sockaddr_in *sin;

	OSMO_ASSERT(mgcp_ctx);
	conn = mgcp_ctx->conn;
	OSMO_ASSERT(conn);
	lchan = mgcp_ctx->lchan;
	OSMO_ASSERT(lchan);

	if (r->head.response_code != 200) {
		LOGP(DMGCP, LOGL_ERROR,
		     "MGCPGW: (%s) CRCX response yields error: %d %s\n",
		     mgcp_ctx->name, r->head.response_code, r->head.comment);
		on_error_goto_dlcx(mgcp_ctx);
		return;
	}

	rc = mgcp_response_parse_params(r);
	if (rc) {
		LOGP(DMGCP, LOGL_ERROR,
		     "MGCPGW: (%s) Cannot parse CRCX response\n",
		     mgcp_ctx->name);
		on_error_goto_dlcx(mgcp_ctx);
		return;
	}

	LOGP(DMGCP, LOGL_DEBUG,
	     "MGCPGW: (%s) CRCX responded with address %s:%u\n",
	     mgcp_ctx->name, r->audio_ip, r->audio_port);

	/* Store address */
	sin = (struct sockaddr_in *)&conn->aoip_rtp_addr_local;
	sin->sin_family = AF_INET;
	sin->sin_addr.s_addr = inet_addr(r->audio_ip);
	sin->sin_port = osmo_ntohs(r->audio_port);

	/* Notify the FSM that we got the response. */
	osmo_fsm_inst_dispatch(mgcp_ctx->fsm, EV_CRCX_NET_RESP, mgcp_ctx);
}

/* Callback for ST_CRCX_NET: Do nothing, just wait until the call ends... */
static void fsm_wait_end_cb(struct osmo_fsm_inst *fi, uint32_t event,
			    void *data)
{
	struct mgcp_ctx *mgcp_ctx = (struct mgcp_ctx *)data;
	struct gsm_lchan *lchan;
	struct osmo_bsc_sccp_con *conn;

	OSMO_ASSERT(mgcp_ctx);
	conn = mgcp_ctx->conn;
	OSMO_ASSERT(conn);

	LOGP(DMSC, LOGL_NOTICE,
	     "MGCPGW: (%s) fsm-state: %s, fsm-event: %s\n",
	     mgcp_ctx->name, get_value_string(fsm_state_names, fi->state),
	     get_value_string(fsm_evt_names, event));

	/* Bail on teardown */
	if (event == EV_TEARDOWN) {
		on_error_goto_dlcx(mgcp_ctx);
		return;
	}

	lchan = mgcp_ctx->lchan;
	OSMO_ASSERT(lchan);

	/* Send assignment completion message via AoIP, this will complete
	 * the circuit. The message will also contain the port and IP-Address
	 * where the MGCPGW expects the RTP input from the MSC side */
	bssmap_send_aoip_ass_compl(lchan);

	LOGP(DMGCP, LOGL_NOTICE,
	     "MGCPGW: (%s) call in progress, waiting for call end...\n",
	     mgcp_ctx->name);

	osmo_fsm_inst_state_chg(mgcp_ctx->fsm, ST_WAIT_END, 0, 0);
}

/* Forward declaration to keep the function in logical order */
static void dlcx_for_all_resp_cb(struct mgcp_response *r, void *priv);

/* Callback for ST_WAIT_END: Remove connection for the BTS and NET side. */
static void fsm_dlcx_all_cb(struct osmo_fsm_inst *fi, uint32_t event,
			    void *data)
{
	struct mgcp_ctx *mgcp_ctx = (struct mgcp_ctx *)data;
	struct gsm_network *network;
	struct osmo_bsc_sccp_con *conn;
	struct msgb *msg;
	struct mgcp_msg mgcp_msg;
	struct mgcp_client *mgcp;
	uint16_t rtp_endpoint;
	unsigned int call_id;

	OSMO_ASSERT(mgcp_ctx);
	network = mgcp_ctx->network;
	OSMO_ASSERT(network);
	conn = mgcp_ctx->conn;
	OSMO_ASSERT(conn);
	mgcp = network->mgw.client;
	OSMO_ASSERT(mgcp);

	LOGP(DMSC, LOGL_NOTICE,
	     "MGCPGW: (%s) fsm-state: %s, fsm-event: %s\n",
	     mgcp_ctx->name, get_value_string(fsm_state_names, fi->state),
	     get_value_string(fsm_evt_names, event));

	rtp_endpoint = conn->rtp_endpoint;
	call_id = conn->conn_id;

	LOGP(DMGCP, LOGL_NOTICE,
	     "MGCPGW: (%s) removing connection for the BTS and NET side on "
	     "MGCPGW endpoint:%x...\n", mgcp_ctx->name, rtp_endpoint);

	/* We now relase the endpoint back to the pool in order to allow
	 * other connections to use this endpoint */
	mgcp_client_release_endpoint(rtp_endpoint, mgcp);

	/* Generate MGCP message string */
	mgcp_msg.verb = MGCP_VERB_DLCX;
	mgcp_msg.presence =
	    (MGCP_MSG_PRESENCE_ENDPOINT | MGCP_MSG_PRESENCE_CALL_ID);
	snprintf(mgcp_msg.endpoint, sizeof(mgcp_msg.endpoint),
		 MGCP_ENDPOINT_FORMAT, rtp_endpoint);
	mgcp_msg.call_id = call_id;
	msg = mgcp_msg_gen(mgcp, &mgcp_msg);
	OSMO_ASSERT(msg);

	/* See note in fsm_init_cb() */
	mgcp_client_tx(mgcp, msg, dlcx_for_all_resp_cb, mgcp_ctx);

	osmo_fsm_inst_state_chg(mgcp_ctx->fsm, ST_DLCX_ALL, MGCP_MGW_TIMEOUT,
				MGCP_MGW_TIMEOUT_TIMER_NO);
}

/* Callback for MGCP-Client: handle response for NET associated CRCX */
static void dlcx_for_all_resp_cb(struct mgcp_response *r, void *priv)
{
	struct mgcp_ctx *mgcp_ctx = (struct mgcp_ctx *)priv;
	struct gsm_network *network;
	struct osmo_bsc_sccp_con *conn;
	struct mgcp_client *mgcp;

	OSMO_ASSERT(mgcp_ctx);
	network = mgcp_ctx->network;
	OSMO_ASSERT(network);
	conn = mgcp_ctx->conn;
	OSMO_ASSERT(conn);
	mgcp = network->mgw.client;
	OSMO_ASSERT(mgcp);

	/* Note: We check the return code, but in case of an error there is
	 * not much that can be done to recover. However, at least we tryed
	 * to remove the connection (if there was even any) */
	if (r->head.response_code != 200) {
		LOGP(DMGCP, LOGL_ERROR,
		     "MGCPGW: (%s) DLCX response yields error: %d %s\n",
		     mgcp_ctx->name, r->head.response_code, r->head.comment);
	}

	/* Notify the FSM that we got the response. */
	osmo_fsm_inst_dispatch(mgcp_ctx->fsm, EV_DLCX_ALL_RESP, mgcp_ctx);
}

/* Callback for ST_DLCX_ALL: Terminate the state machine */
static void fsm_halt_cb(struct osmo_fsm_inst *fi, uint32_t event, void *data)
{
	struct mgcp_ctx *mgcp_ctx = (struct mgcp_ctx *)data;
	struct osmo_bsc_sccp_con *conn;

	OSMO_ASSERT(mgcp_ctx);
	conn = mgcp_ctx->conn;
	OSMO_ASSERT(conn);

	LOGP(DMSC, LOGL_NOTICE,
	     "MGCPGW: (%s) fsm-state: %s, fsm-event: %s\n",
	     mgcp_ctx->name, get_value_string(fsm_state_names, fi->state),
	     get_value_string(fsm_evt_names, event));

	LOGP(DMGCP, LOGL_NOTICE,
	     "MGCPGW: (%s) timeout (T%i) in state %s, state machine halted\n",
	     mgcp_ctx->name, fi->T, get_value_string(fsm_state_names,
						     fi->state));

	/* Send pending sigtran message */
	if (mgcp_ctx->resp) {
		osmo_bsc_sigtran_send(conn, mgcp_ctx->resp);
		mgcp_ctx->resp = NULL;
	}

	/* Destroy the state machine and all context information */
	osmo_fsm_inst_free(mgcp_ctx->fsm);
	memset(mgcp_ctx, 0, sizeof(*mgcp_ctx));
	talloc_free(mgcp_ctx);
}

/* Timer callback to shut down in case of connectivity problems */
static int fsm_timeout_cb(struct osmo_fsm_inst *fi)
{
	struct mgcp_ctx *mgcp_ctx = (struct mgcp_ctx *)fi->priv;
	struct gsm_network *network;
	struct osmo_bsc_sccp_con *conn;
	struct mgcp_client *mgcp;

	OSMO_ASSERT(mgcp_ctx);
	network = mgcp_ctx->network;
	OSMO_ASSERT(mgcp_ctx);
	conn = mgcp_ctx->conn;
	OSMO_ASSERT(conn);
	mgcp = network->mgw.client;
	OSMO_ASSERT(mgcp);

	LOGP(DMGCP, LOGL_ERROR,
	     "MGCPGW: (%s) timeout (T%i) in state %s, attempting graceful teardown...\n",
	     mgcp_ctx->name, fi->T, get_value_string(fsm_state_names,
						     fi->state));

	if (fi->T == MGCP_MGW_TIMEOUT_TIMER_NO) {
		/* Note: We were unable to communicate with the MGCP-GW,
		 * unfortunately there is no meaningful action we can take
		 * now other than giving up. */
		LOGP(DMGCP, LOGL_ERROR,
		     "MGCPGW: (%s) graceful teardown not possible, terminating...\n",
		     mgcp_ctx->name);

		/* At least release the occupied endpoint ID */
		mgcp_client_release_endpoint(conn->rtp_endpoint, mgcp);

		/* Initiate self destruction of the FSM */
		osmo_fsm_inst_state_chg(fi, ST_DLCX_ALL, 0, 0);
		osmo_fsm_inst_dispatch(mgcp_ctx->fsm, EV_TEARDOWN, mgcp_ctx);
	} else if (fi->T == MGCP_BSS_TIMEOUT_TIMER_NO)
		/* Note: If the logic that controls the BSS is unable to
		 * negotiate a connection, we presumably still have a
		 * working connection to the MGCP-GW, we will try to
		 * shut down gracefully */
		on_error_goto_dlcx(mgcp_ctx);
	else {
		/* Note: Ther must not be any unsolicited timers
		 * in this FSM. If so, we have serious problem */
		OSMO_ASSERT(false);
	}

	return 0;
}

static struct osmo_fsm_state fsm_states[] = {

	/* Startup state machine, send CRCX to BTS */
	[ST_INIT] = {
		     .in_event_mask = (1 << EV_INIT),
		     .out_state_mask = (1 << ST_CRCX_BTS),
		     .name = "ST_INIT",
		     .action = fsm_init_cb,
		     },

	/* When the CRCX response for the BTS side is received, then
	 * proceed the assignment */
	[ST_CRCX_BTS] = {
			 .in_event_mask =
			 (1 << EV_TEARDOWN) | (1 << EV_CRCX_BTS_RESP),
			 .out_state_mask =
			 (1 << ST_WAIT_END) | (1 << ST_WAIT_BSS),
			 .name = "ST_CRCX_BTS",
			 .action = fsm_proc_assignmnent_req_cb,
			 },

	/* Wait until the BSS has processed the assignment request,
	 * then send the MDCX command for the BTS side in order to
	 * update the connections with the actual PORT/IP where the
	 * BTS expects the RTP input */
	[ST_WAIT_BSS] = {
			 .in_event_mask =
			 (1 << EV_TEARDOWN) | (1 << EV_ASS_COMPLETE),
			 .out_state_mask =
			 (1 << ST_WAIT_END) | (1 << ST_MDCX_BTS),
			 .name = "ST_WAIT_BSS",
			 .action = fsm_mdcx_bts_cb,
			 },

	/* Wait until the MDCX response for the BTS siede is received, then
	 * directly proceed with sending the CRCX command to connect the
	 * network side. This is done in one phase (no MDCX needed) */
	[ST_MDCX_BTS] = {
			 .in_event_mask =
			 (1 << EV_TEARDOWN) | (1 << EV_MDCX_BTS_RESP),
			 .out_state_mask =
			 (1 << ST_WAIT_END) | (1 << ST_CRCX_NET),
			 .name = "ST_MDCX_BTS",
			 .action = fsm_crcx_net_cb,
			 },

	/* Wait until the CRCX response for the NET side is received. Then
	 * send the assignment complete message via the A-Interface and
	 * enter wait state in order to wait for the end of the call */
	[ST_CRCX_NET] = {
			 .in_event_mask =
			 (1 << EV_TEARDOWN) | (1 << EV_CRCX_NET_RESP),
			 .out_state_mask =
			 (1 << ST_WAIT_END) | (1 << ST_WAIT_END),
			 .name = "ST_CRCX_NET",
			 .action = fsm_wait_end_cb,
			 },

	/* Wait for the ending of the call, this phase may take indefinetly
	 * long. Its up to the subscriber to decide when the call is over. */
	[ST_WAIT_END] = {
			 .in_event_mask = (1 << EV_TEARDOWN),
			 .out_state_mask = (1 << ST_DLCX_ALL),
			 .name = "ST_WAIT_END",
			 .action = fsm_dlcx_all_cb,
			 },

	/* Wait until the MSC confirms that the connections are terminated,
	 * then halt */
	[ST_DLCX_ALL] = {
			 .in_event_mask =
			 (1 << EV_TEARDOWN) | (1 << EV_DLCX_ALL_RESP),
			 .out_state_mask = 0,
			 .name = "ST_DLCX_ALL",
			 .action = fsm_halt_cb,
			 },
};

/* State machine definition */
static struct osmo_fsm fsm = {
	.name = "FSM MGCP",
	.states = fsm_states,
	.num_states = ARRAY_SIZE(fsm_states),
	.log_subsys = DMGCP,
	.timer_cb = fsm_timeout_cb,
};

/* Notify that the a new call begins. This will create a connection for the
 * BTS on the MGCP-GW and set up the port numbers in struct osmo_bsc_sccp_con.
 * After that gsm0808_assign_req() to proceed.
 * Parameter:
 * ctx: talloc context
 * network: associated gsm network
 * conn: associated sccp connection
 * chan_mode: channel mode (system data, passed through)
 * full_rate: full rate flag (system data, passed through)
 * Returns an mgcp_context that contains system data and the OSMO-FSM */
struct mgcp_ctx *mgcp_assignm_req(void *ctx, struct gsm_network *network,
				  struct osmo_bsc_sccp_con *conn, int chan_mode,
				  int full_rate)
{
	struct mgcp_ctx *mgcp_ctx;

	OSMO_ASSERT(network);
	OSMO_ASSERT(conn);

	/* Register the fsm description (if not already done) */
	if (osmo_fsm_find_by_name(fsm.name) != &fsm)
		osmo_fsm_register(&fsm);

	/* Allocate and configure a new fsm instance */
	mgcp_ctx = talloc_zero(ctx, struct mgcp_ctx);
	OSMO_ASSERT(mgcp_ctx);

	snprintf(mgcp_ctx->name, sizeof(mgcp_ctx->name), "MGCP FSM, id=%i",
		 conn->conn_id);
	mgcp_ctx->fsm =
	    osmo_fsm_inst_alloc(&fsm, NULL, ctx, LOGL_DEBUG, "FSM MGCP INST");
	OSMO_ASSERT(mgcp_ctx->fsm);
	mgcp_ctx->fsm->priv = mgcp_ctx;
	LOGP(DMGCP, LOGL_NOTICE, "MGCPGW: (%s) MGCPGW handler fsm created\n",
	     mgcp_ctx->name);
	mgcp_ctx->network = network;
	mgcp_ctx->conn = conn;
	mgcp_ctx->chan_mode = chan_mode;
	mgcp_ctx->full_rate = full_rate;

	/* start state machine */
	osmo_fsm_inst_state_chg(mgcp_ctx->fsm, ST_INIT, 0, 0);
	osmo_fsm_inst_dispatch(mgcp_ctx->fsm, EV_INIT, mgcp_ctx);

	return mgcp_ctx;
}

/* Notify that the call has ended, remove all connections from the MGCP-GW,
 * then send the clear complete message and destroy the FSM instance
 * Parameter:
 * mgcp_ctx: context information (FSM, and pointer to external system data)
 * respmgcp_ctx: pending clear complete message to send via A-Interface */
void mgcp_clear_complete(struct mgcp_ctx *mgcp_ctx, struct msgb *resp)
{
	OSMO_ASSERT(mgcp_ctx);
	OSMO_ASSERT(resp);

	mgcp_ctx->resp = resp;

	osmo_fsm_inst_dispatch(mgcp_ctx->fsm, EV_TEARDOWN, mgcp_ctx);
}

/* Notify that the BSS ready, send the assingnment complete message when the
 * mgcp connection is completed
 * Parameter:
 * mgcp_ctx: context information (FSM, and pointer to external system data)
 * lchan: needed for sending the assignment complete message via A-Interface */
void mgcp_ass_complete(struct mgcp_ctx *mgcp_ctx, struct gsm_lchan *lchan)
{
	OSMO_ASSERT(mgcp_ctx);
	OSMO_ASSERT(lchan);

	mgcp_ctx->lchan = lchan;

	osmo_fsm_inst_dispatch(mgcp_ctx->fsm, EV_ASS_COMPLETE, mgcp_ctx);

	return;
}
