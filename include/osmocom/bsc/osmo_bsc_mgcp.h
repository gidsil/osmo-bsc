/* (C) 2017 by sysmocom - s.f.m.c. GmbH <info@sysmocom.de>
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

#pragma once

#include <osmocom/mgcp_client/mgcp_common.h>
#include <osmocom/mgcp_client/mgcp_client.h>

/* MGCP state handler context. This context information stores all information
 * to handle the direction of the RTP streams via MGCP. There is one instance
 * of this context struct per subscriber connection.
 * (see also struct osmo_bsc_sccp_con) */
struct mgcp_ctx {
	/* FSM instance, which handles the connection switching procedure */
	struct osmo_fsm_inst *fsm;

	/* RTP endpoint number. This number number identifies the endpoint
	 * on the MGW on which the BTS and NET connection is created. This
	 * endpoint number is assigned and released automatically. */
	uint16_t rtp_endpoint;

	/* RTP connection identifiers */
	char conn_id_bts[MGCP_CONN_ID_LENGTH];
	char conn_id_net[MGCP_CONN_ID_LENGTH];

	/* Copy of the pointer and the data with context information
	 * needed to process the AoIP and MGCP requests (system data) */
	struct mgcp_client *mgcp;
	struct gsm_subscriber_connection *conn;
	enum gsm48_chan_mode chan_mode;
	bool full_rate;
	struct gsm_lchan *lchan;
	struct gsm_lchan *ho_lchan;
	struct msgb *resp;
	mgcp_trans_id_t mgw_pending_trans;
};

struct mgcp_ctx *mgcp_assignm_req(void *ctx, struct mgcp_client *mgcp,
				  struct gsm_subscriber_connection *conn,
				  enum gsm48_chan_mode chan_mode, bool full_rate);
void mgcp_clear_complete(struct mgcp_ctx *mgcp_ctx, struct msgb *resp);
void mgcp_ass_complete(struct mgcp_ctx *mgcp_ctx, struct gsm_lchan *lchan);
void mgcp_handover(struct mgcp_ctx *mgcp_ctx, struct gsm_lchan *ho_lchan);
void mgcp_free_ctx(struct mgcp_ctx *mgcp_ctx);
