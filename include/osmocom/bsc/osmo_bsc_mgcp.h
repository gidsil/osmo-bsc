/* (C) 2017 by Sysmocom s.f.m.c. GmbH
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

/* MGCP state handler context (fsm etc..) */
struct mgcp_ctx {
	/* FSM instance, which handles the connection switching procedure */
	struct osmo_fsm_inst *fsm;

	/* A human readable name to display in the logs */
	char name[256];

	/* Copy of the pointer and the data with context information
	 * needed to process the AoIP and MGCP requests (system data) */
	struct gsm_network *network;
	struct osmo_bsc_sccp_con *conn;
	int chan_mode;
	int full_rate;
	struct gsm_lchan *lchan;
	struct msgb *resp;
};

struct mgcp_ctx *mgcp_assignm_req(void *ctx, struct gsm_network *network,
				  struct osmo_bsc_sccp_con *conn, int chan_mode,
				  int full_rate);
void mgcp_clear_complete(struct mgcp_ctx *mgcp_ctx, struct msgb *resp);
void mgcp_ass_complete(struct mgcp_ctx *mgcp_ctx, struct gsm_lchan *lchan);
