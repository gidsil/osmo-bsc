/* OpenBSC VTY common helpers */
/* (C) 2009-2010 by Harald Welte <laforge@gnumonks.org>
 * (C) 2009-2010 by Holger Hans Peter Freyther
 * All Rights Reserved
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

#include <stdlib.h>
#include <string.h>

#include <osmocom/core/talloc.h>

#include <osmocom/bsc/vty.h>
#include <osmocom/bsc/gsm_data.h>
#include <osmocom/bsc/debug.h>
#include <osmocom/bsc/bsc_nat.h>
#include <osmocom/bsc/abis_om2000.h>

#include <osmocom/vty/telnet_interface.h>
#include <osmocom/vty/command.h>
#include <osmocom/vty/buffer.h>
#include <osmocom/vty/vty.h>
#include <osmocom/sigtran/osmo_ss7.h>


int bsc_vty_go_parent(struct vty *vty)
{
	switch (vty->node) {
	case GSMNET_NODE:
		vty->node = CONFIG_NODE;
		vty->index = NULL;
		break;
	case BTS_NODE:
		vty->node = GSMNET_NODE;
		{
			/* set vty->index correctly ! */
			struct gsm_bts *bts = vty->index;
			vty->index = bts->network;
			vty->index_sub = NULL;
		}
		break;
	case TRX_NODE:
		vty->node = BTS_NODE;
		{
			/* set vty->index correctly ! */
			struct gsm_bts_trx *trx = vty->index;
			vty->index = trx->bts;
			vty->index_sub = &trx->bts->description;
		}
		break;
	case TS_NODE:
		vty->node = TRX_NODE;
		{
			/* set vty->index correctly ! */
			struct gsm_bts_trx_ts *ts = vty->index;
			vty->index = ts->trx;
			vty->index_sub = &ts->trx->description;
		}
		break;
	case OML_NODE:
	case OM2K_NODE:
		vty->node = ENABLE_NODE;
		/* NOTE: this only works because it's not part of the config
		 * tree, where outer commands are searched via vty_go_parent()
		 * and only (!) executed when a matching one is found.
		 */
		talloc_free(vty->index);
		vty->index = NULL;
		break;
	case OM2K_CON_GROUP_NODE:
		vty->node = BTS_NODE;
		{
			struct con_group *cg = vty->index;
			struct gsm_bts *bts = cg->bts;
			vty->index = bts;
			vty->index_sub = &bts->description;
		}
		break;
	case NAT_BSC_NODE:
		vty->node = NAT_NODE;
		{
			struct bsc_config *bsc_config = vty->index;
			vty->index = bsc_config->nat;
		}
		break;
	case PGROUP_NODE:
		vty->node = NAT_NODE;
		vty->index = NULL;
		break;
	case TRUNK_NODE:
		vty->node = MGCP_NODE;
		vty->index = NULL;
		break;
	case SMPP_ESME_NODE:
		vty->node = SMPP_NODE;
		vty->index = NULL;
		break;
	case SMPP_NODE:
	case MGCP_NODE:
	case GBPROXY_NODE:
	case SGSN_NODE:
	case NAT_NODE:
	case BSC_NODE:
	case MSC_NODE:
	case MNCC_INT_NODE:
	case NITB_NODE:
		vty->node = CONFIG_NODE;
		vty->index = NULL;
		break;
	case SUBSCR_NODE:
		vty->node = ENABLE_NODE;
		vty->index = NULL;
		break;
	default:
		osmo_ss7_vty_go_parent(vty);
	}

	return vty->node;
}

int bsc_vty_is_config_node(struct vty *vty, int node)
{
	/* Check if libosmo-sccp declares the node in
	 * question as config node */
	if (osmo_ss7_is_config_node(vty, node))
		return 1;

	switch (node) {
	/* add items that are not config */
	case OML_NODE:
	case OM2K_NODE:
	case SUBSCR_NODE:
	case CONFIG_NODE:
		return 0;

	default:
		return 1;
	}
}
