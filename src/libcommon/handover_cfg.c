/* OsmoBSC handover configuration implementation */
/* (C) 2017 by sysmocom - s.f.m.c. GmbH <info@sysmocom.de>
 * (C) 2009-2010 by Harald Welte <laforge@gnumonks.org>
 * All Rights Reserved
 *
 * Author: Neels Hofmeyr <nhofmeyr@sysmocom.de>
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

#include <stdbool.h>
#include <talloc.h>

#include <osmocom/bsc/debug.h>

#include <osmocom/bsc/vty.h>
#include <osmocom/bsc/handover_decision_2.h>
#include <osmocom/bsc/handover_cfg.h>
#include <osmocom/bsc/gsm_data.h>

struct handover_cfg {
	struct handover_cfg *higher_level_cfg;

	void *ctx;
	int ctx_type;

#define HO_CFG_ONE_MEMBER(TYPE, NAME, DEFAULT_VAL, ON_CHANGE, VTY1, VTY2, VTY3, VTY4, VTY5, VTY6) \
	TYPE NAME; \
	bool has_##NAME;

	HO_CFG_ALL_MEMBERS
#undef HO_CFG_ONE_MEMBER
};

struct handover_cfg *ho_cfg_init(void *ctx, enum handover_cfg_ctx_type ctx_type,
				 struct handover_cfg *higher_level_cfg)
{
	struct handover_cfg *ho = talloc_zero(ctx, struct handover_cfg);
	OSMO_ASSERT(ho);
	ho->higher_level_cfg = higher_level_cfg;
	ho->ctx = ctx;
	ho->ctx_type = ctx_type;
	return ho;
}

static void on_change_congestion_check_interval(void *ctx, enum handover_cfg_ctx_type ctx_type)
{
	struct gsm_network *net = NULL;
	struct gsm_bts *bts;

	switch (ctx_type) {
	default:
		LOGP(DHODEC, LOGL_ERROR, "Invalid HO config context type: %d\n", ctx_type);
		return;
	case HO_CFG_CTX_BTS:
		handover_decision_2_reinit_congestion_timer((struct gsm_bts*)ctx);
		return;
	case HO_CFG_CTX_NET:
		/* Restart HO timers for all BTS */
		net = ctx;
		break;
	}

	llist_for_each_entry(bts, &net->bts_list, list) {
		/* If the BTS has its own value, the network level config cannot change it. */
		if (!ho_isset_congestion_check_interval(bts->ho))
			continue;
		handover_decision_2_reinit_congestion_timer(bts);
	}
}

#define HO_CFG_ONE_MEMBER(TYPE, NAME, DEFAULT_VAL, ON_CHANGE, VTY1, VTY2, VTY_ARG_EVAL, VTY4, VTY5, VTY6) \
TYPE ho_get_##NAME(struct handover_cfg *ho) \
{ \
	if (ho->has_##NAME) \
		return ho->NAME; \
	if (ho->higher_level_cfg) \
		return ho_get_##NAME(ho->higher_level_cfg); \
	return VTY_ARG_EVAL(#DEFAULT_VAL); \
} \
\
void ho_set_##NAME(struct handover_cfg *ho, TYPE value) \
{ \
	ho_cfg_on_change_cb_t cb = ON_CHANGE; \
	ho->NAME = value; \
	ho->has_##NAME = true; \
	\
	if (cb) \
		cb(ho->ctx, ho->ctx_type); \
} \
\
bool ho_isset_##NAME(struct handover_cfg *ho) \
{ \
	return ho->has_##NAME; \
} \
\
void ho_clear_##NAME(struct handover_cfg *ho) \
{ \
	ho->has_##NAME = false; \
} \
\
bool ho_isset_on_parent_##NAME(struct handover_cfg *ho) \
{ \
	return ho->higher_level_cfg \
		&& (ho_isset_##NAME(ho->higher_level_cfg) \
		    || ho_isset_on_parent_##NAME(ho->higher_level_cfg)); \
}

HO_CFG_ALL_MEMBERS
#undef HO_CFG_ONE_MEMBER
