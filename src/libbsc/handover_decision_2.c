/* Handover Decision Algorithm 2 for intra-BSC (inter-BTS) handover, public API for OsmoBSC. */

/* (C) 2017 by sysmocom - s.f.m.c. GmbH <info@sysmocom.de>
 *
 * All Rights Reserved
 *
 * Author: Andreas Eversberg <jolly@eversberg.eu>
 *         Neels Hofmeyr <nhofmeyr@sysmocom.de>
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
 */

#include <stdbool.h>

#include <osmocom/bsc/debug.h>
#include <osmocom/bsc/gsm_data.h>
#include <osmocom/bsc/handover_cfg.h>

#define LOGPHOBTS(bts, level, fmt, args...) \
	LOGP(DHODEC, level, "BTS %u: " fmt, bts->nr, ## args)

static bool ho2_initialized = false;

void handover_decision_2_init()
{
	ho2_initialized = true;
}

void congestion_check_cb(void *arg)
{
	struct gsm_bts *bts = arg;
	LOGPHOBTS(bts, LOGL_DEBUG, "Congestion check\n");
}

void handover_decision_2_reinit_congestion_timer(struct gsm_bts *bts)
{
	int congestion_check_interval_s;
	int interval_spread_ms = (bts->nr * 10) % 1000;
	bool was_active;

	/* Don't setup timers from VTY config parsing before the main program has actually initialized the data
	 * structures. */
	if (!ho2_initialized)
		return;

	was_active = bts->ho_congestion_check_timer.active;
	if (was_active)
		osmo_timer_del(&bts->ho_congestion_check_timer);

	congestion_check_interval_s = ho_get_congestion_check_interval(bts->ho);
	if (congestion_check_interval_s < 1) {
		if (was_active)
			LOGPHOBTS(bts, LOGL_NOTICE, "Disabling congestion check\n");
		return;
	}

	LOGPHOBTS(bts, LOGL_NOTICE, "%s periodical congestion check"
		  " every %d seconds (+ 0.%03d seconds to spread BTS checks)\n",
		  was_active? "Starting" : "Restarting",
		  congestion_check_interval_s, interval_spread_ms);

	osmo_timer_setup(&bts->ho_congestion_check_timer,
			 congestion_check_cb, bts);
	osmo_timer_schedule(&bts->ho_congestion_check_timer,
			    congestion_check_interval_s,
			    interval_spread_ms);
}
