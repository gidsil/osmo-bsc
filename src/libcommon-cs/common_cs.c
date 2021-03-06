/* Code used by both libbsc and libmsc (common_cs means "BSC or MSC").
 *
 * (C) 2016 by sysmocom s.m.f.c. <info@sysmocom.de>
 * (C) 2008-2010 by Harald Welte <laforge@gnumonks.org>
 * (C) 2014 by Holger Hans Peter Freyther
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

#include <stdbool.h>

#include <osmocom/gsm/gsm0480.h>
#include <osmocom/gsm/gsm48.h>

#include <osmocom/bsc/common_cs.h>
#include <osmocom/bsc/gsm_data.h>
#include <osmocom/bsc/gsm_data.h>
#include <osmocom/bsc/gsm_04_08_utils.h>

/* Warning: if bsc_network_init() is not called, some of the members of
 * gsm_network are not initialized properly and must not be used! (In
 * particular the llist heads and stats counters.)
 * The long term aim should be to have entirely separate structs for libbsc and
 * libmsc with some common general items.
 */
struct gsm_network *gsm_network_init(void *ctx,
				     uint16_t country_code,
				     uint16_t network_code)
{
	struct gsm_network *net;

	net = talloc_zero(ctx, struct gsm_network);
	if (!net)
		return NULL;

	net->country_code = country_code;
	net->network_code = network_code;

	/* Use 30 min periodic update interval as sane default */
	net->t3212 = 5;

	INIT_LLIST_HEAD(&net->trans_list);
	INIT_LLIST_HEAD(&net->subscr_conns);

	net->bsc_subscribers = talloc_zero(net, struct llist_head);
	INIT_LLIST_HEAD(net->bsc_subscribers);

	net->dyn_ts_allow_tch_f = true;

	return net;
}

struct msgb *gsm48_create_mm_serv_rej(enum gsm48_reject_value value)
{
	struct msgb *msg;
	struct gsm48_hdr *gh;

	msg = gsm48_msgb_alloc_name("GSM 04.08 SERV REJ");
	if (!msg)
		return NULL;

	gh = (struct gsm48_hdr *) msgb_put(msg, sizeof(*gh) + 1);
	gh->proto_discr = GSM48_PDISC_MM;
	gh->msg_type = GSM48_MT_MM_CM_SERV_REJ;
	gh->data[0] = value;

	return msg;
}

struct msgb *gsm48_create_loc_upd_rej(uint8_t cause)
{
	struct gsm48_hdr *gh;
	struct msgb *msg;

	msg = gsm48_msgb_alloc_name("GSM 04.08 LOC UPD REJ");
	if (!msg)
		return NULL;

	gh = (struct gsm48_hdr *) msgb_put(msg, sizeof(*gh) + 1);
	gh->proto_discr = GSM48_PDISC_MM;
	gh->msg_type = GSM48_MT_MM_LOC_UPD_REJECT;
	gh->data[0] = cause;
	return msg;
}

int gsm48_extract_mi(uint8_t *classmark2_lv, int length, char *mi_string, uint8_t *mi_type)
{
	/* Check the size for the classmark */
	if (length < 1 + *classmark2_lv)
		return -1;

	uint8_t *mi_lv = classmark2_lv + *classmark2_lv + 1;
	if (length < 2 + *classmark2_lv + mi_lv[0])
		return -2;

	*mi_type = mi_lv[1] & GSM_MI_TYPE_MASK;
	return gsm48_mi_to_string(mi_string, GSM48_MI_SIZE, mi_lv+1, *mi_lv);
}

int gsm48_paging_extract_mi(struct gsm48_pag_resp *resp, int length,
			    char *mi_string, uint8_t *mi_type)
{
	static const uint32_t classmark_offset =
		offsetof(struct gsm48_pag_resp, classmark2);
	uint8_t *classmark2_lv = (uint8_t *) &resp->classmark2;
	return gsm48_extract_mi(classmark2_lv, length - classmark_offset,
				mi_string, mi_type);
}
