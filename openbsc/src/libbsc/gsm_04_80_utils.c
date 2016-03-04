/* OpenBSC utility functions for 3GPP TS 04.80 */

/* (C) 2016 by sysmocom s.m.f.c. GmbH <info@sysmocom.de>
 *
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

#include <openbsc/gsm_04_80.h>
#include <openbsc/bsc_api.h>

int gsm0480_send_ussdNotify(struct gsm_subscriber_connection *conn, int level, const char *text)
{
	struct msgb *msg = gsm0480_gen_ussdNotify(level, text);
	if (!msg)
		return -1;
	return gsm0808_submit_dtap(conn, msg, 0, 0);
}

int gsm0480_send_releaseComplete(struct gsm_subscriber_connection *conn)
{
	struct msgb *msg = gsm0480_gen_releaseComplete();
	if (!msg)
		return -1;
	return gsm0808_submit_dtap(conn, msg, 0, 0);
}
