#pragma once

void gsm_net_update_ctype(struct gsm_network *network);
enum gsm_chan_t get_ctype_by_chreq(struct gsm_network *network, uint8_t ra);
int get_reason_by_chreq(uint8_t ra, int neci);
int gsm48_send_rr_release(struct gsm_lchan *lchan);
int send_siemens_mrpci(struct gsm_lchan *lchan,
		       uint8_t *classmark2_lv);
int gsm48_handle_paging_resp(struct gsm_subscriber_connection *conn,
			     struct msgb *msg, struct bsc_subscr *bsub);
int gsm48_send_rr_ciph_mode(struct gsm_lchan *lchan, int want_imeisv);
void gsm48_lchan2chan_desc(struct gsm48_chan_desc *cd,
			   const struct gsm_lchan *lchan);
int gsm48_multirate_config(uint8_t *lv, const struct amr_multirate_conf *mr, const struct amr_mode *modes);
int gsm48_send_ho_cmd(struct gsm_lchan *old_lchan, struct gsm_lchan *new_lchan,
		      uint8_t power_command, uint8_t ho_ref);
int gsm48_send_rr_ass_cmd(struct gsm_lchan *dest_lchan, struct gsm_lchan *lchan, uint8_t power_command);
int gsm48_lchan_modify(struct gsm_lchan *lchan, uint8_t mode);
int gsm48_rx_rr_modif_ack(struct msgb *msg);
int gsm48_parse_meas_rep(struct gsm_meas_rep *rep, struct msgb *msg);
int gsm48_tx_mm_serv_ack(struct gsm_subscriber_connection *conn);
int gsm48_tx_mm_serv_rej(struct gsm_subscriber_connection *conn,
			 enum gsm48_reject_value value);

#define GSM48_ALLOC_SIZE        2048
#define GSM48_ALLOC_HEADROOM    256

static inline struct msgb *gsm48_msgb_alloc_name(const char *name)
{
        return msgb_alloc_headroom(GSM48_ALLOC_SIZE, GSM48_ALLOC_HEADROOM,
                                   name);
}
