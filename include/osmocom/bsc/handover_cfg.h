#pragma once

#include <stdbool.h>
#include <string.h>

struct vty;

/* handover_cfg is an opaque struct to manage several levels of configuration. There is an overall handover
 * config on 'network' level and a per-'bts' specific handover config. If the 'bts' level sets no values,
 * the defaults from 'network' level are used implicitly, and changes take effect immediately. */
struct handover_cfg;

struct handover_cfg *ho_cfg_init(void *ctx, struct handover_cfg *higher_level_cfg);

#define HO_CFG_STR_HANDOVER "Handover options\n"
#define HO_CFG_STR_WIN HO_CFG_STR_HANDOVER "Measurement averaging settings\n"
#define HO_CFG_STR_WIN_RXLEV HO_CFG_STR_WIN "Received-Level averaging\n"
#define HO_CFG_STR_WIN_RXQUAL HO_CFG_STR_WIN "Received-Quality averaging\n"
#define HO_CFG_STR_POWER_BUDGET HO_CFG_STR_HANDOVER "Neighbor cell power triggering\n" "Neighbor cell power triggering\n"
#define HO_CFG_STR_AVG_COUNT "Number of values to average over\n"
#define HO_CFG_STR_2 " (HO algo 2 only)\n"
#define HO_CFG_STR_MIN "Minimum Level/Quality thresholds before triggering HO" HO_CFG_STR_2
#define HO_CFG_STR_AFS_BIAS "Configure bias to prefer AFS (AMR on TCH/F) over other codecs" HO_CFG_STR_2
#define HO_CFG_STR_MIN_TCH "Minimum free TCH timeslots before cell is considered congested" HO_CFG_STR_2
#define HO_CFG_STR_PENALTY_TIME "Set penalty times to wait between repeated handovers" HO_CFG_STR_2

#define as_is(x) (x)

static inline bool a2bool(const char *arg)
{
	return (bool)(atoi(arg));
}

static inline int bool2i(bool arg)
{
	return arg? 1 : 0;
}

static inline bool a2tdma(const char *arg)
{
	if (!strcmp(arg, "full"))
		return true;
	return false;
}

static inline const char *tdma2a(bool val)
{
	return val? "full" : "subset";
}

static inline const int a2congestion_check_interval(const char *arg)
{
	if (!strcmp(arg, "disabled"))
		return 0;
	return atoi(arg);
}

static inline const char *congestion_check_interval2a(int val)
{
	static char str[9];
	if (val < 1
	    || snprintf(str, sizeof(str), "%d", val) >= sizeof(str))
		return "disabled";
	return str;
}

/* The HO_CFG_ONE_MEMBER macro gets redefined, depending on whether to define struct members,
 * function declarations or definitions... It is of the format
 *   HO_CFG_ONE_MEMBER(TYPE, NAME, DEFAULT_VAL,
 *                     VTY_CMD, VTY_CMD_ARG, VTY_ARG_EVAL,
 *                     VTY_WRITE_FMT, VTY_WRITE_CONV,
 *                     VTY_DOC)
 * Then using HO_CFG_ALL_MEMBERS can save a lot of code dup in defining API declaration, API
 * definitions, VTY commands and VTY write code. Of course this doesn't prevent us from adding manual
 * members as well, in case future additions don't fit in this scheme.
 *
 * TYPE: a type name like int.
 * NAME: a variable name suitable for a struct member.
 * DEFAULT_VAL: default value, as passed to the VTY, e.g. '0' or 'foo', without quotes.
 * VTY_CMD: a command string for VTY without any arguments.
 * VTY_CMD_ARG: VTY value range like '<0-23>' or 'foo|bar', will become '(VTY_CMD_ARG|default)'.
 * VTY_ARG_EVAL: function name for parsing the VTY arg[0], e.g. 'atoi'.
 * VTY_WRITE_FMT: printf-like string format for vty_out().
 * VTY_WRITE_CONV: function name to convert struct value to VTY_WRITE_FMT, e.g. 'as_is'.
 * VTY_DOC: VTY documentation strings to match VTY_CMD and VTY_CMD_ARGs.
 */
#define HO_CFG_ALL_MEMBERS \
	\
	HO_CFG_ONE_MEMBER(bool, ho_active, 0, \
		"handover", "0|1", a2bool, "%d", bool2i, \
		HO_CFG_STR_HANDOVER \
		"Disable in-call handover\n" \
		"Enable in-call handover\n" \
		"Enable/disable handover: ") \
	\
	HO_CFG_ONE_MEMBER(int, algorithm, 1, \
		"handover algorithm", "1|2", atoi, "%d", as_is, \
		HO_CFG_STR_HANDOVER \
		"Choose algorithm for handover decision\n" \
		"Algorithm 1: trigger handover based on comparing current cell and neighbor RxLev and RxQual," \
		" only.\n" \
		"Algorithm 2: trigger handover on RxLev/RxQual, and also to balance the load across several" \
		" cells. Consider available codecs. Prevent repeated handover by penalty timers.\n") \
	\
	HO_CFG_ONE_MEMBER(unsigned int, rxlev_avg_win, 10, \
		"handover window rxlev averaging", "<1-10>", atoi, "%u", as_is, \
		HO_CFG_STR_WIN_RXLEV \
		"How many RxLev measurements are used for averaging\n" \
		"RxLev averaging: " HO_CFG_STR_AVG_COUNT) \
	\
	HO_CFG_ONE_MEMBER(unsigned int, rxqual_avg_win, 1, \
		"handover window rxqual averaging", "<1-10>", atoi, "%u", as_is, \
		HO_CFG_STR_WIN_RXQUAL \
		"How many RxQual measurements are used for averaging\n" \
		"RxQual averaging: " HO_CFG_STR_AVG_COUNT) \
	\
	HO_CFG_ONE_MEMBER(unsigned int, rxlev_neigh_avg_win, 10, \
		"handover window rxlev neighbor averaging", "<1-10>", atoi, "%u", as_is, \
		HO_CFG_STR_WIN_RXLEV \
		"How many Neighbor RxLev measurements are used for averaging\n" \
		"How many Neighbor RxLev measurements are used for averaging\n" \
		"Neighbor RxLev averaging: " HO_CFG_STR_AVG_COUNT) \
	\
	HO_CFG_ONE_MEMBER(unsigned int, pwr_interval, 6, \
		"handover power budget interval", "<1-99>", atoi, "%u", as_is, \
		HO_CFG_STR_POWER_BUDGET \
		"How often to check for a better cell (SACCH frames)\n" \
		"Check for stronger neighbor every N number of SACCH frames\n") \
	\
	HO_CFG_ONE_MEMBER(unsigned int, pwr_hysteresis, 3, \
		"handover power budget hysteresis", "<0-999>", atoi, "%u", as_is, \
		HO_CFG_STR_POWER_BUDGET \
		"How many dBm stronger must a neighbor be to become a HO candidate\n" \
		"Neighbor's strength difference in dBm\n") \
	\
	HO_CFG_ONE_MEMBER(unsigned int, max_distance, 9999, \
		"handover maximum distance" , "<0-9999>", atoi, "%u", as_is, \
		HO_CFG_STR_HANDOVER \
		"Maximum Timing-Advance value (i.e. MS distance) before triggering HO\n" \
		"Maximum Timing-Advance value (i.e. MS distance) before triggering HO\n" \
		"Maximum Timing-Advance value (i.e. MS distance) before triggering HO\n") \
	\
	HO_CFG_ONE_MEMBER(bool, as_active, 0, \
		"handover assignment", "0|1", a2bool, "%d", bool2i, \
		HO_CFG_STR_HANDOVER \
		"Enable or disable in-call channel re-assignment" HO_CFG_STR_2 \
		"Disable in-call assignment\n" \
		"Enable in-call assignment\n") \
	\
	HO_CFG_ONE_MEMBER(int, congestion_check_interval, 10, \
		"handover congestion-check", "disabled|<1-60>", \
		a2congestion_check_interval, "%s", congestion_check_interval2a, \
		HO_CFG_STR_HANDOVER \
		"Configure congestion check interval" HO_CFG_STR_2 \
		"Disable congestion checking, do not handover based on cell overload\n" \
		"Congestion check interval in seconds\n") \
	\
	HO_CFG_ONE_MEMBER(bool, full_tdma, subset, \
		"handover tdma-measurement", "full|subset", a2tdma, "%s", tdma2a, \
		HO_CFG_STR_HANDOVER \
		"Define measurement set of TDMA frames" HO_CFG_STR_2 \
		"Full set of 102/104 TDMA frames\n" \
		"Sub set of 4 TDMA frames (SACCH)\n") \
	\
	HO_CFG_ONE_MEMBER(int, min_rxlev, -100, \
		"handover min rxlev", "<-110--50>", atoi, "%d", as_is, \
		HO_CFG_STR_HANDOVER \
		HO_CFG_STR_MIN \
		"How weak may RxLev of an MS become before triggering HO\n" \
		"minimum RxLev (dBm)\n") \
	\
	HO_CFG_ONE_MEMBER(int, min_rxqual, 5, \
		"handover min rxqual", "<0-7>", atoi, "%d", as_is, \
		HO_CFG_STR_HANDOVER \
		HO_CFG_STR_MIN \
		"How bad may RxQual of an MS become before triggering HO\n" \
		"minimum RxQual (dBm)\n") \
	\
	HO_CFG_ONE_MEMBER(int, afs_bias_rxlev, 0, \
		"handover afs-bias rxlev", "<0-20>", atoi, "%d", as_is, \
		HO_CFG_STR_HANDOVER \
		HO_CFG_STR_AFS_BIAS \
		"RxLev improvement bias for AFS over other codecs\n" \
		"Virtual RxLev improvement (dBm)\n") \
	\
	HO_CFG_ONE_MEMBER(int, afs_bias_rxqual, 0, \
		"handover afs-bias rxqual", "<0-7>", atoi, "%d", as_is, \
		HO_CFG_STR_HANDOVER \
		HO_CFG_STR_AFS_BIAS \
		"RxQual improvement bias for AFS over other codecs\n" \
		"Virtual RxQual improvement (dBm)\n") \
	\
	HO_CFG_ONE_MEMBER(int, tchf_min_slots, 0, \
		"handover min-free-slots tch/f", "<0-9999>", atoi, "%d", as_is, \
		HO_CFG_STR_HANDOVER \
		HO_CFG_STR_MIN_TCH \
		"Minimum free TCH/F timeslots before cell is considered congested\n" \
		"Number of TCH/F slots\n") \
	\
	HO_CFG_ONE_MEMBER(int, tchh_min_slots, 0, \
		"handover min-free-slots tch/h", "<0-9999>", atoi, "%d", as_is, \
		HO_CFG_STR_HANDOVER \
		HO_CFG_STR_MIN_TCH \
		"Minimum free TCH/H timeslots before cell is considered congested\n" \
		"Number of TCH/H slots\n") \
	\
	HO_CFG_ONE_MEMBER(int, ho_max, 9999, \
		"handover max-handovers", "<1-9999>", atoi, "%d", as_is, \
		HO_CFG_STR_HANDOVER \
		"Maximum number of concurrent handovers allowed per cell" HO_CFG_STR_2 \
		"Number\n") \
	\
	HO_CFG_ONE_MEMBER(int, penalty_max_dist, 300, \
		"handover penalty-time max-distance", "<0-99999>", atoi, "%d", as_is, \
		HO_CFG_STR_HANDOVER \
		HO_CFG_STR_PENALTY_TIME \
		"Time to suspend handovers after leaving this cell due to exceeding max distance\n" \
		"Seconds\n") \
	\
	HO_CFG_ONE_MEMBER(int, penalty_failed_ho, 60, \
		"handover penalty-time failed-ho", "<0-99999>", atoi, "%d", as_is, \
		HO_CFG_STR_HANDOVER \
		HO_CFG_STR_PENALTY_TIME \
		"Time to suspend handovers after handover failure to this cell\n" \
		"Seconds\n") \
	\
	HO_CFG_ONE_MEMBER(int, penalty_failed_as, 60, \
		"handover penalty-time failed-assignment", "<0-99999>", atoi, "%d", as_is, \
		HO_CFG_STR_HANDOVER \
		HO_CFG_STR_PENALTY_TIME \
		"Time to suspend handovers after assignment failure in this cell\n" \
		"Seconds\n") \
	\
	HO_CFG_ONE_MEMBER(int, retries, 0, \
		"handover retries", "<0-9>", atoi, "%d", as_is, \
		HO_CFG_STR_HANDOVER \
		"Immediately retry on handover/assignment failure" HO_CFG_STR_2 \
		"Number of retries\n") \
	

/* Declare public API for handover cfg parameters... */
 
#define HO_CFG_ONE_MEMBER(TYPE, NAME, DEFAULT_VAL, VTY1, VTY2, VTY3, VTY4, VTY5, VTY6) \
	TYPE ho_get_##NAME(struct handover_cfg *ho); \
	void ho_set_##NAME(struct handover_cfg *ho, TYPE val); \
	bool ho_isset_##NAME(struct handover_cfg *ho); \
	void ho_clear_##NAME(struct handover_cfg *ho); \
	bool ho_isset_on_parent_##NAME(struct handover_cfg *ho);

HO_CFG_ALL_MEMBERS
#undef HO_CFG_ONE_MEMBER
