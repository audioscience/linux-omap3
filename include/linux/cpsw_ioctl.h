#ifndef __CPSW_IOCTL_H__
#define __CPSW_IOCTL_H__

#define CPSW_TSCORR_IOCTL (SIOCDEVPRIVATE+1)

enum cpsw_tscorr_type {
	mbit_corr = 0,
	gbit_corr,
	tscorr_type_count
};

struct cpsw_tscorr {
	int32_t tx_correction;
	int32_t rx_correction;
};

enum cpsw_tscorr_cmd {
	cmd_invalid = 0,
	cmd_set,
	cmd_get
};

struct cpsw_tscorr_payload {
	int32_t cmd;
	int32_t corr_type;
	struct cpsw_tscorr corr_data;
	int32_t err;
};

#endif /* __CPSW_IOCTL_H__ */
