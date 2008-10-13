/*
 * arch/arm/plat-omap/edisco_drv.h
 *
 * Copyright (C) 2008 Texas Instruments.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 */

#ifndef OMAP_EDISCO_DRV_H
#define OMAP_EDISCO_DRV_H


#include "types.h"
#include "edisco_dis.h"
#include "dsi_dis.h"

#define MIPI_DSI	0x1
#define MIPI_DPI	0x2
#define MIPI_DBI_B	0x3
#define MIPI_DBI_C	0x4
#define RGB_12_8	0x5
#define T_HSSI		0x6

#define REFCLK_32KHZ	0x1
#define REFCLK_19P2MHZ	0x2

#define DCS_SOFT_RESET			0x01
#define DCS_EXIT_SLEEP_MODE		0x11
#define DCS_SET_DISPLAY_ON		0x29
#define DCS_SET_DISPLAY_OFF		0x28
#define DCS_SET_PIXEL_FORMAT		0x3A
#define DCS_SET_DISPLAY_BUFFER_IO_CONTROL	0x82
#define DCS_SET_OUTPUT_PIXEL_CLK_FREQ	0x9E
#define DCS_SET_OUTPUT_VERTICAL_TIMING	0x8B
#define DCS_SET_OUTPUT_HORIZONTAL_TIMING	0x92
#define DCS_SET_SPECIAL_COMMAND_SET	0x9D
#define DCS_SET_COLUMN_ADDRESS		0x2A
#define DCS_SET_PAGE_ADDRESS		0x2B
#define DCS_WRITE_EDISCO_REGISTER	0xFD
#define DCS_WRITE_INDEX_REGISTER 	0xFB
#define DCS_WRITE_MEMORY_START		0x2C
#define DCS_WRITE_MEMORY_CONTINUE	0x3C
#define DCS_VSYNC_START				0x01
#define DCS_HSYNC_START				0x21

#define DCS_GET_ID1			0xDA
#define DCS_GET_ID2			0xDB
#define DCS_GET_ID3			0xDC
#define DCS_GET_DIAG_RESULT			0x0F
#define DCS_GET_PIXEL_FORMAT		0x0C
#define DCS_GET_SCAN_LINE			0x45
#define DCS_GET_DISPLAY_BUF_IO_CTRL 0x83
#define DCS_GET_OUTPUT_VERTICAL_FRONT_PORCH 0x8C
#define DCS_GET_OUTPUT_VERTICAL_SYNC_PULSE 0x8D
#define DCS_GET_OUTPUT_VERTICAL_BACK_PORCH 0x8E
#define DCS_GET_OUTPUT_VERTICAL_ACT_LINES_MSB 0x8F
#define DCS_GET_OUTPUT_VERTICAL_ACT_LINES_CSB 0x90
#define DCS_GET_OUTPUT_VERTICAL_ACT_LINES_LSB 0x91
#define DCS_GET_OUTPUT_HORIZONTAL_FRONT_PORCH 0x93
#define DCS_GET_OUTPUT_HORIZONTAL_SYNC_PULSE 0x94
#define DCS_GET_OUTPUT_HORIZONTAL_BACK_PORCH 0x95
#define DCS_GET_OUTPUT_HORIZONTAL_ACT_LINES_MSB 0x96
#define DCS_GET_OUTPUT_HORIZONTAL_ACT_LINES_CSB 0x97
#define DCS_GET_OUTPUT_HORIZONTAL_ACT_LINES_LSB 0x98
#define DCS_GET_OUTPUT_PCLK_FREQ 0x9F
#define DCS_READ_EDISCO_REGISTER 0xFC
#define DCS_READ_MEMORY_START	 0x2E
#define DCS_READ_MEMORY_CONTINUE 0x3E


#define DCS_GET_POWER_MODE 	0x0A
#define DCS_GET_ADDRESS_MODE 0x0B
#define DCS_GET_SIGNAL_MODE 0x0E

typedef struct {

	T_EDISCO_INIT init;
	U8 input_interface;
	U8 output_interface;
	U32 max_data_rate;
	U16 num_data_lanes;
	U8 virtual_channel_id;
	U8 data_lane_pol;
	U8 clk_lane_pol;
	BOOLEAN ecc_flag;
	BOOLEAN checksum_flag;
	U16 max_resp_size;
	U8 display_mode;
	U8 input_clk_fre;
	T_VM_TIMING video_mode_timing;
	T_DPI_OUT_CONFIG output_intf_config;
	U32 dsi_handle;
} T_EDISCO_DIS;

void edisco_init(U32 *device_handle);
S32 edisco_read(U32 device_handle, U32 read_tag, U32 *len, U8 *buf);
void edisco_write(U32 device_handle, U32 write_tag, U32 *len, U8 *buf);
void edisco_deinit(U32 device_handle);

S32 init_dsi_driver(T_EDISCO_DIS *edisco_dis);
S32 init_edisco(T_EDISCO_DIS *edisco_dis);
S32 read_edisco_id(U32 dsi_handle, U8 *buf);
S32 start_image_display(U32 dsi_handle);
S32 stop_image_display(U32 dsi_handle);
S32 start_colorbar_display(U32 dsi_handle);
S32 stop_colorbar_display(U32 dsi_handle);
S32 reset_edisco(U32 dsi_handle);
S32 exit_sleep_mode(U32 dsi_handle);
S32 set_pixel_format(T_DPI_OUT_CONFIG *lcd_config, U32 dsi_handle);
S32 set_output_pixel_clk(T_DPI_OUT_CONFIG *lcd_config, U32 dsi_handle,\
				U8 edisco_refclk);
S32 set_output_vertical_timing(T_DPI_OUT_CONFIG *lcd_config, U32 dsi_handle);
S32 set_output_horizontal_timing(T_DPI_OUT_CONFIG *lcd_config, U32 dsi_handle);
S32 enable_special_cmd_set(U32 dsi_handle);
S32 set_output_interface_signal_polarity(T_DPI_OUT_CONFIG *lcd_config,\
						U32 dsi_handle);
S32 set_output_interface_pixel_format(T_DPI_OUT_CONFIG *lcd_config,\
						U32 dsi_handle);
S32 set_image_size(U16 image_width, U16 image_height, U32 dsi_handle);
S32 get_diag_result(U32 dsi_handle);

/*====================== FUNCTIONS PROTOTYPES ==========================*/

S32 get_pixel_format(U32 dsi_handle);
S32 get_output_vertical_front_porch(U32 dsi_handle);
S32 get_output_vertical_sync_pulse(U32 dsi_handle);
S32 get_output_vertical_back_porch(U32 dsi_handle);
S32 get_output_vertical_act_lines_msb(U32 dsi_handle);
S32 get_output_vertical_act_lines_csb(U32 dsi_handle);
S32 get_output_vertical_act_lines_lsb(U32 dsi_handle);
S32 get_output_horizontal_front_porch(U32 dsi_handle);
S32 get_output_horizontal_sync_pulse(U32 dsi_handle);
S32 get_output_horizontal_back_porch(U32 dsi_handle);
S32 get_output_horizontal_act_lines_msb(U32 dsi_handle);
S32 get_output_horizontal_act_lines_csb(U32 dsi_handle);
S32 get_output_horizontal_act_lines_lsb(U32 dsi_handle);
S32 read_eDISCO_spec_reg(U32 dsi_handle);
S32 get_output_pclk_freq(U32 dsi_handle);
S32 get_buffer_io_line(U32 dsi_handle);
S32 enable_reg_settings(U32 dsi_handle);
S32 get_power_mode(U32 dsi_handle);
S32 get_address_mode(U32 dsi_handle);
S32 get_signal_mode(U32 dsi_handle);
S32 get_scan_line(U32 dsi_handle);
S32 start_read_from_edisco_buffer(U32 dsi_handle);
S32 start_write_to_edisco_buffer(U32 dsi_handle);
S32 continue_write_to_edisco_buffer(U32 dsi_handle);
S32 set_display_on(U32 dsi_handle);
S32 disable_eot_reception(U32 dsi_handle);

#endif /* OMAP_EDISCO_DRV_H */
