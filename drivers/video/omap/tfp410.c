#include <linux/init.h>
#include <linux/module.h>
#include <asm/arch/omap-dss.h>
#include <linux/i2c/twl4030.h>

#define ENCODER_NAME	"LCD_ENCODER"

int dvi_initialize(void *data)
{
        twl4030_i2c_write_u8(TWL4030_MODULE_GPIO, 0x80,0x03);
        twl4030_i2c_write_u8(TWL4030_MODULE_GPIO, 0x80,0x06);
	return 0;
}

int dvi_deinitialize(void *data)
{
	twl4030_i2c_write_u8(TWL4030_MODULE_GPIO, 0x00,0x03);
	twl4030_i2c_write_u8(TWL4030_MODULE_GPIO, 0x00,0x06);
	return 0;
}

int dvi_set_mode(char *mode_name, void *data)
{
	return 0;
}

int dvi_set_output(char *output_name, void *data)
{
	return 0;
}

struct omap_encoder_device dvi_enc = {
	.enc_name = ENCODER_NAME,
	.initialize = dvi_initialize,
	.deinitialize = dvi_deinitialize,
	.set_mode = dvi_set_mode,
	.set_output = dvi_set_output
};
