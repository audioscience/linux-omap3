#include <linux/init.h>
#include <linux/module.h>
#include <asm/arch/omap-dss.h>

#define ENCODER_NAME	"TV_ENCODER"

int tv_initialize(void *data)
{
	return 0;
}

int tv_deinitialize(void *data)
{
	return 0;
}

int tv_set_mode(char *mode_name, void *data)
{
	return 0;
}

int tv_set_output(char *output_name, void *data)
{
	return 0;
}

struct omap_encoder_device tv_enc = {
	.enc_name = ENCODER_NAME,
	.initialize = tv_initialize,
	.deinitialize = tv_deinitialize,
	.set_mode = tv_set_mode,
	.set_output = tv_set_output
};
