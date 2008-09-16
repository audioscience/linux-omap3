#include <linux/init.h>
#include <linux/module.h>
/*#include <linux/config.h>*/
#include <asm/arch/omap-dss.h>
#include <asm/arch/gpio.h>

#define ENCODER_NAME	"LCD_ENCODER"

#define LCD_PANEL_ENABLE_GPIO       153
#define LCD_PANEL_LR                2
#define LCD_PANEL_UD                3
#define LCD_PANEL_INI               152
#define LCD_PANEL_QVGA              154
#define LCD_PANEL_RESB              155

int lcd_initialize(void *data)
{
	omap_set_gpio_dataout(LCD_PANEL_RESB, 1);
	omap_set_gpio_dataout(LCD_PANEL_INI, 1);

	omap_set_gpio_dataout(LCD_PANEL_QVGA, 0);
	omap_set_gpio_dataout(LCD_PANEL_LR, 1);
	omap_set_gpio_dataout(LCD_PANEL_UD, 1);

        omap_set_gpio_dataout(LCD_PANEL_ENABLE_GPIO, 0);

	return 0;
}

int lcd_deinitialize(void *data)
{
	omap_set_gpio_dataout(LCD_PANEL_RESB, 0);
	omap_set_gpio_dataout(LCD_PANEL_INI, 0);
	omap_set_gpio_dataout(LCD_PANEL_ENABLE_GPIO, 1);

	return 0;
}

int lcd_set_mode(char *mode_name, void *data)
{
	return 0;
}

int lcd_set_output(char *output_name, void *data)
{
	return 0;
}

struct omap_encoder_device lcd_enc = {
	.enc_name = ENCODER_NAME,
	.initialize = lcd_initialize,
	.deinitialize = lcd_deinitialize,
	.set_mode = lcd_set_mode,
	.set_output = lcd_set_output
};
