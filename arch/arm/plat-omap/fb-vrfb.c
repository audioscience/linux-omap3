#include <linux/kernel.h>
#include <mach/omapfb.h>

#ifdef DEBUG
#define DBG(format, ...) printk(KERN_DEBUG "VRFB: " format, ## __VA_ARGS__)
#else
#define DBG(format, ...)
#endif

static unsigned int sms_rot_virt_base(int context, int rot)
{
        switch (context)
        {
        case 0:
        case 1:
        case 2:
        case 3:
                return (0x70000000 | 0x4000000 * (context) | 0x1000000 * (rot));
                break;
        case 4:
        case 5:
        case 6:
        case 7:
                return (0xE0000000 | 0x4000000 * (context - 4) | 0x1000000 * (rot));
                break;
        case 8:
        case 9:
        case 10:
        case 11:
                return (0xF0000000 | 0x4000000 * (context - 8) | 0x1000000 * (rot));
        }
        return -1;
}
#define SMS_ROT_VIRT_BASE(context, rot) sms_rot_virt_base(context, rot)

#define PAGE_WIDTH_EXP         5 /* Assuming SDRAM pagesize= 1024 */
#define PAGE_HEIGHT_EXP        5 /* 1024 = 2^5 * 2^5 */
#define SMS_IMAGEHEIGHT_OFFSET 16
#define SMS_IMAGEWIDTH_OFFSET  0
#define SMS_PH_OFFSET          8
#define SMS_PW_OFFSET          4
#define SMS_PS_OFFSET          0
#define VRFB_LINE_LEN          2048

#define OMAP_SMS_BASE          0x6C000000
#define SMS_ROT_CONTROL(context)	(OMAP_SMS_BASE + 0x180 + 0x10 * context)
#define SMS_ROT_SIZE(context)		(OMAP_SMS_BASE + 0x184 + 0x10 * context)
#define SMS_ROT_PHYSICAL_BA(context)	(OMAP_SMS_BASE + 0x188 + 0x10 * context)

#define VRFB_NUM_CTXS 12
/* bitmap of reserved contexts */
static unsigned ctx_map;

static inline u32 calc_vrfb_div(u32 img_side, u32 page_exp)
{
	u32 div = img_side / page_exp;
	if ((div * page_exp) < img_side)
		return div + 1;
	else
		return div;
}

void omap_vrfb_setup(int ctx, unsigned long paddr, u32 width, u32 height,
		int bytespp)
{
	int page_width_exp, page_height_exp, pixel_size_exp;
	int div;
	u32 vrfb_width;
	u32 vrfb_height;
	u32 bytes_per_pixel = bytespp;

	DBG("omapfb_set_vrfb(%d, %lx, %dx%d, %d)\n", ctx, paddr,
			width, height, bytespp);

	page_width_exp = PAGE_WIDTH_EXP;
	page_height_exp = PAGE_HEIGHT_EXP;
	pixel_size_exp = bytes_per_pixel >> 1;

	div = calc_vrfb_div(width * bytes_per_pixel, 1 << page_width_exp);
	vrfb_width = (div * (1 << page_width_exp)) / bytes_per_pixel;

	div = calc_vrfb_div(height, 1 << page_height_exp);
	vrfb_height = div * (1 << page_height_exp);

	DBG("vrfb w %u, h %u\n", vrfb_width, vrfb_height);

	omap_writel(paddr, SMS_ROT_PHYSICAL_BA(ctx));
	omap_writel((vrfb_width << SMS_IMAGEWIDTH_OFFSET)
			| (vrfb_height << SMS_IMAGEHEIGHT_OFFSET),
			SMS_ROT_SIZE(ctx));

	omap_writel(pixel_size_exp << SMS_PS_OFFSET
			| page_width_exp  << SMS_PW_OFFSET
			| page_height_exp << SMS_PH_OFFSET,
			SMS_ROT_CONTROL(ctx));

	DBG("vrfb offset %d, %d\n", vrfb_width - width, vrfb_height - height);

}
EXPORT_SYMBOL(omap_vrfb_setup);

static int omapfb_request_vrfb_mem(int ctx, int rot,
		u32 *paddr)
{
	*paddr = SMS_ROT_VIRT_BASE(ctx, rot);
	if (!request_mem_region(*paddr, VRFB_SIZE, "vrfb")) {
		printk(KERN_ERR "vrfb: request_mem_region failed\n");
		return -ENOMEM;
	}

	return 0;
}

void omap_vrfb_release_ctx(struct vrfb *vrfb)
{
	int rot;

	DBG("release ctx %d\n", vrfb->context);

	ctx_map &= ~(1 << vrfb->context);

	for (rot = 0; rot < 4; ++rot) {
		if(vrfb->vaddr[rot]) {
			iounmap(vrfb->vaddr[rot]);
			vrfb->vaddr[rot] = NULL;
		}

		if(vrfb->paddr[rot]) {
			release_mem_region(vrfb->paddr[rot], VRFB_SIZE);
			vrfb->paddr[rot] = 0;
		}
	}

	vrfb->context = -1;
}
EXPORT_SYMBOL(omap_vrfb_release_ctx);

int omap_vrfb_create_ctx(struct vrfb *vrfb)
{
	u32 paddr;
	int ctx, rot;

	DBG("create ctx\n");

	for (ctx = 0; ctx < VRFB_NUM_CTXS; ++ctx)
		if ((ctx_map & (1 << ctx)) == 0)
			break;

	if (ctx == VRFB_NUM_CTXS) {
		printk(KERN_ERR "vrfb: no free contexts\n");
		return -EBUSY;
	}

	DBG("found free ctx %d\n", ctx);

	ctx_map |= 1 << ctx;

	memset(vrfb, 0, sizeof(*vrfb));

	vrfb->context = ctx;

	for (rot = 0; rot < 4; ++rot) {
		int r;
		r = omapfb_request_vrfb_mem(ctx, rot, &paddr);
		if (r) {
			printk(KERN_ERR "vrfb: failed to reserve VRFB "
					"area for ctx %d, rotation %d\n",
					ctx, rot * 90);
			omap_vrfb_release_ctx(vrfb);
			return r;
		}
		vrfb->paddr[rot] = paddr;
		DBG("VRFB %d/%d: %lx\n", ctx, rot*90,
				vrfb->paddr[rot]);
	}

	return 0;
}
EXPORT_SYMBOL(omap_vrfb_create_ctx);

