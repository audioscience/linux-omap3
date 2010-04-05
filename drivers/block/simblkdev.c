#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/vmalloc.h>
#include <linux/genhd.h>
#include <linux/blkdev.h>
#include <linux/hdreg.h>
#include <linux/scatterlist.h>

MODULE_LICENSE("GPL");

#define SBD_IOADDR      CONFIG_SIMBLKDEV_PHYSADDR
#define SBD_IO_SIZE     0x10000

/* Low level */
/* Commands IDs */
#define SBD_GETSIZE      0xBABA0000	/* Get the size of attached disk */
#define SBD_READBLK      0xBABA0001	/* Read a block from file */
#define SBD_WRITEBLK     0xBABA0002	/* Write a block to file */
#define SBD_START        0xBABA0003	/* Start the simulation */
#define SBD_STOP         0xBABA0004	/* Stop the simulation */

/* SimBlkDev state structure */
#define BUFFER_SIZE     (4096 + 100)
typedef struct {
	/* This goes to zero when cmd is processed, result is in buffer */
	volatile uint32_t execute;
	/* command ID */
	volatile uint32_t cmdId;
	/* Buffer area */
	volatile uint8_t buffer[BUFFER_SIZE];
} SBDSTATE;

/* Start result */
typedef struct {
	int ret;
} llsbd_res_start;

/* Stop command & result */
typedef struct {
	int fd;
} llsbd_cmd_stop;

typedef struct {
	int ret;
} llsbd_res_stop;

/* GetSize command & result */
typedef struct {
	int fd;
} llsbd_cmd_getsize;

typedef struct {
	uint32_t size;		/* Upto 4GB size */
} llsbd_res_getsize;

/* ReadBlk command & result */
typedef struct {
	int fd;
	uint32_t offset;	/* Offset to read from */
	uint32_t count;		/* Number of bytes to be read */
	uint32_t phys;		/* DDR address */
} llsbd_cmd_readblk;

typedef struct {
	uint32_t count;		/* Number of bytes read */
} llsbd_res_readblk;

/* WriteBlk command & result */
typedef struct {
	int fd;
	uint32_t offset;	/* Write to offset */
	uint32_t count;		/* Number of bytes to be written */
	uint32_t phys;		/* DDR address */
} llsbd_cmd_writeblk;

typedef struct {
	uint32_t count;		/* Number of bytes wrote */
} llsbd_res_writeblk;

/* low level api */
static int llsbd_start(void *ioaddr, char *filename)
{

	SBDSTATE *sbd = (SBDSTATE *) ioaddr;
	llsbd_res_start *res = (llsbd_res_start *) sbd->buffer;
	char *fp = (char *)sbd->buffer;

	strcpy(fp, filename);
	sbd->cmdId = SBD_START;
	sbd->execute = 1;

	while (sbd->execute == 1)
		/* BUSY LOOP */;

	return res->ret;
}

static int llsbd_stop(void *ioaddr, int fd)
{

	SBDSTATE *sbd = (SBDSTATE *) ioaddr;
	llsbd_cmd_stop *cmd = (llsbd_cmd_stop *) sbd->buffer;
	llsbd_res_stop *res = (llsbd_res_stop *) sbd->buffer;

	cmd->fd = fd;
	sbd->cmdId = SBD_STOP;
	sbd->execute = 1;

	while (sbd->execute == 1)
		/* BUSY LOOP */;

	return res->ret;
}

static uint32_t llsbd_getsize(void *ioaddr, int fd)
{

	SBDSTATE *sbd = (SBDSTATE *) ioaddr;
	llsbd_cmd_getsize *cmd = (llsbd_cmd_getsize *) sbd->buffer;
	llsbd_res_getsize *res = (llsbd_res_getsize *) sbd->buffer;

	cmd->fd = fd;
	sbd->cmdId = SBD_GETSIZE;
	sbd->execute = 1;

	while (sbd->execute == 1)
		/* BUSY LOOP */;

	return res->size;
}

static int llsbd_readblk(void *ioaddr,
			 int fd, uint32_t offset, void *buffer, uint32_t count)
{

	SBDSTATE *sbd = (SBDSTATE *) ioaddr;
	llsbd_cmd_readblk *cmd = (llsbd_cmd_readblk *) sbd->buffer;
	llsbd_res_readblk *res = (llsbd_res_readblk *) sbd->buffer;

	if (count > PAGE_SIZE)
		return 0;

	cmd->fd = fd;
	cmd->offset = offset;
	cmd->count = count;
	cmd->phys = (uint32_t) buffer;
	sbd->cmdId = SBD_READBLK;
	sbd->execute = 1;

	while (sbd->execute == 1)
		/* BUSY LOOP */;

	return res->count;
}

static int llsbd_writeblk(void *ioaddr,
			  int fd, uint32_t offset, void *buffer, uint32_t count)
{

	SBDSTATE *sbd = (SBDSTATE *) ioaddr;
	llsbd_cmd_writeblk *cmd = (llsbd_cmd_writeblk *) sbd->buffer;
	llsbd_res_writeblk *res = (llsbd_res_writeblk *) sbd->buffer;

	if (count > PAGE_SIZE)
		return 0;

	cmd->fd = fd;
	cmd->offset = offset;
	cmd->count = count;
	cmd->phys = (uint32_t) buffer;
	sbd->cmdId = SBD_WRITEBLK;
	sbd->execute = 1;

	while (sbd->execute == 1)
		/* BUSY LOOP */;

	return res->count;
}

/* Major number */
#define SBD_MAJOR       58

/* Sector size */
#define SECTOR_SIZE     512

/* Request queue */
static struct request_queue *SimBlkDev_queue;

/* SimBlkDev state */
typedef struct SimBlkDev {
	unsigned long size;
	spinlock_t lock;
	struct gendisk *gd;
	void *ioaddr;
	int fd;
} SimBlkDev;

/* SimBlkDev state variable */
static SimBlkDev simBlkDev;

/* IO function */
static
void SimBlkDev_doIO(SimBlkDev *dev,
		    unsigned long sector,
		    unsigned long nsect,
		    char *buffer,
		    int write, struct request_queue *q, struct request *req)
{
	unsigned long offset = sector * SECTOR_SIZE;
	unsigned long nbytes = nsect * SECTOR_SIZE;
	unsigned long phys;
	struct page *page;

	if ((offset + nbytes) > dev->size) {
		printk(KERN_NOTICE "SimBlkDev: Beyond-end write (%ld %ld)\n",
		       offset, nbytes);
		return;
	}

	phys = virt_to_phys(buffer);

	if (write) {
		if (llsbd_writeblk
		    (dev->ioaddr, dev->fd, offset, (void *)phys, nbytes)
		    != nbytes) {
			printk(KERN_NOTICE
			       "SimBlkDev: Low level write error "
			       "(%ld 0x%08x %ld)\n",
			       offset, (unsigned int)phys, (long int)nbytes);
			return;
		}
	} else {
		if (llsbd_readblk
		    (dev->ioaddr, dev->fd, offset, (void *)phys, nbytes)
		    != nbytes) {
			printk(KERN_NOTICE
			       "SimBlkDev: Low level read error "
			       "(%ld 0x%08x %ld)\n",
			       offset, (unsigned int)phys, (long int)nbytes);
			return;
		}

		page = pfn_to_page(phys >> PAGE_SHIFT);
		flush_dcache_page(page);
		SetPageUptodate(page);
		if (PageError(page))
			ClearPageError(page);
	}
}

/* Request function */
static void SimBlkDev_request(struct request_queue *q)
{
	struct request *req;

	req = blk_fetch_request(q);
	while (req) {
		SimBlkDev_doIO((SimBlkDev *) req->rq_disk->private_data,
			       blk_rq_pos(req),
			       blk_rq_cur_sectors(req),
			       req->buffer, rq_data_dir(req), q, req);

		if (!(__blk_end_request_cur(req, 0)))
			req = blk_fetch_request(q);
	}
}

/* SimBlkDev IOCTL */
int SimBlkDev_ioctl(struct inode *inode, struct file *filp,
		    unsigned int cmd, unsigned long arg)
{
	long size;
	struct hd_geometry geo;

	switch (cmd) {
	case HDIO_GETGEO:
		size = simBlkDev.size;
		geo.cylinders = size / (64 * 32 * 512);
		geo.heads = 64;
		geo.sectors = 32;
		if (copy_to_user((void *)arg, &geo, sizeof(geo)))
			return -EFAULT;
		return 0;
	}

	return -ENOTTY;		/* unknown command */
}

/*
 * The device operations structure.
 */
static struct block_device_operations SimBlkDev_ops = {
	.owner = THIS_MODULE,
	.ioctl = SimBlkDev_ioctl
};

static int __init SimBlkDev_init(void)
{
	int major_num = SBD_MAJOR;

/*
 * Set up our internal device.
 */
	simBlkDev.ioaddr = (void *)ioremap_nocache(SBD_IOADDR, SBD_IO_SIZE);
	if (simBlkDev.ioaddr == NULL) {
		printk(KERN_WARNING "SimBlkDev: Failed to map io region\n");
		goto out;
	}

	spin_lock_init(&simBlkDev.lock);

/*
 * Get a request queue.
 */
	SimBlkDev_queue = blk_init_queue(SimBlkDev_request, &simBlkDev.lock);
	if (SimBlkDev_queue == NULL)
		goto out;
	blk_queue_logical_block_size(SimBlkDev_queue, SECTOR_SIZE);
	blk_queue_max_sectors(SimBlkDev_queue, (PAGE_SIZE / SECTOR_SIZE));
	blk_queue_max_phys_segments(SimBlkDev_queue, 1);
	blk_queue_max_hw_segments(SimBlkDev_queue, 1);
	blk_queue_max_segment_size(SimBlkDev_queue, PAGE_SIZE);
/*
 * Get registered.
 */
	major_num = register_blkdev(SBD_MAJOR, "SimBlkDev");
	if (major_num < 0) {
		printk(KERN_WARNING
		       "SimBlkDev: unable to get major number %d\n", SBD_MAJOR);
		goto out;
	} else {
		printk("major allocated: %d\n", SBD_MAJOR);
	}

/*
 * Start the low level emulation
 */
	simBlkDev.fd = llsbd_start(simBlkDev.ioaddr, "c:\\temp\\hd.img");
	if (simBlkDev.fd < 0) {
		printk(KERN_WARNING
		       "SimBlkDev: unable to start the low level emulation\n");
		goto out_unregister;
	}

/*
 * Set the size of hdd
 */
	simBlkDev.size = llsbd_getsize(simBlkDev.ioaddr, simBlkDev.fd);
	if (simBlkDev.size <= 0) {
		printk(KERN_WARNING "SimBlkDev: zero size harddisk\n");
		goto out_unregister;
	} else {
		printk(KERN_INFO "Using c:\\temp\\hd.img of size %ld\n",
		       simBlkDev.size);
	}

/*
 * And the gendisk structure.
 */
	simBlkDev.gd = alloc_disk(1);
	if (!simBlkDev.gd)
		goto out_unregister;

	simBlkDev.gd->major = SBD_MAJOR;
	simBlkDev.gd->first_minor = 0;
	simBlkDev.gd->fops = &SimBlkDev_ops;
	simBlkDev.gd->private_data = &simBlkDev;
	strcpy(simBlkDev.gd->disk_name, "SimBlkDev0");
	set_capacity(simBlkDev.gd, simBlkDev.size);
	simBlkDev.gd->queue = SimBlkDev_queue;
	add_disk(simBlkDev.gd);

	return 0;

out_unregister:
	unregister_blkdev(SBD_MAJOR, "SimBlkDev");
out:
	return -ENOMEM;
}

static void __exit SimBlkDev_exit(void)
{
	int major_num = SBD_MAJOR;

	llsbd_stop(simBlkDev.ioaddr, simBlkDev.fd);

	del_gendisk(simBlkDev.gd);

	put_disk(simBlkDev.gd);

	unregister_blkdev(major_num, "SimBlkDev");

	blk_cleanup_queue(SimBlkDev_queue);
}

module_init(SimBlkDev_init);
module_exit(SimBlkDev_exit);
