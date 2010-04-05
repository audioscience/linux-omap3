#include <linux/fs.h>
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/pagemap.h>
#include <linux/statfs.h>
#include <linux/seq_file.h>
#include <linux/mount.h>
#include "ll.h"

/* ---- COMMAND IDS ----- */
#define FS_CREATE       0xBABA0000	/* File open */
#define FS_OPEN         0xBABA0001	/* File open */
#define FS_CLOSE        0xBABA0002	/* File close */
#define FS_READ         0xBABA0003	/* File read */
#define FS_WRITE        0xBABA0004	/* File write */
#define FS_SEEK         0xBABA0005	/* File seek */
#define FS_STAT         0xBABA0006	/* File stat */
#define FS_FSTAT        0xBABA0007	/* File stat */
#define FS_MKDIR        0xBABA0008	/* mkdir */
#define FS_RMDIR        0xBABA0009	/* Remove dir */
#define FS_RENAME       0xBABA000A	/* Rename */
#define FS_ACCESS       0xBABA000B	/* Check status of a file */
#define FS_REMOVE       0xBABA000C	/* delete a file */
#define FS_OPENDIR      0xBABA000D	/* open a dir */
#define FS_READDIR      0xBABA000E
#define FS_TELLDIR      0xBABA000F
#define FS_SEEKDIR      0xBABA0010
#define FS_CLOSEDIR     0xBABA0011

/* ---- FS state structure ----- */
#define BUFFER_SIZE     0x4000	/* 4KiB */
typedef struct {
	/* windows 0 and linux 1 */
	volatile int hostos;
	volatile int error;
	/* This goes to zero when cmd is processed, result is in buffer */
	volatile uint32_t execute;
	/* command ID */
	volatile uint32_t cmdId;
	/* Buffer area */
	volatile uint8_t buffer[BUFFER_SIZE];
} FSSTATE;

/* ---- Create command & result, API ----- */
typedef struct {
	uint32_t fd;
} ll_res_create;

int ll_create(void *ioaddr, const char *path)
{
	FSSTATE *fss = (FSSTATE *) ioaddr;
	ll_res_create *res = (ll_res_create *) fss->buffer;
	char *f = (char *)(fss->buffer);
	unsigned long flags;
	int ret;

	local_irq_save(flags);

	/* copy parameters */
	strcpy(f, path);

	/* execute the command */
	fss->cmdId = FS_CREATE;
	fss->execute = 1;

	/* Wait for command to finish */
	while (fss->execute == 1)
		/* BUSY LOOP */;

	if (res->fd == -1)
		ret = fss->error;
	else
		ret = res->fd;

	local_irq_restore(flags);

	return ret;
}

/* ---- Open command & result, API ----- */
typedef struct {
	uint32_t oflag;
} ll_cmd_open;

typedef struct {
	uint32_t fd;
} ll_res_open;

int ll_open(void *ioaddr, const char *path, int flag)
{
	FSSTATE *fss = (FSSTATE *) ioaddr;
	ll_cmd_open *cmd = (ll_cmd_open *) fss->buffer;
	ll_res_open *res = (ll_res_open *) fss->buffer;
	char *f = (char *)(fss->buffer + sizeof(ll_cmd_open));
	unsigned long flags;
	int ret;

	local_irq_save(flags);

	/* copy parameters */
	cmd->oflag = flag;
	strcpy(f, path);

	/* execute the command */
	fss->cmdId = FS_OPEN;
	fss->execute = 1;

	/* Wait for command to finish */
	while (fss->execute == 1)
		/* BUSY LOOP */;

	if (res->fd == -1)
		ret = fss->error;
	else
		ret = res->fd;

	local_irq_restore(flags);

	return ret;
}

/* ---- Close command & result, API ----- */
typedef struct {
	uint32_t fd;
} ll_cmd_close;

typedef struct {
	uint32_t ret;
} ll_res_close;

int ll_close(void *ioaddr, int fd)
{
	FSSTATE *fss = (FSSTATE *) ioaddr;
	ll_cmd_close *cmd = (ll_cmd_close *) fss->buffer;
	ll_res_close *res = (ll_res_close *) fss->buffer;
	unsigned long flags;
	int ret;

	local_irq_save(flags);

	/* copy parameters */
	cmd->fd = fd;

	/* execute the command */
	fss->cmdId = FS_CLOSE;
	fss->execute = 1;

	/* Wait for command to finish */
	while (fss->execute == 1)
		/* BUSY LOOP */;

	if (res->ret == -1)
		ret = fss->error;
	else
		ret = res->ret;

	local_irq_restore(flags);

	return ret;
}

/* ---- Read command & result, API ----- */
typedef struct {
	uint32_t fd;
	uint32_t count;
	uint32_t phys;
} ll_cmd_read;

typedef struct {
	uint32_t ret;
} ll_res_read;

int ll_read(void *ioaddr, int fd, void *buffer, uint32_t count)
{
	FSSTATE *fss = (FSSTATE *) ioaddr;
	ll_cmd_read *cmd = (ll_cmd_read *) fss->buffer;
	ll_res_read *res = (ll_res_read *) fss->buffer;
	unsigned long flags;
	int ret;

	local_irq_save(flags);

	/* copy parameters */
    cmd->fd    = fd;
    cmd->count = count;
    cmd->phys  = (uint32_t) buffer;

	/* execute the command */
	fss->cmdId = FS_READ;
	fss->execute = 1;

	/* Wait for command to finish */
	while (fss->execute == 1)
		/* BUSY LOOP */;
	if (res->ret == -1)
		ret = fss->error;
	else
		ret = res->ret;

	local_irq_restore(flags);

	return ret;
}

/* ---- Write command & result, API ----- */
typedef struct {
	uint32_t fd;
	uint32_t count;
	uint32_t phys;
} ll_cmd_write;

typedef struct {
	uint32_t ret;
} ll_res_write;

int ll_write(void *ioaddr, int fd, void *buffer, uint32_t count)
{
	FSSTATE *fss = (FSSTATE *) ioaddr;
	ll_cmd_write *cmd = (ll_cmd_write *) fss->buffer;
	ll_res_write *res = (ll_res_write *) fss->buffer;
	unsigned long flags;
	int ret;

	local_irq_save(flags);

	/* copy parameters */
	cmd->fd = fd;
	cmd->count = count;
    cmd->phys  = (uint32_t) buffer;

	/* execute the command */
	fss->cmdId = FS_WRITE;
	fss->execute = 1;

	/* Wait for command to finish */
	while (fss->execute == 1)
		/* BUSY LOOP */;

	if (res->ret == -1)
		ret = fss->error;
	else
		ret = res->ret;

	local_irq_restore(flags);

	return ret;
}

/* ---- Seek command & result, API ----- */
typedef struct {
	uint32_t fd;
	uint32_t offset;
	uint32_t origin;
} ll_cmd_seek;

typedef struct {
	uint32_t offset;
} ll_res_seek;

long ll_seek(void *ioaddr, int fd, long offset, int origin)
{
	FSSTATE *fss = (FSSTATE *) ioaddr;
	ll_cmd_seek *cmd = (ll_cmd_seek *) fss->buffer;
	ll_res_seek *res = (ll_res_seek *) fss->buffer;
	unsigned long flags;
	long ret;

	local_irq_save(flags);

	/* copy parameters */
	cmd->fd = fd;
	cmd->offset = offset;
	cmd->origin = origin;

	/* execute the command */
	fss->cmdId = FS_SEEK;
	fss->execute = 1;

	/* Wait for command to finish */
	while (fss->execute == 1)
		/* BUSY LOOP */;

	if (res->offset == -1)
		ret = fss->error;
	else
		ret = res->offset;

	local_irq_restore(flags);

	return ret;
}

/* ---- stat command & result, API ----- */
typedef struct {
	uint32_t ret;
} ll_res_stat;

int ll_stat(void *ioaddr, char *path, _ll_stat * stbuf)
{
	FSSTATE *fss = (FSSTATE *) ioaddr;
	ll_res_stat *res = (ll_res_stat *) fss->buffer;
	char *p = (char *)fss->buffer;
	_ll_stat *sb = (_ll_stat *) (fss->buffer + sizeof(ll_res_stat));
	unsigned long flags;
	int ret;

	local_irq_save(flags);

	strcpy(p, path);

	/* execute the command */
	fss->cmdId = FS_STAT;
	fss->execute = 1;

	/* Wait for command to finish */
	while (fss->execute == 1)
		/* BUSY LOOP */;

	stbuf->st_dev = sb->st_dev;
	stbuf->st_ino = sb->st_ino;
	stbuf->st_mode = sb->st_mode;
	stbuf->st_nlink = sb->st_nlink;
	stbuf->st_uid = sb->st_uid;
	stbuf->st_gid = sb->st_gid;
	stbuf->st_rdev = sb->st_rdev;
	stbuf->st_size = sb->st_size;
	stbuf->st_atime = sb->st_atime;
	stbuf->st_mtime = sb->st_mtime;
	stbuf->st_ctime = sb->st_ctime;

	if (res->ret == -1)
		ret = fss->error;
	else
		ret = res->ret;

	local_irq_restore(flags);

	return ret;
}

/* ---- fstat command & result, API ----- */
typedef struct {
	uint32_t fd;
} ll_cmd_fstat;

typedef struct {
	uint32_t ret;
} ll_res_fstat;

int ll_fstat(void *ioaddr, int fd, _ll_stat * stbuf)
{
	FSSTATE *fss = (FSSTATE *) ioaddr;
	ll_cmd_fstat *cmd = (ll_cmd_fstat *) fss->buffer;
	ll_res_fstat *res = (ll_res_fstat *) fss->buffer;
	_ll_stat *sb = (_ll_stat *) (fss->buffer + sizeof(ll_res_fstat));
	unsigned long flags;
	int ret;

	local_irq_save(flags);

	cmd->fd = fd;

	/* execute the command */
	fss->cmdId = FS_FSTAT;
	fss->execute = 1;

	/* Wait for command to finish */
	while (fss->execute == 1)
		/* BUSY LOOP */;

	stbuf->st_dev = sb->st_dev;
	stbuf->st_ino = sb->st_ino;
	stbuf->st_mode = sb->st_mode;
	stbuf->st_nlink = sb->st_nlink;
	stbuf->st_uid = sb->st_uid;
	stbuf->st_gid = sb->st_gid;
	stbuf->st_rdev = sb->st_rdev;
	stbuf->st_size = sb->st_size;
	stbuf->st_atime = sb->st_atime;
	stbuf->st_mtime = sb->st_mtime;
	stbuf->st_ctime = sb->st_ctime;

	if (res->ret == -1)
		ret = fss->error;
	else
		ret = res->ret;

	local_irq_restore(flags);

	return ret;
}

/* ---- rename command & result, API ----- */
typedef struct {
	char *oldname;
	char *newname;
} ll_cmd_rename;

typedef struct {
	uint32_t ret;
} ll_res_rename;

int ll_rename(void *ioaddr, char *oldname, char *newname)
{
	FSSTATE *fss = (FSSTATE *) ioaddr;
	ll_cmd_rename *cmd = (ll_cmd_rename *) fss->buffer;
	ll_res_rename *res = (ll_res_rename *) fss->buffer;
	char *p = (char *)fss->buffer;
	unsigned long flags;
	int ret;

	local_irq_save(flags);

	strcpy(p, oldname);
	cmd->oldname = (char *)0;
	p = (char *)(p + strlen(p) + 1);
	strcpy(p, newname);
	cmd->newname = (char *)(strlen(p) + 1);

	/* execute the command */
	fss->cmdId = FS_RENAME;
	fss->execute = 1;

	/* Wait for command to finish */
	while (fss->execute == 1)
		/* BUSY LOOP */;

	if (res->ret == -1)
		ret = fss->error;
	else
		ret = res->ret;

	local_irq_restore(flags);

	return ret;
}

/* ---- mkdir command & result, API ----- */
typedef struct {
	uint32_t ret;
} ll_res_mkdir;

int ll_mkdir(void *ioaddr, char *path)
{
	FSSTATE *fss = (FSSTATE *) ioaddr;
	ll_res_mkdir *res = (ll_res_mkdir *) fss->buffer;
	char *p = (char *)fss->buffer;
	unsigned long flags;
	int ret;

	local_irq_save(flags);

	strcpy(p, path);

	/* execute the command */
	fss->cmdId = FS_MKDIR;
	fss->execute = 1;

	/* Wait for command to finish */
	while (fss->execute == 1)
		/* BUSY LOOP */;

	if (res->ret == -1)
		ret = fss->error;
	else
		ret = res->ret;

	local_irq_restore(flags);

	return ret;
}

/* ---- rmdir command & result, API ----- */
typedef struct {
	uint32_t ret;
} ll_res_rmdir;

int ll_rmdir(void *ioaddr, char *path)
{
	FSSTATE *fss = (FSSTATE *) ioaddr;
	ll_res_rmdir *res = (ll_res_rmdir *) fss->buffer;
	char *p = (char *)fss->buffer;
	unsigned long flags;
	int ret;

	local_irq_save(flags);

	strcpy(p, path);

	/* execute the command */
	fss->cmdId = FS_RMDIR;
	fss->execute = 1;

	/* Wait for command to finish */
	while (fss->execute == 1)
		/* BUSY LOOP */;

	if (res->ret == -1)
		ret = fss->error;
	else
		ret = res->ret;

	local_irq_restore(flags);

	return ret;
}

/* ---- remove command & result, API ----- */
typedef struct {
	uint32_t ret;
} ll_res_remove;

int ll_remove(void *ioaddr, char *path)
{
	FSSTATE *fss = (FSSTATE *) ioaddr;
	ll_res_remove *res = (ll_res_remove *) fss->buffer;
	char *p = (char *)fss->buffer;
	unsigned long flags;
	int ret;

	local_irq_save(flags);

	strcpy(p, path);

	/* execute the command */
	fss->cmdId = FS_REMOVE;
	fss->execute = 1;

	/* Wait for command to finish */
	while (fss->execute == 1)
		/* BUSY LOOP */;

	if (res->ret == -1)
		ret = fss->error;
	else
		ret = res->ret;

	local_irq_restore(flags);

	return ret;
}

/* ---- access command & result, API ----- */
typedef struct {
	uint32_t mode;
} ll_cmd_access;

typedef struct {
	uint32_t ret;
} ll_res_access;

int ll_access(void *ioaddr, char *path, int mode)
{
	FSSTATE *fss = (FSSTATE *) ioaddr;
	ll_cmd_access *cmd = (ll_cmd_access *) fss->buffer;
	ll_res_access *res = (ll_res_access *) fss->buffer;
	char *p = (char *)(fss->buffer + sizeof(ll_cmd_access));
	unsigned long flags;
	int ret;

	local_irq_save(flags);

	strcpy(p, path);
	cmd->mode = mode;

	/* execute the command */
	fss->cmdId = FS_ACCESS;
	fss->execute = 1;

	/* Wait for command to finish */
	while (fss->execute == 1)
		/* BUSY LOOP */;

	if (res->ret == -1)
		ret = fss->error;
	else
		ret = res->ret;

	local_irq_restore(flags);

	return ret;
}

/* ---- opendir command & result, API ----- */
typedef struct {
	void *dhandle;
} ll_res_opendir;

void *ll_opendir(void *ioaddr, char *path)
{
	FSSTATE *fss = (FSSTATE *) ioaddr;
	ll_res_opendir *res = (ll_res_opendir *) fss->buffer;
	char *p = (char *)fss->buffer;
	unsigned long flags;
	void *ret;

	local_irq_save(flags);

	strcpy(p, path);

	/* execute the command */
	fss->cmdId = FS_OPENDIR;
	fss->execute = 1;

	/* Wait for command to finish */
	while (fss->execute == 1)
		/* BUSY LOOP */;

	ret = res->dhandle;
	local_irq_restore(flags);

	return ret;
}

/* ---- readdir command & result, API ----- */
typedef struct {
	void *dhandle;
	uint32_t pos;
} ll_cmd_readdir;

typedef struct {
	uint32_t ret;
	uint32_t pos;
	uint64_t ino;
} ll_res_readdir;

char *ll_readdir(void *ioaddr, void *dir, long long *ino, long *pos)
{
	FSSTATE *fss = (FSSTATE *) ioaddr;
	ll_res_readdir *res = (ll_res_readdir *) fss->buffer;
	ll_cmd_readdir *cmd = (ll_cmd_readdir *) fss->buffer;
	char *d = (char *)(fss->buffer + sizeof(ll_res_readdir));
	unsigned long flags;
	char *ret;

	local_irq_save(flags);

	cmd->dhandle = dir;

	/* execute the command */
	fss->cmdId = FS_READDIR;
	fss->execute = 1;

	/* Wait for command to finish */
	while (fss->execute == 1)
		/* BUSY LOOP */;

	if (res->ret == 0) {
		*ino = res->ino;
		*pos = res->pos;
		ret = d;
	} else {
		*ino = -1;
		ret = NULL;
	}

	local_irq_restore(flags);

	return ret;
}

/* ---- ll_telldir command & result, API ----- */
typedef struct {
	void *dhandle;
} ll_cmd_telldir;

typedef struct {
	uint32_t pos;
} ll_res_telldir;

long ll_telldir(void *ioaddr, void *dir)
{
	FSSTATE *fss = (FSSTATE *) ioaddr;
	ll_res_telldir *res = (ll_res_telldir *) fss->buffer;
	ll_cmd_telldir *cmd = (ll_cmd_telldir *) fss->buffer;
	unsigned long flags;
	long ret;

	local_irq_save(flags);

	cmd->dhandle = dir;

	/* execute the command */
	fss->cmdId = FS_TELLDIR;
	fss->execute = 1;

	/* Wait for command to finish */
	while (fss->execute == 1)
		/* BUSY LOOP */;

	ret = res->pos;
	local_irq_restore(flags);

	return ret;
}

/* ---- ll_seekdir command & result, API ----- */
typedef struct {
	void *dhandle;
	uint32_t pos;
} ll_cmd_seekdir;

typedef struct {
	uint32_t pos;
} ll_res_seekdir;

long ll_seekdir(void *ioaddr, void *dir, long pos)
{
	FSSTATE *fss = (FSSTATE *) ioaddr;
	ll_res_seekdir *res = (ll_res_seekdir *) fss->buffer;
	ll_cmd_seekdir *cmd = (ll_cmd_seekdir *) fss->buffer;
	unsigned long flags;
	long ret;

	local_irq_save(flags);

	cmd->dhandle = dir;
	cmd->pos = pos;

	/* execute the command */
	fss->cmdId = FS_SEEKDIR;
	fss->execute = 1;

	/* Wait for command to finish */
	while (fss->execute == 1)
		/* BUSY LOOP */;

	ret = res->pos;
	local_irq_restore(flags);

	return ret;
}

/* ---- ll_closedir command & result, API ----- */
typedef struct {
	void *dhandle;
} ll_cmd_closedir;

typedef struct {
	uint32_t ret;
} ll_res_closedir;

long ll_closedir(void *ioaddr, void *dir)
{
	FSSTATE *fss = (FSSTATE *) ioaddr;
	ll_res_closedir *res = (ll_res_closedir *) fss->buffer;
	ll_cmd_closedir *cmd = (ll_cmd_closedir *) fss->buffer;
	unsigned long flags;
	long ret;

	local_irq_save(flags);

	cmd->dhandle = dir;

	/* execute the command */
	fss->cmdId = FS_CLOSEDIR;
	fss->execute = 1;

	/* Wait for command to finish */
	while (fss->execute == 1)
		/* BUSY LOOP */;

	ret = res->ret;
	local_irq_restore(flags);

	return ret;
}

int ll_hostos(void *ioaddr)
{
	FSSTATE *fss = (FSSTATE *) ioaddr;
	return fss->hostos;
}