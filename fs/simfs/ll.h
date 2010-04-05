#ifndef _LL_H_
#define _LL_H_

/* ---- stat structure ----- */
typedef struct {
	uint32_t st_dev;
	uint32_t st_mode;
	uint32_t st_nlink;
	uint32_t st_uid;
	uint32_t st_gid;
	uint32_t st_rdev;
	uint32_t st_size;
	uint32_t st_atime;
	uint32_t st_mtime;
	uint32_t st_ctime;
	uint64_t st_ino;
} _ll_stat;

int ll_hostos(void *ioaddr);

int ll_create(void *ioaddr, const char *path);

int ll_open(void *addr, const char *path, int flags);

int ll_close(void *addr, int fd);

int ll_read(void *ioaddr, int fd, void *buffer, uint32_t count);

int ll_write(void *ioaddr, int fd, void *buffer, uint32_t count);

long ll_seek(void *ioaddr, int fd, long offset, int origin);

int ll_stat(void *ioaddr, char *path, _ll_stat * stbuf);

int ll_fstat(void *ioaddr, int fd, _ll_stat * stbuf);

int ll_rename(void *ioaddr, char *oldname, char *newname);

int ll_mkdir(void *ioaddr, char *path);

int ll_rmdir(void *ioaddr, char *path);

int ll_remove(void *ioaddr, char *path);

int ll_access(void *ioaddr, char *path, int mode);

void *ll_opendir(void *ioaddr, char *path);

char *ll_readdir(void *ioaddr, void *dir, long long *ino, long *pos);

long ll_telldir(void *ioaddr, void *dir);

long ll_seekdir(void *ioaddr, void *dir, long pos);

long ll_closedir(void *ioaddr, void *dir);

#endif /* _LL_H_ */
