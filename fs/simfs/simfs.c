#include <linux/module.h>
#include <linux/fs.h>
#include <linux/pagemap.h>
#include <linux/highmem.h>
#include <linux/time.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/ramfs.h>
#include <linux/sched.h>
#include <linux/parser.h>
#include <linux/mutex.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include "ll.h"

/* ---- simfs magic ---- */
#define SIMFS_MAGIC         0xBABABABA

static void *SIMFS_virtAddr;

/* Insmod parameter */
static char * WINDOWS_ROOTPATH = "c:\\temp";
module_param (WINDOWS_ROOTPATH, charp, S_IRUGO);
static char * LINUX_ROOTPATH = "/tmp";
module_param (LINUX_ROOTPATH, charp, S_IRUGO);

/* ---- simfs inode type ---- */
typedef struct {
	struct inode vfs_inode;
	int fd;
	int mode;
	char *name;
	void *ioaddr;
} simfs_inode_t;

/* */
atomic_t inode_counter;

/* ---- Return simfs inode from an inode ---- */
static inline simfs_inode_t *SIMFS_I(struct inode *inode)
{
	return list_entry(inode, simfs_inode_t, vfs_inode);
}

/* ---- Return simfs inode from file ptr ---- */
#define FILE_SIMFS_I(file) SIMFS_I((file)->f_path.dentry->d_inode)

static struct inode *simfs_inode_get(struct super_block *sb, long ino);
static int simfs_read_name(struct inode *ino, char *name);

/* ---- Returns sim name for a dentry ---- */
static char *simfs_dentry_name(struct dentry *dentry, int extra)
{
	struct dentry *parent;
	char *root;
	char *name;
	int len;

	len = 0;
	parent = dentry;
	while (parent->d_parent != parent) {
		len += parent->d_name.len + 1;
		parent = parent->d_parent;
	}

	root = SIMFS_I(parent->d_inode)->name;
	len += strlen(root);
	name = kmalloc(len + 1 + extra, GFP_KERNEL);
	if (name == NULL)
		return NULL;

	name[len] = '\0';
	parent = dentry;
	while (parent->d_parent != parent) {
		len -= parent->d_name.len + 1;
		name[len] = '\\';
		strncpy(&name[len + 1], parent->d_name.name,
			parent->d_name.len);
		parent = parent->d_parent;
	}
	strncpy(name, root, strlen(root));
	return name;
}

/* ---- Returns sim name for an inode ---- */
static char *simfs_inode_name(struct inode *ino, int extra)
{
	struct dentry *dentry;

	dentry = list_entry(ino->i_dentry.next, struct dentry, d_alias);

	return simfs_dentry_name(dentry, extra);
}

/* ---- file open ---- */
int simfs_file_open(struct inode *ino, struct file *file)
{
	char *name;
	fmode_t mode = 0;
	int r = 0, w = 0, fd;

	mode = file->f_mode & (FMODE_READ | FMODE_WRITE);
	if ((mode & SIMFS_I(ino)->mode) == mode)
		return 0;

	/*
	 * The file may already have been opened, but with the wrong access,
	 * so this resets things and reopens the file with the new access.
	 */
	if (SIMFS_I(ino)->fd != -1) {
		ll_close(SIMFS_I(ino)->ioaddr, SIMFS_I(ino)->fd);
		SIMFS_I(ino)->fd = -1;
	}

	SIMFS_I(ino)->mode |= mode;
	if (SIMFS_I(ino)->mode & FMODE_READ)
		r = 1;
	if (SIMFS_I(ino)->mode & FMODE_WRITE)
		w = 1;
	if (w)
		r = 1;

	name = simfs_dentry_name(file->f_path.dentry, 0);
	if (name == NULL)
		return -ENOMEM;

	fd = ll_open(SIMFS_I(ino)->ioaddr, name, O_RDWR);
	kfree(name);
	if (fd < 0)
		return fd;
	FILE_SIMFS_I(file)->fd = fd;

	return 0;
}

int simfs_file_close(struct inode *ino, struct file *file)
{
	return ll_close(SIMFS_I(ino)->ioaddr, SIMFS_I(ino)->fd);
}

/* ---- file file operations ---- */
static const struct file_operations simfs_f_fops = {
	.llseek = generic_file_llseek,
	.read = do_sync_read,
	.splice_read = generic_file_splice_read,
	.aio_read = generic_file_aio_read,
	.aio_write = generic_file_aio_write,
	.write = do_sync_write,
	.mmap = generic_file_mmap,
	.open = simfs_file_open,
	.release = simfs_file_close,
};

/* ---- dir file operations ---- */
int simfs_readdir(struct file *file, void *ent, filldir_t filldir)
{
	void *dir;
	char *name;
	unsigned long long next, ino;
	int error, len;

	name = simfs_dentry_name(file->f_path.dentry, 0);
	if (name == NULL)
		return -ENOMEM;
	dir = ll_opendir(FILE_SIMFS_I(file)->ioaddr, name);
	kfree(name);
	if (dir == NULL)
		return -EINVAL;

	next = file->f_pos;
	while (1) {
		error = ll_seekdir(FILE_SIMFS_I(file)->ioaddr, dir, next);
		if (error < 0)
			break;

		name = ll_readdir(FILE_SIMFS_I(file)->ioaddr,
				  dir, (long long *)&ino, (long *)&next);
		if (!name)
			break;

		len = strlen(name);
		if (strcmp(name, ".") == 0) {
			if (filldir(ent, ".", 1, 0,
				    file->f_path.dentry->d_inode->i_ino,
				    DT_DIR) < 0)
				goto out;
		}

		else if (strcmp(name, "..") == 0) {
			if (filldir(ent, "..", 2, 0,
				    parent_ino(file->f_path.dentry),
				    DT_DIR) < 0)
				goto out;
		} else {
			error = (*filldir) (ent, name, len, file->f_pos,
					    ino, DT_UNKNOWN);
			if (error < 0)
				break;
		}

		file->f_pos = next;
	}
out:
	ll_closedir(FILE_SIMFS_I(file)->ioaddr, dir);
	return 0;
}

/* ---- dir file operations ---- */
static const struct file_operations simfs_d_fops = {
	.llseek = generic_file_llseek,
	.readdir = simfs_readdir,
	.read = generic_read_dir,
};

/* ---- create a new inode ---- */
int simfs_create(struct inode *dir, struct dentry *dentry, int mode,
		 struct nameidata *nd)
{
	struct inode *inode;
	char *name;
	int error, fd;

	inode = simfs_inode_get(dir->i_sb, 0);
	if (IS_ERR(inode)) {
		error = PTR_ERR(inode);
		goto out;
	}

	error = -ENOMEM;
	name = simfs_dentry_name(dentry, 0);
	if (name == NULL)
		goto out_put;

	fd = ll_create(SIMFS_I(dir)->ioaddr, (const char *)name);
	if (fd < 0)
		error = fd;
	else
		error = simfs_read_name(inode, name);

	kfree(name);
	if (error)
		goto out_put;

	SIMFS_I(inode)->fd = fd;
	SIMFS_I(inode)->mode = FMODE_READ | FMODE_WRITE;
	d_instantiate(dentry, inode);
	return 0;

out_put:
	iput(inode);
out:
	return error;
}

static int simfs_d_delete(struct dentry *dentry)
{
	return 1;
}

static const struct dentry_operations simfs_dentry_ops = {
	.d_delete = simfs_d_delete,
};

/* ---- find a inode ---- */
struct dentry *simfs_lookup(struct inode *ino, struct dentry *dentry,
			    struct nameidata *nd)
{
	struct inode *inode;
	char *name;
	int err;

	inode = simfs_inode_get(ino->i_sb, 0);
	if (IS_ERR(inode)) {
		err = PTR_ERR(inode);
		goto out;
	}

	err = -ENOMEM;
	name = simfs_dentry_name(dentry, 0);
	if (name == NULL)
		goto out_put;

	err = simfs_read_name(inode, name);
	kfree(name);
	if (err == -ENOENT) {
		iput(inode);
		inode = NULL;
	} else if (err)
		goto out_put;

	d_add(dentry, inode);
	dentry->d_op = &simfs_dentry_ops;
	return NULL;

out_put:
	iput(inode);
out:
	return ERR_PTR(err);
}

/* ---- return the name  ---- */
static char *inode_dentry_name(struct inode *ino, struct dentry *dentry)
{
	char *file;
	int len;

	file = simfs_inode_name(ino, dentry->d_name.len + 1);
	if (file == NULL)
		return NULL;
	strcat(file, "\\");
	len = strlen(file);
	strncat(file, dentry->d_name.name, dentry->d_name.len);
	file[len + dentry->d_name.len] = '\0';
	return file;
}

/* ---- create a new dir  ---- */
int simfs_mkdir(struct inode *ino, struct dentry *dentry, int mode)
{
	char *file;
	int err;

	file = inode_dentry_name(ino, dentry);
	if (file == NULL)
		return -ENOMEM;
	err = ll_mkdir(SIMFS_I(ino)->ioaddr, file);
	kfree(file);
	return err;
}

/* ---- remove a dir ---- */
int simfs_rmdir(struct inode *ino, struct dentry *dentry)
{
	char *file;
	int err;
	_ll_stat st;

	if ((file = inode_dentry_name(ino, dentry)) == NULL)
		return -ENOMEM;
	err = ll_stat (SIMFS_I(ino)->ioaddr, file, &st);
	if (err < 0)
		return err;

	if (S_ISDIR(st.st_mode))
		err = ll_rmdir (SIMFS_I(ino)->ioaddr, file);
	else
		err = ll_remove (SIMFS_I(ino)->ioaddr, file);

	kfree(file);
	return err;
}

/* ---- rename ---- */
int simfs_rename(struct inode *from_ino, struct dentry *from,
		 struct inode *to_ino, struct dentry *to)
{
	char *from_name, *to_name;
	int err;

	from_name = inode_dentry_name(from_ino, from);
	if (from_name == NULL)
		return -ENOMEM;
	to_name = inode_dentry_name(to_ino, to);
	if (to_name == NULL) {
		kfree(from_name);
		return -ENOMEM;
	}
	err = ll_rename(SIMFS_I(from_ino)->ioaddr, from_name, to_name);
	kfree(from_name);
	kfree(to_name);
	return err;
}

/* ---- dir inode operations ---- */
static const struct inode_operations simfs_d_iops = {
	.create = simfs_create,
	.lookup = simfs_lookup,
	.mkdir = simfs_mkdir,
	.rmdir = simfs_rmdir,
	.rename = simfs_rename,
};

/* ---- file inode operations ---- */
static const struct inode_operations simfs_f_iops = {
	.create = simfs_create,
	.lookup = simfs_lookup,
	.mkdir = simfs_mkdir,
	.rmdir = simfs_rmdir,
	.rename = simfs_rename,
};

/* ---- write page ---- */
int simfs_writepage(struct page *page, struct writeback_control *wbc)
{
    struct address_space * mapping = page->mapping;
    struct inode *         inode = mapping->host;
    char *                 buffer;
    unsigned long long     base, temp;
    int                    count = PAGE_CACHE_SIZE;
    int                    end_index = inode->i_size >> PAGE_CACHE_SHIFT;
    int err;
    unsigned long phys;

    if (page->index >= end_index)
        count = inode->i_size & (PAGE_CACHE_SIZE-1);

    buffer = kmap(page);
    base = ((unsigned long long) page->index) << PAGE_CACHE_SHIFT;

    temp = ll_seek (SIMFS_I(inode)->ioaddr, SIMFS_I(inode)->fd, base, SEEK_SET);
    if (temp != base) {
        err = -EINVAL;
        goto out;
    }
    else {
        base = temp;
    }

    phys = (page_to_pfn (page) << PAGE_SHIFT);
    err = ll_write (SIMFS_I(inode)->ioaddr, SIMFS_I(inode)->fd, (void *) phys, count);
    if (err != count) {
        ClearPageUptodate(page);
        goto out;
    }

    if (base > inode->i_size)
        inode->i_size = (base + count);

    if (PageError(page))
        ClearPageError(page);
    err = 0;

 out:
    kunmap(page);

    unlock_page(page);
    return err;
}

/* ---- readpage operations ---- */
int simfs_readpage(struct file *file, struct page *page)
{
    char *    buffer;
    long long start;
    int       err = 0;
    unsigned long long temp;
    unsigned long phys;

    start = (long long) page->index << PAGE_CACHE_SHIFT;
    buffer = kmap(page);

    temp = ll_seek (FILE_SIMFS_I(file)->ioaddr, FILE_SIMFS_I(file)->fd, start, SEEK_SET);
    if (temp != start) {
        goto out;
    }

    phys = (page_to_pfn (page) << PAGE_SHIFT);
    err = ll_read (FILE_SIMFS_I(file)->ioaddr,
                   FILE_SIMFS_I(file)->fd,
                   (void *) phys,
                   PAGE_CACHE_SIZE);
    if (err < 0)
        goto out;

    memset(&buffer[err], 0, PAGE_CACHE_SIZE - err);

    flush_dcache_page(page);
    SetPageUptodate(page);
    if (PageError(page)) ClearPageError(page);
    err = 0;

 out:
    kunmap(page);
    unlock_page(page);
    return err;
}

/* ---- write begin ---- */
int simfs_write_begin(struct file *file, struct address_space *mapping,
		      loff_t pos, unsigned len, unsigned flags,
		      struct page **pagep, void **fsdata)
{
	pgoff_t index = pos >> PAGE_CACHE_SHIFT;

	*pagep = grab_cache_page_write_begin(mapping, index, flags);
	if (!*pagep)
		return -ENOMEM;
	return 0;
}

/* ---- write end ---- */
int simfs_write_end(struct file *file, struct address_space *mapping,
		    loff_t pos, unsigned len, unsigned copied,
		    struct page *page, void *fsdata)
{
    struct inode *inode = mapping->host;
    void *buffer;
    unsigned from = pos & (PAGE_CACHE_SIZE - 1);
    int err;
    unsigned long long temp;
    unsigned long phys;

    buffer = kmap(page);
    temp = ll_seek (FILE_SIMFS_I(file)->ioaddr,
                    FILE_SIMFS_I(file)->fd,
                    pos,
                    SEEK_SET);
    if (temp != pos) {
        return -EINVAL;
    }

    phys = (page_to_pfn (page) << PAGE_SHIFT);
    err = ll_write (FILE_SIMFS_I(file)->ioaddr,
                    FILE_SIMFS_I(file)->fd,
                    (void *) (phys + from),
                    copied);
    kunmap(page);

    if (!PageUptodate(page) && err == PAGE_CACHE_SIZE)
        SetPageUptodate(page);

    /*
     * If err > 0, write_file has added err to pos, so we are comparing
     * i_size against the last byte written.
     */
    if (err > 0 && (pos > inode->i_size))
        inode->i_size = (pos + copied);
    unlock_page(page);
    page_cache_release(page);

    return err;
}

/* ---- address operations ---- */
static const struct address_space_operations simfs_aops = {
	.writepage = simfs_writepage,
	.readpage = simfs_readpage,
	.set_page_dirty = __set_page_dirty_nobuffers,
	.write_begin = simfs_write_begin,
	.write_end = simfs_write_end,
};

/* ---- Reads path stats and fills the inode ---- */
static int simfs_read_name(struct inode *ino, char *name)
{
	int err;
	_ll_stat st;

	err = ll_stat(SIMFS_I(ino)->ioaddr, name, &st);
	if (err < 0)
		return err;

	if (S_ISDIR(st.st_mode)) {
		ino->i_op = &simfs_d_iops;
		ino->i_fop = &simfs_d_fops;
	} else {
		ino->i_op = &simfs_f_iops;
		ino->i_fop = &simfs_f_fops;
	}

	ino->i_mapping->a_ops = &simfs_aops;

	/* Copy the stat to inode */
	ino->i_ino = st.st_ino;
	ino->i_mode = st.st_mode;
	ino->i_nlink = st.st_nlink;
	ino->i_size = st.st_size;
	ino->i_blocks = st.st_size / PAGE_CACHE_SIZE;
	ino->i_gid = current_fsgid();
	ino->i_uid = current_fsuid();
	ino->i_ctime.tv_sec = st.st_ctime;
	ino->i_ctime.tv_nsec = 0;
	ino->i_atime.tv_sec = st.st_atime;
	ino->i_atime.tv_nsec = 0;
	ino->i_mtime.tv_sec = st.st_mtime;
	ino->i_mtime.tv_nsec = 0;

	return 0;
}

/* ---- Reads inode info from sim ip ---- */
static int simfs_read_inode(struct inode *ino)
{
	char *name;
	int err = 0;

	/*
	 * Unfortunately, we are called from iget() when we don't have a dentry
	 * allocated yet.
	 */
	if (list_empty(&ino->i_dentry))
		goto out;

	err = -ENOMEM;
	name = simfs_inode_name(ino, 0);
	if (name == NULL)
		goto out;

	err = simfs_read_name(ino, name);
	kfree(name);
out:
	return err;
}

/* ---- Allocates new inode and initialize it ---- */
static struct inode *simfs_inode_get(struct super_block *sb, long ino)
{
	struct inode *inode;
	long ret;

	inode = iget_locked(sb, ino);
	if (!inode)
		return ERR_PTR(-ENOMEM);
	if (inode->i_state & I_NEW) {
		ret = simfs_read_inode(inode);
		if (ret < 0) {
			iget_failed(inode);
			return ERR_PTR(ret);
		}
		unlock_new_inode(inode);
	}
	return inode;
}

/* ---- Allocates a simfs inode ---- */
static struct inode *simfs_alloc_inode(struct super_block *sb)
{
	simfs_inode_t *si;

	si = kmalloc(sizeof(simfs_inode_t), GFP_KERNEL);
	if (si == NULL)
		return NULL;

	si->name = NULL;
	si->fd = -1;
	si->mode = 0;
	si->ioaddr = (void *)SIMFS_virtAddr;	/* FS io address */

	inode_init_once(&si->vfs_inode);

	return &si->vfs_inode;
}

/* ---- deletes a simfs inode  ---- */
static void simfs_delete_inode(struct inode *inode)
{
	truncate_inode_pages(&inode->i_data, 0);
	if (SIMFS_I(inode)->fd != -1) {
		ll_close(SIMFS_I(inode)->ioaddr, SIMFS_I(inode)->fd);
		SIMFS_I(inode)->fd = -1;
	}
	clear_inode(inode);
}

/* ---- destroys a simfs inode ---- */
static void simfs_destroy_inode(struct inode *inode)
{
    kfree(SIMFS_I(inode)->name);
    if (SIMFS_I(inode)->fd != -1) {
        ll_close (SIMFS_I(inode)->ioaddr, SIMFS_I(inode)->fd);
        SIMFS_I(inode)->fd = -1;
        printk(KERN_DEBUG "Closing sim fd in .destroy_inode\n");
    }
    kfree(SIMFS_I(inode));
}

/* ---- super block operations ---- */
static const struct super_operations simfs_s_ops = {
	.alloc_inode = simfs_alloc_inode,
	.drop_inode = generic_delete_inode,
	.delete_inode = simfs_delete_inode,
	.destroy_inode = simfs_destroy_inode,
};

/* ---- Fill super block ---- */
static int simfs_fill_sb_common(struct super_block *sb, void *d, int silent)
{
    char *         rpath;
    struct inode * inode;
    int            err;

    sb->s_blocksize      = PAGE_CACHE_SIZE;
    sb->s_blocksize_bits = PAGE_CACHE_SHIFT;
    sb->s_magic          = SIMFS_MAGIC;
    sb->s_op             = &simfs_s_ops;

    /* Populate the root inode */
    inode = simfs_inode_get (sb, 0);
    if (!inode)
        return -EINVAL;

    if (strlen ((char *)d) == 0) {
		if (ll_hostos(SIMFS_virtAddr))
			rpath = WINDOWS_ROOTPATH;
		else
			rpath = LINUX_ROOTPATH;
    }
    else {
        rpath = (char *) d;
    }

    /* write the name of this inode */
    SIMFS_I(inode)->name = (char *) kmalloc (strlen(rpath) + 1, GFP_KERNEL);
    if (!SIMFS_I(inode)->name) {
        iput (inode);
        return -EINVAL;
    }

    strcpy (SIMFS_I(inode)->name, rpath);

    sb->s_root = d_alloc_root(inode);
    if (sb->s_root == NULL) {
        kfree (SIMFS_I(inode)->name);
        iput (inode);
        return -EINVAL;
    }

    err = simfs_read_inode (inode);
    if (err < 0) {
        dput (sb->s_root);
        sb->s_root = NULL;
        kfree (SIMFS_I(inode)->name);
        iput (inode);
        return -EINVAL;
    }

    return 0;
}

/* ---- Read super block ---- */
static int simfs_read_sb(struct file_system_type *type,
			 int flags, const char *dev_name,
			 void *data, struct vfsmount *mnt)
{
	atomic_set(&inode_counter, 0);
	return get_sb_nodev(type, flags, data, simfs_fill_sb_common, mnt);
}

/* ---- Filesystem struct ---- */
static struct file_system_type simfs_type = {
	.owner = THIS_MODULE,
	.name = "simfs",
	.get_sb = simfs_read_sb,
	.kill_sb = kill_anon_super,
	.fs_flags = 0,
};

/* ---- Module startup & cleanup ---- */
static int __init init_simfs(void)
{
	SIMFS_virtAddr = ioremap(CONFIG_SIMFS_PHYSADDR, 0x10000);
	if (SIMFS_virtAddr == NULL) {
		printk(KERN_ERR "Failed to map %x to kernel virtual space\n",
		       CONFIG_SIMFS_PHYSADDR);
		return -1;
	}
	return register_filesystem(&simfs_type);
}

static void __exit exit_simfs(void)
{
	iounmap(SIMFS_virtAddr);
	unregister_filesystem(&simfs_type);
}

module_init(init_simfs)
    module_exit(exit_simfs)
    MODULE_LICENSE("GPL");
