// SPDX-License-Identifier: GPL-2.0+
/*
 * fat_write.c
 *
 * R/W (V)FAT 12/16/32 filesystem implementation by Donggeun Kim
 */

#include <common.h>
#include <command.h>
#include <config.h>
#include <fat.h>
#include <asm/byteorder.h>
#include <part.h>
#include <linux/ctype.h>
#include <div64.h>
#include <linux/math64.h>
#include "fat.c"

static void uppercase(char *str, int len)
{
	int i;

	for (i = 0; i < len; i++) {
		*str = toupper(*str);
		str++;
	}
}

static int total_sector;
static int disk_write(__u32 block, __u32 nr_blocks, void *buf)
{
	ulong ret;

	if (!cur_dev)
		return -1;

	if (cur_part_info.start + block + nr_blocks >
		cur_part_info.start + total_sector) {
		printf("error: overflow occurs\n");
		return -1;
	}

	ret = blk_dwrite(cur_dev, cur_part_info.start + block, nr_blocks, buf);
	if (nr_blocks && ret == 0)
		return -1;

	return ret;
}

/*
 * Set short name in directory entry
 */
static void set_name(dir_entry *dirent, const char *filename)
{
	char s_name[VFAT_MAXLEN_BYTES];
	char *period;
	int period_location, len, i, ext_num;

	if (filename == NULL)
		return;

	len = strlen(filename);
	if (len == 0)
		return;

	strcpy(s_name, filename);
	uppercase(s_name, len);

	period = strchr(s_name, '.');
	if (period == NULL) {
		period_location = len;
		ext_num = 0;
	} else {
		period_location = period - s_name;
		ext_num = len - period_location - 1;
	}

	/* Pad spaces when the length of file name is shorter than eight */
	if (period_location < 8) {
		memcpy(dirent->name, s_name, period_location);
		for (i = period_location; i < 8; i++)
			dirent->name[i] = ' ';
	} else if (period_location == 8) {
		memcpy(dirent->name, s_name, period_location);
	} else {
		memcpy(dirent->name, s_name, 6);
		dirent->name[6] = '~';
		dirent->name[7] = '1';
	}

	if (ext_num < 3) {
		memcpy(dirent->ext, s_name + period_location + 1, ext_num);
		for (i = ext_num; i < 3; i++)
			dirent->ext[i] = ' ';
	} else
		memcpy(dirent->ext, s_name + period_location + 1, 3);

	debug("name : %s\n", dirent->name);
	debug("ext : %s\n", dirent->ext);
}

static __u8 num_of_fats;
/*
 * Write fat buffer into block device
 */
static int flush_dirty_fat_buffer(fsdata *mydata)
{
	int getsize = FATBUFBLOCKS;
	__u32 fatlength = mydata->fatlength;
	__u8 *bufptr = mydata->fatbuf;
	__u32 startblock = mydata->fatbufnum * FATBUFBLOCKS;

	debug("debug: evicting %d, dirty: %d\n", mydata->fatbufnum,
	      (int)mydata->fat_dirty);

	if ((!mydata->fat_dirty) || (mydata->fatbufnum == -1))
		return 0;

	/* Cap length if fatlength is not a multiple of FATBUFBLOCKS */
	if (startblock + getsize > fatlength)
		getsize = fatlength - startblock;

	startblock += mydata->fat_sect;

	/* Write FAT buf */
	if (disk_write(startblock, getsize, bufptr) < 0) {
		debug("error: writing FAT blocks\n");
		return -1;
	}

	if (num_of_fats == 2) {
		/* Update corresponding second FAT blocks */
		startblock += mydata->fatlength;
		if (disk_write(startblock, getsize, bufptr) < 0) {
			debug("error: writing second FAT blocks\n");
			return -1;
		}
	}
	mydata->fat_dirty = 0;

	return 0;
}

/*
 * Set the file name information from 'name' into 'slotptr',
 */
static int str2slot(dir_slot *slotptr, const char *name, int *idx)
{
	int j, end_idx = 0;

	for (j = 0; j <= 8; j += 2) {
		if (name[*idx] == 0x00) {
			slotptr->name0_4[j] = 0;
			slotptr->name0_4[j + 1] = 0;
			end_idx++;
			goto name0_4;
		}
		slotptr->name0_4[j] = name[*idx];
		(*idx)++;
		end_idx++;
	}
	for (j = 0; j <= 10; j += 2) {
		if (name[*idx] == 0x00) {
			slotptr->name5_10[j] = 0;
			slotptr->name5_10[j + 1] = 0;
			end_idx++;
			goto name5_10;
		}
		slotptr->name5_10[j] = name[*idx];
		(*idx)++;
		end_idx++;
	}
	for (j = 0; j <= 2; j += 2) {
		if (name[*idx] == 0x00) {
			slotptr->name11_12[j] = 0;
			slotptr->name11_12[j + 1] = 0;
			end_idx++;
			goto name11_12;
		}
		slotptr->name11_12[j] = name[*idx];
		(*idx)++;
		end_idx++;
	}

	if (name[*idx] == 0x00)
		return 1;

	return 0;
/* Not used characters are filled with 0xff 0xff */
name0_4:
	for (; end_idx < 5; end_idx++) {
		slotptr->name0_4[end_idx * 2] = 0xff;
		slotptr->name0_4[end_idx * 2 + 1] = 0xff;
	}
	end_idx = 5;
name5_10:
	end_idx -= 5;
	for (; end_idx < 6; end_idx++) {
		slotptr->name5_10[end_idx * 2] = 0xff;
		slotptr->name5_10[end_idx * 2 + 1] = 0xff;
	}
	end_idx = 11;
name11_12:
	end_idx -= 11;
	for (; end_idx < 2; end_idx++) {
		slotptr->name11_12[end_idx * 2] = 0xff;
		slotptr->name11_12[end_idx * 2 + 1] = 0xff;
	}

	return 1;
}

static int is_next_clust(fsdata *mydata, dir_entry *dentptr);
static void flush_dir_table(fsdata *mydata, dir_entry **dentptr);

/*
 * Fill dir_slot entries with appropriate name, id, and attr
 * The real directory entry is returned by 'dentptr'
 */
static void
fill_dir_slot(fsdata *mydata, dir_entry **dentptr, const char *l_name)
{
	__u8 temp_dir_slot_buffer[MAX_LFN_SLOT * sizeof(dir_slot)];
	dir_slot *slotptr = (dir_slot *)temp_dir_slot_buffer;
	__u8 counter = 0, checksum;
	int idx = 0, ret;

	/* Get short file name checksum value */
	checksum = mkcksum((*dentptr)->name, (*dentptr)->ext);

	do {
		memset(slotptr, 0x00, sizeof(dir_slot));
		ret = str2slot(slotptr, l_name, &idx);
		slotptr->id = ++counter;
		slotptr->attr = ATTR_VFAT;
		slotptr->alias_checksum = checksum;
		slotptr++;
	} while (ret == 0);

	slotptr--;
	slotptr->id |= LAST_LONG_ENTRY_MASK;

	while (counter >= 1) {
		if (is_next_clust(mydata, *dentptr)) {
			/* A new cluster is allocated for directory table */
			flush_dir_table(mydata, dentptr);
		}
		memcpy(*dentptr, slotptr, sizeof(dir_slot));
		(*dentptr)++;
		slotptr--;
		counter--;
	}

	if (is_next_clust(mydata, *dentptr)) {
		/* A new cluster is allocated for directory table */
		flush_dir_table(mydata, dentptr);
	}
}

static __u32 dir_curclust;

/*
 * Extract the full long filename starting at 'retdent' (which is really
 * a slot) into 'l_name'. If successful also copy the real directory entry
 * into 'retdent'
 * If additional adjacent cluster for directory entries is read into memory,
 * then 'get_contents_vfatname_block' is copied into 'get_dentfromdir_block' and
 * the location of the real directory entry is returned by 'retdent'
 * Return 0 on success, -1 otherwise.
 */
static int
get_long_file_name(fsdata *mydata, int curclust, __u8 *cluster,
	      dir_entry **retdent, char *l_name)
{
	dir_entry *realdent;
	dir_slot *slotptr = (dir_slot *)(*retdent);
	dir_slot *slotptr2 = NULL;
	__u8 *buflimit = cluster + mydata->sect_size * ((curclust == 0) ?
							PREFETCH_BLOCKS :
							mydata->clust_size);
	__u8 counter = (slotptr->id & ~LAST_LONG_ENTRY_MASK) & 0xff;
	int idx = 0, cur_position = 0;

	if (counter > VFAT_MAXSEQ) {
		debug("Error: VFAT name is too long\n");
		return -1;
	}

	while ((__u8 *)slotptr < buflimit) {
		if (counter == 0)
			break;
		if (((slotptr->id & ~LAST_LONG_ENTRY_MASK) & 0xff) != counter)
			return -1;
		slotptr++;
		counter--;
	}

	if ((__u8 *)slotptr >= buflimit) {
		if (curclust == 0)
			return -1;
		curclust = get_fatent(mydata, dir_curclust);
		if (CHECK_CLUST(curclust, mydata->fatsize)) {
			debug("curclust: 0x%x\n", curclust);
			printf("Invalid FAT entry\n");
			return -1;
		}

		dir_curclust = curclust;

		if (get_cluster(mydata, curclust, get_contents_vfatname_block,
				mydata->clust_size * mydata->sect_size) != 0) {
			debug("Error: reading directory block\n");
			return -1;
		}

		slotptr2 = (dir_slot *)get_contents_vfatname_block;
		while (counter > 0) {
			if (((slotptr2->id & ~LAST_LONG_ENTRY_MASK)
			    & 0xff) != counter)
				return -1;
			slotptr2++;
			counter--;
		}

		/* Save the real directory entry */
		realdent = (dir_entry *)slotptr2;
		while ((__u8 *)slotptr2 > get_contents_vfatname_block) {
			slotptr2--;
			slot2str(slotptr2, l_name, &idx);
		}
	} else {
		/* Save the real directory entry */
		realdent = (dir_entry *)slotptr;
	}

	do {
		slotptr--;
		if (slot2str(slotptr, l_name, &idx))
			break;
	} while (!(slotptr->id & LAST_LONG_ENTRY_MASK));

	l_name[idx] = '\0';
	if (*l_name == DELETED_FLAG)
		*l_name = '\0';
	else if (*l_name == aRING)
		*l_name = DELETED_FLAG;
	downcase(l_name, INT_MAX);

	/* Return the real directory entry */
	*retdent = realdent;

	if (slotptr2) {
		memcpy(get_dentfromdir_block, get_contents_vfatname_block,
			mydata->clust_size * mydata->sect_size);
		cur_position = (__u8 *)realdent - get_contents_vfatname_block;
		*retdent = (dir_entry *) &get_dentfromdir_block[cur_position];
	}

	return 0;
}

/*
 * Set the entry at index 'entry' in a FAT (12/16/32) table.
 */
static int set_fatent_value(fsdata *mydata, __u32 entry, __u32 entry_value)
{
	__u32 bufnum, offset, off16;
	__u16 val1, val2;

	switch (mydata->fatsize) {
	case 32:
		bufnum = entry / FAT32BUFSIZE;
		offset = entry - bufnum * FAT32BUFSIZE;
		break;
	case 16:
		bufnum = entry / FAT16BUFSIZE;
		offset = entry - bufnum * FAT16BUFSIZE;
		break;
	case 12:
		bufnum = entry / FAT12BUFSIZE;
		offset = entry - bufnum * FAT12BUFSIZE;
		break;
	default:
		/* Unsupported FAT size */
		return -1;
	}

	/* Read a new block of FAT entries into the cache. */
	if (bufnum != mydata->fatbufnum) {
		int getsize = FATBUFBLOCKS;
		__u8 *bufptr = mydata->fatbuf;
		__u32 fatlength = mydata->fatlength;
		__u32 startblock = bufnum * FATBUFBLOCKS;

		/* Cap length if fatlength is not a multiple of FATBUFBLOCKS */
		if (startblock + getsize > fatlength)
			getsize = fatlength - startblock;

		if (flush_dirty_fat_buffer(mydata) < 0)
			return -1;

		startblock += mydata->fat_sect;

		if (disk_read(startblock, getsize, bufptr) < 0) {
			debug("Error reading FAT blocks\n");
			return -1;
		}
		mydata->fatbufnum = bufnum;
	}

	/* Mark as dirty */
	mydata->fat_dirty = 1;

	/* Set the actual entry */
	switch (mydata->fatsize) {
	case 32:
		((__u32 *) mydata->fatbuf)[offset] = cpu_to_le32(entry_value);
		break;
	case 16:
		((__u16 *) mydata->fatbuf)[offset] = cpu_to_le16(entry_value);
		break;
	case 12:
		off16 = (offset * 3) / 4;

		switch (offset & 0x3) {
		case 0:
			val1 = cpu_to_le16(entry_value) & 0xfff;
			((__u16 *)mydata->fatbuf)[off16] &= ~0xfff;
			((__u16 *)mydata->fatbuf)[off16] |= val1;
			break;
		case 1:
			val1 = cpu_to_le16(entry_value) & 0xf;
			val2 = (cpu_to_le16(entry_value) >> 4) & 0xff;

			((__u16 *)mydata->fatbuf)[off16] &= ~0xf000;
			((__u16 *)mydata->fatbuf)[off16] |= (val1 << 12);

			((__u16 *)mydata->fatbuf)[off16 + 1] &= ~0xff;
			((__u16 *)mydata->fatbuf)[off16 + 1] |= val2;
			break;
		case 2:
			val1 = cpu_to_le16(entry_value) & 0xff;
			val2 = (cpu_to_le16(entry_value) >> 8) & 0xf;

			((__u16 *)mydata->fatbuf)[off16] &= ~0xff00;
			((__u16 *)mydata->fatbuf)[off16] |= (val1 << 8);

			((__u16 *)mydata->fatbuf)[off16 + 1] &= ~0xf;
			((__u16 *)mydata->fatbuf)[off16 + 1] |= val2;
			break;
		case 3:
			val1 = cpu_to_le16(entry_value) & 0xfff;
			((__u16 *)mydata->fatbuf)[off16] &= ~0xfff0;
			((__u16 *)mydata->fatbuf)[off16] |= (val1 << 4);
			break;
		default:
			break;
		}

		break;
	default:
		return -1;
	}

	return 0;
}

/*
 * Determine the next free cluster after 'entry' in a FAT (12/16/32) table
 * and link it to 'entry'. EOC marker is not set on returned entry.
 */
static __u32 determine_fatent(fsdata *mydata, __u32 entry)
{
	__u32 next_fat, next_entry = entry + 1;

	while (1) {
		next_fat = get_fatent(mydata, next_entry);
		if (next_fat == 0) {
			/* found free entry, link to entry */
			set_fatent_value(mydata, entry, next_entry);
			break;
		}
		next_entry++;
	}
	debug("FAT%d: entry: %08x, entry_value: %04x\n",
	       mydata->fatsize, entry, next_entry);

	return next_entry;
}

/*
 * Write at most 'size' bytes from 'buffer' into the specified cluster.
 * Return 0 on success, -1 otherwise.
 */
static int
set_cluster(fsdata *mydata, __u32 clustnum, __u8 *buffer,
	     unsigned long size)
{
	__u32 idx = 0;
	__u32 startsect;
	int ret;

	if (clustnum > 0)
		startsect = clust_to_sect(mydata, clustnum);
	else
		startsect = mydata->rootdir_sect;

	debug("clustnum: %d, startsect: %d\n", clustnum, startsect);

	if ((unsigned long)buffer & (ARCH_DMA_MINALIGN - 1)) {
		ALLOC_CACHE_ALIGN_BUFFER(__u8, tmpbuf, mydata->sect_size);

		printf("FAT: Misaligned buffer address (%p)\n", buffer);

		while (size >= mydata->sect_size) {
			memcpy(tmpbuf, buffer, mydata->sect_size);
			ret = disk_write(startsect++, 1, tmpbuf);
			if (ret != 1) {
				debug("Error writing data (got %d)\n", ret);
				return -1;
			}

			buffer += mydata->sect_size;
			size -= mydata->sect_size;
		}
	} else if (size >= mydata->sect_size) {
		idx = size / mydata->sect_size;
		ret = disk_write(startsect, idx, buffer);
		if (ret != idx) {
			debug("Error writing data (got %d)\n", ret);
			return -1;
		}

		startsect += idx;
		idx *= mydata->sect_size;
		buffer += idx;
		size -= idx;
	}

	if (size) {
		ALLOC_CACHE_ALIGN_BUFFER(__u8, tmpbuf, mydata->sect_size);

		memcpy(tmpbuf, buffer, size);
		ret = disk_write(startsect, 1, tmpbuf);
		if (ret != 1) {
			debug("Error writing data (got %d)\n", ret);
			return -1;
		}
	}

	return 0;
}

/*
 * Find the first empty cluster
 */
static int find_empty_cluster(fsdata *mydata)
{
	__u32 fat_val, entry = 3;

	while (1) {
		fat_val = get_fatent(mydata, entry);
		if (fat_val == 0)
			break;
		entry++;
	}

	return entry;
}

/*
 * Write directory entries in 'get_dentfromdir_block' to block device
 */
static void flush_dir_table(fsdata *mydata, dir_entry **dentptr)
{
	int dir_newclust = 0;

	if (set_cluster(mydata, dir_curclust,
		    get_dentfromdir_block,
		    mydata->clust_size * mydata->sect_size) != 0) {
		printf("error: wrinting directory entry\n");
		return;
	}
	dir_newclust = find_empty_cluster(mydata);
	set_fatent_value(mydata, dir_curclust, dir_newclust);
	if (mydata->fatsize == 32)
		set_fatent_value(mydata, dir_newclust, 0xffffff8);
	else if (mydata->fatsize == 16)
		set_fatent_value(mydata, dir_newclust, 0xfff8);
	else if (mydata->fatsize == 12)
		set_fatent_value(mydata, dir_newclust, 0xff8);

	dir_curclust = dir_newclust;

	if (flush_dirty_fat_buffer(mydata) < 0)
		return;

	memset(get_dentfromdir_block, 0x00,
		mydata->clust_size * mydata->sect_size);

	*dentptr = (dir_entry *) get_dentfromdir_block;
}

/*
 * Set empty cluster from 'entry' to the end of a file
 */
static int clear_fatent(fsdata *mydata, __u32 entry)
{
	__u32 fat_val;

	while (!CHECK_CLUST(entry, mydata->fatsize)) {
		fat_val = get_fatent(mydata, entry);
		if (fat_val != 0)
			set_fatent_value(mydata, entry, 0);
		else
			break;

		entry = fat_val;
	}

	/* Flush fat buffer */
	if (flush_dirty_fat_buffer(mydata) < 0)
		return -1;

	return 0;
}

/*
 * Write at most 'maxsize' bytes from 'buffer' into
 * the file associated with 'dentptr'
 * Update the number of bytes written in *gotsize and return 0
 * or return -1 on fatal errors.
 */
static int
set_contents(fsdata *mydata, dir_entry *dentptr, __u8 *buffer,
	      loff_t maxsize, loff_t *gotsize)
{
	loff_t filesize = FAT2CPU32(dentptr->size);
	unsigned int bytesperclust = mydata->clust_size * mydata->sect_size;
	__u32 curclust = START(dentptr);
	__u32 endclust = 0, newclust = 0;
	loff_t actsize;

	*gotsize = 0;
	debug("Filesize: %llu bytes\n", filesize);

	if (maxsize > 0 && filesize > maxsize)
		filesize = maxsize;

	debug("%llu bytes\n", filesize);

	if (!curclust) {
		if (filesize) {
			debug("error: nonempty clusterless file!\n");
			return -1;
		}
		return 0;
	}

	actsize = bytesperclust;
	endclust = curclust;
	do {
		/* search for consecutive clusters */
		while (actsize < filesize) {
			newclust = determine_fatent(mydata, endclust);

			if ((newclust - 1) != endclust)
				goto getit;

			if (CHECK_CLUST(newclust, mydata->fatsize)) {
				debug("newclust: 0x%x\n", newclust);
				debug("Invalid FAT entry\n");
				return 0;
			}
			endclust = newclust;
			actsize += bytesperclust;
		}

		/* set remaining bytes */
		actsize = filesize;
		if (set_cluster(mydata, curclust, buffer, (int)actsize) != 0) {
			debug("error: writing cluster\n");
			return -1;
		}
		*gotsize += actsize;

		/* Mark end of file in FAT */
		if (mydata->fatsize == 12)
			newclust = 0xfff;
		else if (mydata->fatsize == 16)
			newclust = 0xffff;
		else if (mydata->fatsize == 32)
			newclust = 0xfffffff;
		set_fatent_value(mydata, endclust, newclust);

		return 0;
getit:
		if (set_cluster(mydata, curclust, buffer, (int)actsize) != 0) {
			debug("error: writing cluster\n");
			return -1;
		}
		*gotsize += actsize;
		filesize -= actsize;
		buffer += actsize;

		if (CHECK_CLUST(newclust, mydata->fatsize)) {
			debug("newclust: 0x%x\n", newclust);
			debug("Invalid FAT entry\n");
			return 0;
		}
		actsize = bytesperclust;
		curclust = endclust = newclust;
	} while (1);
}

/*
 * Set start cluster in directory entry
 */
static void set_start_cluster(const fsdata *mydata, dir_entry *dentptr,
				__u32 start_cluster)
{
	if (mydata->fatsize == 32)
		dentptr->starthi =
			cpu_to_le16((start_cluster & 0xffff0000) >> 16);
	dentptr->start = cpu_to_le16(start_cluster & 0xffff);
}

/*
 * Fill dir_entry
 */
static void fill_dentry(fsdata *mydata, dir_entry *dentptr,
	const char *filename, __u32 start_cluster, __u32 size, __u8 attr)
{
	set_start_cluster(mydata, dentptr, start_cluster);
	dentptr->size = cpu_to_le32(size);

	dentptr->attr = attr;

	set_name(dentptr, filename);
}

/*
 * Check whether adding a file makes the file system to
 * exceed the size of the block device
 * Return -1 when overflow occurs, otherwise return 0
 */
static int check_overflow(fsdata *mydata, __u32 clustnum, loff_t size)
{
	__u32 startsect, sect_num, offset;

	if (clustnum > 0) {
		startsect = clust_to_sect(mydata, clustnum);
	} else {
		startsect = mydata->rootdir_sect;
	}

	sect_num = div_u64_rem(size, mydata->sect_size, &offset);

	if (offset != 0)
		sect_num++;

	if (startsect + sect_num > total_sector)
		return -1;
	return 0;
}

/*
 * Check if adding several entries exceed one cluster boundary
 */
static int is_next_clust(fsdata *mydata, dir_entry *dentptr)
{
	int cur_position;

	cur_position = (__u8 *)dentptr - get_dentfromdir_block;

	if (cur_position >= mydata->clust_size * mydata->sect_size)
		return 1;
	else
		return 0;
}

static dir_entry *empty_dentptr;
/*
 * Find a directory entry based on filename or start cluster number
 * If the directory entry is not found,
 * the new position for writing a directory entry will be returned
 */
static dir_entry *find_directory_entry(fsdata *mydata, int startsect,
	char *filename, dir_entry *retdent, __u32 start)
{
	__u32 curclust = sect_to_clust(mydata, startsect);

	debug("get_dentfromdir: %s\n", filename);

	while (1) {
		dir_entry *dentptr;

		int i;

		if (get_cluster(mydata, curclust, get_dentfromdir_block,
			    mydata->clust_size * mydata->sect_size) != 0) {
			printf("Error: reading directory block\n");
			return NULL;
		}

		dentptr = (dir_entry *)get_dentfromdir_block;

		dir_curclust = curclust;

		for (i = 0; i < DIRENTSPERCLUST; i++) {
			char s_name[14], l_name[VFAT_MAXLEN_BYTES];

			l_name[0] = '\0';
			if (dentptr->name[0] == DELETED_FLAG) {
				dentptr++;
				if (is_next_clust(mydata, dentptr))
					break;
				continue;
			}
			if ((dentptr->attr & ATTR_VOLUME)) {
				if ((dentptr->attr & ATTR_VFAT) &&
				    (dentptr->name[0] & LAST_LONG_ENTRY_MASK)) {
					get_long_file_name(mydata, curclust,
						     get_dentfromdir_block,
						     &dentptr, l_name);
					debug("vfatname: |%s|\n", l_name);
				} else {
					/* Volume label or VFAT entry */
					dentptr++;
					if (is_next_clust(mydata, dentptr))
						break;
					continue;
				}
			}
			if (dentptr->name[0] == 0) {
				debug("Dentname == NULL - %d\n", i);
				empty_dentptr = dentptr;
				return NULL;
			}

			get_name(dentptr, s_name);

			if (strncasecmp(filename, s_name, sizeof(s_name)) &&
			    strncasecmp(filename, l_name, sizeof(l_name))) {
				debug("Mismatch: |%s|%s|\n",
					s_name, l_name);
				dentptr++;
				if (is_next_clust(mydata, dentptr))
					break;
				continue;
			}

			memcpy(retdent, dentptr, sizeof(dir_entry));

			debug("DentName: %s", s_name);
			debug(", start: 0x%x", START(dentptr));
			debug(", size:  0x%x %s\n",
			      FAT2CPU32(dentptr->size),
			      (dentptr->attr & ATTR_DIR) ?
			      "(DIR)" : "");

			return dentptr;
		}

		/*
		 * In FAT16/12, the root dir is locate before data area, shows
		 * in following:
		 * -------------------------------------------------------------
		 * | Boot | FAT1 & 2 | Root dir | Data (start from cluster #2) |
		 * -------------------------------------------------------------
		 *
		 * As a result if curclust is in Root dir, it is a negative
		 * number or 0, 1.
		 *
		 */
		if (mydata->fatsize != 32 && (int)curclust <= 1) {
			/* Current clust is in root dir, set to next clust */
			curclust++;
			if ((int)curclust <= 1)
				continue;	/* continue to find */

			/* Reach the end of root dir */
			empty_dentptr = dentptr;
			return NULL;
		}

		curclust = get_fatent(mydata, dir_curclust);
		if (IS_LAST_CLUST(curclust, mydata->fatsize)) {
			empty_dentptr = dentptr;
			return NULL;
		}
		if (CHECK_CLUST(curclust, mydata->fatsize)) {
			debug("curclust: 0x%x\n", curclust);
			debug("Invalid FAT entry\n");
			return NULL;
		}
	}

	return NULL;
}

static int do_fat_write(const char *filename, void *buffer, loff_t size,
			loff_t *actwrite)
{
	dir_entry *dentptr, *retdent;
	__u32 startsect;
	__u32 start_cluster;
	boot_sector bs;
	volume_info volinfo;
	fsdata datablock;
	fsdata *mydata = &datablock;
	int cursect;
	int ret = -1, name_len;
	char l_filename[VFAT_MAXLEN_BYTES];

	*actwrite = size;
	dir_curclust = 0;

	if (read_bootsectandvi(&bs, &volinfo, &mydata->fatsize)) {
		debug("error: reading boot sector\n");
		return -1;
	}

	total_sector = bs.total_sect;
	if (total_sector == 0)
		total_sector = (int)cur_part_info.size; /* cast of lbaint_t */

	if (mydata->fatsize == 32)
		mydata->fatlength = bs.fat32_length;
	else
		mydata->fatlength = bs.fat_length;

	mydata->fat_sect = bs.reserved;

	cursect = mydata->rootdir_sect
		= mydata->fat_sect + mydata->fatlength * bs.fats;
	num_of_fats = bs.fats;

	mydata->sect_size = (bs.sector_size[1] << 8) + bs.sector_size[0];
	mydata->clust_size = bs.cluster_size;

	if (mydata->fatsize == 32) {
		mydata->data_begin = mydata->rootdir_sect -
					(mydata->clust_size * 2);
	} else {
		int rootdir_size;

		rootdir_size = ((bs.dir_entries[1]  * (int)256 +
				 bs.dir_entries[0]) *
				 sizeof(dir_entry)) /
				 mydata->sect_size;
		mydata->data_begin = mydata->rootdir_sect +
					rootdir_size -
					(mydata->clust_size * 2);
	}

	mydata->fatbufnum = -1;
	mydata->fat_dirty = 0;
	mydata->fatbuf = memalign(ARCH_DMA_MINALIGN, FATBUFSIZE);
	if (mydata->fatbuf == NULL) {
		debug("Error: allocating memory\n");
		return -1;
	}

	if (disk_read(cursect,
		(mydata->fatsize == 32) ?
		(mydata->clust_size) :
		PREFETCH_BLOCKS, do_fat_read_at_block) < 0) {
		debug("Error: reading rootdir block\n");
		goto exit;
	}
	dentptr = (dir_entry *) do_fat_read_at_block;

	name_len = strlen(filename);
	if (name_len >= VFAT_MAXLEN_BYTES)
		name_len = VFAT_MAXLEN_BYTES - 1;

	memcpy(l_filename, filename, name_len);
	l_filename[name_len] = 0; /* terminate the string */
	downcase(l_filename, INT_MAX);

	startsect = mydata->rootdir_sect;
	retdent = find_directory_entry(mydata, startsect,
				l_filename, dentptr, 0);
	if (retdent) {
		/* Update file size and start_cluster in a directory entry */
		retdent->size = cpu_to_le32(size);
		start_cluster = START(retdent);

		if (start_cluster) {
			if (size) {
				ret = check_overflow(mydata, start_cluster,
							size);
				if (ret) {
					printf("Error: %llu overflow\n", size);
					goto exit;
				}
			}

			ret = clear_fatent(mydata, start_cluster);
			if (ret) {
				printf("Error: clearing FAT entries\n");
				goto exit;
			}

			if (!size)
				set_start_cluster(mydata, retdent, 0);
		} else if (size) {
			ret = start_cluster = find_empty_cluster(mydata);
			if (ret < 0) {
				printf("Error: finding empty cluster\n");
				goto exit;
			}

			ret = check_overflow(mydata, start_cluster, size);
			if (ret) {
				printf("Error: %llu overflow\n", size);
				goto exit;
			}

			set_start_cluster(mydata, retdent, start_cluster);
		}
	} else {
		/* Set short name to set alias checksum field in dir_slot */
		set_name(empty_dentptr, filename);
		fill_dir_slot(mydata, &empty_dentptr, filename);

		if (size) {
			ret = start_cluster = find_empty_cluster(mydata);
			if (ret < 0) {
				printf("Error: finding empty cluster\n");
				goto exit;
			}

			ret = check_overflow(mydata, start_cluster, size);
			if (ret) {
				printf("Error: %llu overflow\n", size);
				goto exit;
			}
		} else {
			start_cluster = 0;
		}

		/* Set attribute as archieve for regular file */
		fill_dentry(mydata, empty_dentptr, filename,
			start_cluster, size, 0x20);

		retdent = empty_dentptr;
	}

	ret = set_contents(mydata, retdent, buffer, size, actwrite);
	if (ret < 0) {
		printf("Error: writing contents\n");
		goto exit;
	}
	debug("attempt to write 0x%llx bytes\n", *actwrite);

	/* Flush fat buffer */
	ret = flush_dirty_fat_buffer(mydata);
	if (ret) {
		printf("Error: flush fat buffer\n");
		goto exit;
	}

	/* Write directory table to device */
	ret = set_cluster(mydata, dir_curclust, get_dentfromdir_block,
			mydata->clust_size * mydata->sect_size);
	if (ret)
		printf("Error: writing directory entry\n");

exit:
	free(mydata->fatbuf);
	return ret;
}

int file_fat_write(const char *filename, void *buffer, loff_t offset,
		   loff_t maxsize, loff_t *actwrite)
{
	if (offset != 0) {
		printf("Error: non zero offset is currently not supported.\n");
		return -1;
	}

	printf("writing %s\n", filename);
	return do_fat_write(filename, buffer, maxsize, actwrite);
}
