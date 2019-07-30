// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2013-2014 Synopsys, Inc. All rights reserved.
 */

#include <common.h>
#include <elf.h>
#include <asm-generic/sections.h>

extern ulong __image_copy_start;
extern ulong __ivt_end;

DECLARE_GLOBAL_DATA_PTR;

int copy_uboot_to_ram(void)
{
	size_t len = (size_t)&__image_copy_end - (size_t)&__image_copy_start;

	if (gd->flags & GD_FLG_SKIP_RELOC)
		return 0;

	memcpy((void *)gd->relocaddr, (void *)&__image_copy_start, len);

	return 0;
}

int clear_bss(void)
{
	ulong dst_addr = (ulong)&__bss_start + gd->reloc_off;
	size_t len = (size_t)&__bss_end - (size_t)&__bss_start;

	memset((void *)dst_addr, 0x00, len);

	return 0;
}

/*
 * Base functionality is taken from x86 version with added ARC-specifics
 */
int do_elf_reloc_fixups(void)
{
	Elf32_Rela *re_src = (Elf32_Rela *)(&__rel_dyn_start);
	Elf32_Rela *re_end = (Elf32_Rela *)(&__rel_dyn_end);

	if (gd->flags & GD_FLG_SKIP_RELOC)
		return 0;

	debug("Section .rela.dyn is located at %08x-%08x\n",
	      (unsigned int)re_src, (unsigned int)re_end);

	Elf32_Addr *offset_ptr_rom, *last_offset = NULL;
	Elf32_Addr *offset_ptr_ram;

	do {
		/* Get the location from the relocation entry */
		offset_ptr_rom = (Elf32_Addr *)re_src->r_offset;

		/* Check that the location of the relocation is in .text */
		if (offset_ptr_rom >= (Elf32_Addr *)&__image_copy_start &&
		    offset_ptr_rom > last_offset) {
			unsigned int val;
			/* Switch to the in-RAM version */
			offset_ptr_ram = (Elf32_Addr *)((ulong)offset_ptr_rom +
							gd->reloc_off);

			debug("Patching value @ %08x (relocated to %08x)\n",
			      (unsigned int)offset_ptr_rom,
			      (unsigned int)offset_ptr_ram);

			/*
			 * Use "memcpy" because target location might be
			 * 16-bit aligned on ARC so we may need to read
			 * byte-by-byte. On attempt to read entire word by
			 * CPU throws an exception
			 */
			memcpy(&val, offset_ptr_ram, sizeof(int));

#ifdef __LITTLE_ENDIAN__
			/* If location in ".text" section swap value */
			if ((unsigned int)offset_ptr_rom <
			    (unsigned int)&__ivt_end)
				val = (val << 16) | (val >> 16);
#endif

			/* Check that the target points into executable */
			if (val >= (unsigned int)&__image_copy_start && val <=
			    (unsigned int)&__image_copy_end) {
				val += gd->reloc_off;
#ifdef __LITTLE_ENDIAN__
				/* If location in ".text" section swap value */
				if ((unsigned int)offset_ptr_rom <
				    (unsigned int)&__ivt_end)
					val = (val << 16) | (val >> 16);
#endif
				memcpy(offset_ptr_ram, &val, sizeof(int));
			}
		}
		last_offset = offset_ptr_rom;

	} while (++re_src < re_end);

	return 0;
}
