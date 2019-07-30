// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2014 Google, Inc
 *
 * Memory Type Range Regsters - these are used to tell the CPU whether
 * memory is cacheable and if so the cache write mode to use.
 *
 * These can speed up booting. See the mtrr command.
 *
 * Reference: Intel Architecture Software Developer's Manual, Volume 3:
 * System Programming
 */

#include <common.h>
#include <asm/io.h>
#include <asm/msr.h>
#include <asm/mtrr.h>

DECLARE_GLOBAL_DATA_PTR;

/* Prepare to adjust MTRRs */
void mtrr_open(struct mtrr_state *state)
{
	if (!gd->arch.has_mtrr)
		return;

	state->enable_cache = dcache_status();

	if (state->enable_cache)
		disable_caches();
	state->deftype = native_read_msr(MTRR_DEF_TYPE_MSR);
	wrmsrl(MTRR_DEF_TYPE_MSR, state->deftype & ~MTRR_DEF_TYPE_EN);
}

/* Clean up after adjusting MTRRs, and enable them */
void mtrr_close(struct mtrr_state *state)
{
	if (!gd->arch.has_mtrr)
		return;

	wrmsrl(MTRR_DEF_TYPE_MSR, state->deftype | MTRR_DEF_TYPE_EN);
	if (state->enable_cache)
		enable_caches();
}

int mtrr_commit(bool do_caches)
{
	struct mtrr_request *req = gd->arch.mtrr_req;
	struct mtrr_state state;
	uint64_t mask;
	int i;

	if (!gd->arch.has_mtrr)
		return -ENOSYS;

	mtrr_open(&state);
	for (i = 0; i < gd->arch.mtrr_req_count; i++, req++) {
		mask = ~(req->size - 1);
		mask &= (1ULL << CONFIG_CPU_ADDR_BITS) - 1;
		wrmsrl(MTRR_PHYS_BASE_MSR(i), req->start | req->type);
		wrmsrl(MTRR_PHYS_MASK_MSR(i), mask | MTRR_PHYS_MASK_VALID);
	}

	/* Clear the ones that are unused */
	for (; i < MTRR_COUNT; i++)
		wrmsrl(MTRR_PHYS_MASK_MSR(i), 0);
	mtrr_close(&state);

	return 0;
}

int mtrr_add_request(int type, uint64_t start, uint64_t size)
{
	struct mtrr_request *req;
	uint64_t mask;

	if (!gd->arch.has_mtrr)
		return -ENOSYS;

	if (gd->arch.mtrr_req_count == MAX_MTRR_REQUESTS)
		return -ENOSPC;
	req = &gd->arch.mtrr_req[gd->arch.mtrr_req_count++];
	req->type = type;
	req->start = start;
	req->size = size;
	debug("%d: type=%d, %08llx  %08llx\n", gd->arch.mtrr_req_count - 1,
	      req->type, req->start, req->size);
	mask = ~(req->size - 1);
	mask &= (1ULL << CONFIG_CPU_ADDR_BITS) - 1;
	mask |= MTRR_PHYS_MASK_VALID;
	debug("   %016llx %016llx\n", req->start | req->type, mask);

	return 0;
}
