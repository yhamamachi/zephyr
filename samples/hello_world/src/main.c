/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>

#ifdef CONFIG_BOARD_RCAR_SPARROW_HAWK_R8A779G3_R52
#include <stdint.h>
static inline uint32_t read_mpidr(void)
{
	uint32_t mpidr;
	__asm__ volatile ("mrc p15, 0, %0, c0, c0, 5" : "=r" (mpidr));
	return mpidr;
}
#endif //CONFIG_BOARD_RCAR_SPARROW_HAWK_R8A779G3_R52

int main(void)
{
#ifdef CONFIG_BOARD_RCAR_SPARROW_HAWK_R8A779G3_R52
	uint32_t mpidr = read_mpidr();
	uint32_t core_id = (mpidr >> 8) & 0xff;
	printf("Hello World! %s (core_id=%d)\n", CONFIG_BOARD_TARGET, core_id);
#else
	printf("Hello World! %s\n", CONFIG_BOARD_TARGET);
#endif

	return 0;
}
