/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/printk.h>
#include <inttypes.h>
static void myputc(char c)
{
   /* Wait until HSFSR bit TEND is set, i.e. FIFO is empty */
   while (!(sys_read16(0xe6540000 + 0x10) & (1 << 6)))
       ;

   /* Write character to HSFTDR */
   sys_write8(c, 0xe6540000 + 0x0c);
   /* Clear HSFSR bit TEND */
   sys_write16(sys_read16(0xe6540000 + 0x10) & ~(1 << 6), 0xe6540000 + 0x10);
}
static void myput(char *str) {
    while ( *str != '\0' ) {
        myputc(*str);
        str++;
    }
}


int main(void)
{
        myput("Hello from Zephyr\n\r");
	printf("Hello World! %s\n", CONFIG_BOARD_TARGET);

        myput("Goodbye from Zephyr\n\r");
	return 0;
}
