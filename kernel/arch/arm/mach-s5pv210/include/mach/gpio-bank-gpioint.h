/* linux/arch/arm/plat-s5pc1xx/include/plat/gpio-bank-gpioint.h
 *
 * Copyright 2008 Openmoko, Inc.
 * Copyright 2008 Simtec Electronics
 * 	Ben Dooks <ben@simtec.co.uk>
 * 	http://armlinux.simtec.co.uk/
 *
 * GPIO Bank gpio interrupt register and configuration definitions
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#define S5PV210_GPIOINTCON_BASE		(S5P_VA_GPIO + 0x700)
#define S5PV210_GPIOINTFLT_BASE		(S5P_VA_GPIO + 0x800)
#define S5PV210_GPIOINTMASK_BASE	(S5P_VA_GPIO + 0x900)
#define S5PV210_GPIOINTPEND_BASE	(S5P_VA_GPIO + 0xA00)

#define S5PV210_GPA0INT_CON		(S5PV210_GPIOINTCON_BASE + 0x00)
#define S5PV210_GPA1INT_CON		(S5PV210_GPIOINTCON_BASE + 0x04)
#define S5PV210_GPBINT_CON		(S5PV210_GPIOINTCON_BASE + 0x08)
#define S5PV210_GPC0INT_CON		(S5PV210_GPIOINTCON_BASE + 0x0c)
#define S5PV210_GPC1INT_CON		(S5PV210_GPIOINTCON_BASE + 0x10)
#define S5PV210_GPD0INT_CON		(S5PV210_GPIOINTCON_BASE + 0x14)
#define S5PV210_GPD1INT_CON		(S5PV210_GPIOINTCON_BASE + 0x18)
#define S5PV210_GPE0INT_CON		(S5PV210_GPIOINTCON_BASE + 0x1c)
#define S5PV210_GPE1INT_CON		(S5PV210_GPIOINTCON_BASE + 0x20)
#define S5PV210_GPF0INT_CON		(S5PV210_GPIOINTCON_BASE + 0x24)
#define S5PV210_GPF1INT_CON		(S5PV210_GPIOINTCON_BASE + 0x28)
#define S5PV210_GPF2INT_CON		(S5PV210_GPIOINTCON_BASE + 0x2c)
#define S5PV210_GPF3INT_CON		(S5PV210_GPIOINTCON_BASE + 0x30)
#define S5PV210_GPG0INT_CON		(S5PV210_GPIOINTCON_BASE + 0x34)
#define S5PV210_GPG1INT_CON		(S5PV210_GPIOINTCON_BASE + 0x38)
#define S5PV210_GPG2INT_CON		(S5PV210_GPIOINTCON_BASE + 0x3c)
#define S5PV210_GPG3INT_CON		(S5PV210_GPIOINTCON_BASE + 0x40)
#define S5PV210_GPJ0INT_CON		(S5PV210_GPIOINTCON_BASE + 0x44)
#define S5PV210_GPJ1INT_CON		(S5PV210_GPIOINTCON_BASE + 0x48)
#define S5PV210_GPJ2INT_CON		(S5PV210_GPIOINTCON_BASE + 0x4c)
#define S5PV210_GPJ3INT_CON		(S5PV210_GPIOINTCON_BASE + 0x50)
#define S5PV210_GPJ4INT_CON		(S5PV210_GPIOINTCON_BASE + 0x54)


#define S5PV210_GPA0INT_FLTCON0		(S5PV210_GPIOINTFLT_BASE + 0x00)
#define S5PV210_GPA0INT_FLTCON1		(S5PV210_GPIOINTFLT_BASE + 0x04)
#define S5PV210_GPA1INT_FLTCON0		(S5PV210_GPIOINTFLT_BASE + 0x08)
#define S5PV210_GPA1INT_FLTCON1		(S5PV210_GPIOINTFLT_BASE + 0x0c)
#define S5PV210_GPBINT_FLTCON0		(S5PV210_GPIOINTFLT_BASE + 0x10)
#define S5PV210_GPBINT_FLTCON1		(S5PV210_GPIOINTFLT_BASE + 0x14)
#define S5PV210_GPC0INT_FLTCON0		(S5PV210_GPIOINTFLT_BASE + 0x18)
#define S5PV210_GPC0INT_FLTCON1		(S5PV210_GPIOINTFLT_BASE + 0x1c)
#define S5PV210_GPC1INT_FLTCON0		(S5PV210_GPIOINTFLT_BASE + 0x20)
#define S5PV210_GPC1INT_FLTCON1		(S5PV210_GPIOINTFLT_BASE + 0x24)
#define S5PV210_GPD0INT_FLTCON0		(S5PV210_GPIOINTFLT_BASE + 0x28)
#define S5PV210_GPD0INT_FLTCON1		(S5PV210_GPIOINTFLT_BASE + 0x2c)
#define S5PV210_GPD1INT_FLTCON0		(S5PV210_GPIOINTFLT_BASE + 0x30)
#define S5PV210_GPD1INT_FLTCON1		(S5PV210_GPIOINTFLT_BASE + 0x34)
#define S5PV210_GPE0INT_FLTCON0		(S5PV210_GPIOINTFLT_BASE + 0x38)
#define S5PV210_GPE0INT_FLTCON1		(S5PV210_GPIOINTFLT_BASE + 0x3c)
#define S5PV210_GPE1INT_FLTCON0		(S5PV210_GPIOINTFLT_BASE + 0x40)
#define S5PV210_GPE1INT_FLTCON1		(S5PV210_GPIOINTFLT_BASE + 0x44)
#define S5PV210_GPF0INT_FLTCON0		(S5PV210_GPIOINTFLT_BASE + 0x48)
#define S5PV210_GPF0INT_FLTCON1		(S5PV210_GPIOINTFLT_BASE + 0x4c)
#define S5PV210_GPF1INT_FLTCON0		(S5PV210_GPIOINTFLT_BASE + 0x50)
#define S5PV210_GPF1INT_FLTCON1		(S5PV210_GPIOINTFLT_BASE + 0x54)
#define S5PV210_GPF2INT_FLTCON0		(S5PV210_GPIOINTFLT_BASE + 0x58)
#define S5PV210_GPF2INT_FLTCON1		(S5PV210_GPIOINTFLT_BASE + 0x5c)
#define S5PV210_GPF3INT_FLTCON0		(S5PV210_GPIOINTFLT_BASE + 0x60)
#define S5PV210_GPF3INT_FLTCON1		(S5PV210_GPIOINTFLT_BASE + 0x64)
#define S5PV210_GPG0INT_FLTCON0		(S5PV210_GPIOINTFLT_BASE + 0x68)
#define S5PV210_GPG0INT_FLTCON1		(S5PV210_GPIOINTFLT_BASE + 0x6c)
#define S5PV210_GPG1INT_FLTCON0		(S5PV210_GPIOINTFLT_BASE + 0x70)
#define S5PV210_GPG1INT_FLTCON1		(S5PV210_GPIOINTFLT_BASE + 0x74)
#define S5PV210_GPG2INT_FLTCON0		(S5PV210_GPIOINTFLT_BASE + 0x78)
#define S5PV210_GPG2INT_FLTCON1		(S5PV210_GPIOINTFLT_BASE + 0x7c)
#define S5PV210_GPG3INT_FLTCON0		(S5PV210_GPIOINTFLT_BASE + 0x80)
#define S5PV210_GPG3INT_FLTCON1		(S5PV210_GPIOINTFLT_BASE + 0x84)
#define S5PV210_GPJ0INT_FLTCON0		(S5PV210_GPIOINTFLT_BASE + 0x88)
#define S5PV210_GPJ0INT_FLTCON1		(S5PV210_GPIOINTFLT_BASE + 0x8c)
#define S5PV210_GPJ1INT_FLTCON0		(S5PV210_GPIOINTFLT_BASE + 0x90)
#define S5PV210_GPJ1INT_FLTCON1		(S5PV210_GPIOINTFLT_BASE + 0x94)
#define S5PV210_GPJ2INT_FLTCON0		(S5PV210_GPIOINTFLT_BASE + 0x98)
#define S5PV210_GPJ2INT_FLTCON1		(S5PV210_GPIOINTFLT_BASE + 0x9c)
#define S5PV210_GPJ3INT_FLTCON0		(S5PV210_GPIOINTFLT_BASE + 0xA0)
#define S5PV210_GPJ3INT_FLTCON1		(S5PV210_GPIOINTFLT_BASE + 0xA4)
#define S5PV210_GPJ4INT_FLTCON0		(S5PV210_GPIOINTFLT_BASE + 0xA8)
#define S5PV210_GPJ4INT_FLTCON1		(S5PV210_GPIOINTFLT_BASE + 0xAc)





#define S5PV210_GPA0INT_MASK		(S5PV210_GPIOINTMASK_BASE + 0x00)
#define S5PV210_GPA1INT_MASK		(S5PV210_GPIOINTMASK_BASE + 0x04)
#define S5PV210_GPBINT_MASK		(S5PV210_GPIOINTMASK_BASE + 0x08)
#define S5PV210_GPC0INT_MASK		(S5PV210_GPIOINTMASK_BASE + 0x0c)
#define S5PV210_GPC1INT_MASK		(S5PV210_GPIOINTMASK_BASE + 0x10)
#define S5PV210_GPD0INT_MASK		(S5PV210_GPIOINTMASK_BASE + 0x14)
#define S5PV210_GPD1INT_MASK		(S5PV210_GPIOINTMASK_BASE + 0x18)
#define S5PV210_GPE0INT_MASK		(S5PV210_GPIOINTMASK_BASE + 0x1c)
#define S5PV210_GPE1INT_MASK		(S5PV210_GPIOINTMASK_BASE + 0x20)
#define S5PV210_GPF0INT_MASK		(S5PV210_GPIOINTMASK_BASE + 0x24)
#define S5PV210_GPF1INT_MASK		(S5PV210_GPIOINTMASK_BASE + 0x28)
#define S5PV210_GPF2INT_MASK		(S5PV210_GPIOINTMASK_BASE + 0x2c)
#define S5PV210_GPF3INT_MASK		(S5PV210_GPIOINTMASK_BASE + 0x30)
#define S5PV210_GPG0INT_MASK		(S5PV210_GPIOINTMASK_BASE + 0x34)
#define S5PV210_GPG1INT_MASK		(S5PV210_GPIOINTMASK_BASE + 0x38)
#define S5PV210_GPG2INT_MASK		(S5PV210_GPIOINTMASK_BASE + 0x3c)
#define S5PV210_GPG3INT_MASK		(S5PV210_GPIOINTMASK_BASE + 0x40)
#define S5PV210_GPJ0INT_MASK		(S5PV210_GPIOINTMASK_BASE + 0x44)
#define S5PV210_GPJ1INT_MASK		(S5PV210_GPIOINTMASK_BASE + 0x48)
#define S5PV210_GPJ2INT_MASK		(S5PV210_GPIOINTMASK_BASE + 0x4c)
#define S5PV210_GPJ3INT_MASK		(S5PV210_GPIOINTMASK_BASE + 0x50)
#define S5PV210_GPJ4INT_MASK		(S5PV210_GPIOINTMASK_BASE + 0x54)

#define S5PV210_GPA0INT_PEND		(S5PV210_GPIOINTPEND_BASE + 0x00)
#define S5PV210_GPA1INT_PEND		(S5PV210_GPIOINTPEND_BASE + 0x04)
#define S5PV210_GPBINT_PEND		(S5PV210_GPIOINTPEND_BASE + 0x08)
#define S5PV210_GPC0INT_PEND		(S5PV210_GPIOINTPEND_BASE + 0x0c)
#define S5PV210_GPC1INT_PEND		(S5PV210_GPIOINTPEND_BASE + 0x10)
#define S5PV210_GPD0INT_PEND		(S5PV210_GPIOINTPEND_BASE + 0x14)
#define S5PV210_GPD1INT_PEND		(S5PV210_GPIOINTPEND_BASE + 0x18)
#define S5PV210_GPE0INT_PEND		(S5PV210_GPIOINTPEND_BASE + 0x1c)
#define S5PV210_GPE1INT_PEND		(S5PV210_GPIOINTPEND_BASE + 0x20)
#define S5PV210_GPF0INT_PEND		(S5PV210_GPIOINTPEND_BASE + 0x24)
#define S5PV210_GPF1INT_PEND		(S5PV210_GPIOINTPEND_BASE + 0x28)
#define S5PV210_GPF2INT_PEND		(S5PV210_GPIOINTPEND_BASE + 0x2c)
#define S5PV210_GPF3INT_PEND		(S5PV210_GPIOINTPEND_BASE + 0x30)
#define S5PV210_GPG0INT_PEND		(S5PV210_GPIOINTPEND_BASE + 0x34)
#define S5PV210_GPG1INT_PEND		(S5PV210_GPIOINTPEND_BASE + 0x38)
#define S5PV210_GPG2INT_PEND		(S5PV210_GPIOINTPEND_BASE + 0x3c)
#define S5PV210_GPG3INT_PEND		(S5PV210_GPIOINTPEND_BASE + 0x40)
#define S5PV210_GPJ0INT_PEND		(S5PV210_GPIOINTPEND_BASE + 0x44)
#define S5PV210_GPJ1INT_PEND		(S5PV210_GPIOINTPEND_BASE + 0x48)
#define S5PV210_GPJ2INT_PEND		(S5PV210_GPIOINTPEND_BASE + 0x4c)
#define S5PV210_GPJ3INT_PEND		(S5PV210_GPIOINTPEND_BASE + 0x50)
#define S5PV210_GPJ4INT_PEND		(S5PV210_GPIOINTPEND_BASE + 0x54)

