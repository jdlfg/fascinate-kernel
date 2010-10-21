#ifndef __ASM_ARCH_MAP_ARIES_H
#define __ASM_ARCH_MAP_ARIES_H __FILE__

#define ONEDRAM1G_SHARED_AREA_PHYS  0x35000000
#define ONEDRAM1G_SHARED_AREA_VIRT      0xFC800000
#define ONEDRAM1G_SHARED_AREA_SIZE      (SZ_4M*4)

// reserve framebuffer area (shared with bootloader, PBL/SBL)
#define LOGO_MEM_SIZE       		0x180000
#define LOGO_MEM_BASE		        (ONEDRAM1G_SHARED_AREA_PHYS - LOGO_MEM_SIZE)    // 0x34E80000


#endif /* __ASM_ARCH_MAP_ARIES_H */
