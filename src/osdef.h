/* $XFree86: xc/programs/Xserver/hw/xfree86/drivers/sis/osdef.h,v 1.5 2003/10/30 18:53:42 twini Exp $ */

/* OS depending defines */

/* The choices are: */
/* #define LINUX_KERNEL	 */  	/* Kernel framebuffer */
#define LINUX_XF86    		/* XFree86 */

/**********************************************************************/
#ifdef LINUX_KERNEL /* ----------------------------*/
#include <linux/config.h>

#ifdef CONFIG_FB_SIS_300
#define SIS300
#endif

#ifdef CONFIG_FB_SIS_315
#define SIS315H
#endif

#if 1
#define SISFBACCEL	/* Include 2D acceleration */
#endif

#endif

#ifdef LINUX_XF86 /* ----------------------------- */
#define SIS300
#define SIS315H
#endif

/**********************************************************************/
#ifdef LINUX_KERNEL
#define SiS_SetMemory(MemoryAddress,MemorySize,value) memset(MemoryAddress, value, MemorySize)
#define SiS_MemoryCopy(Destination,Soruce,Length) memcpy(Destination,Soruce,Length)
#endif

#ifdef LINUX_XF86
#define SiS_SetMemory(MemoryAddress,MemorySize,value) memset(MemoryAddress, value, MemorySize)
#define SiS_MemoryCopy(Destination,Soruce,Length) memcpy(Destination,Soruce,Length)
#endif

/**********************************************************************/

#ifdef OutPortByte
#undef OutPortByte
#endif /* OutPortByte */

#ifdef OutPortWord
#undef OutPortWord
#endif /* OutPortWord */

#ifdef OutPortLong
#undef OutPortLong
#endif /* OutPortLong */

#ifdef InPortByte
#undef InPortByte
#endif /* InPortByte */

#ifdef InPortWord
#undef InPortWord
#endif /* InPortWord */

#ifdef InPortLong
#undef InPortLong
#endif /* InPortLong */

/**********************************************************************/
/*  LINUX KERNEL                                                      */
/**********************************************************************/

#ifdef LINUX_KERNEL
#define OutPortByte(p,v) outb((u8)(v),(u16)(p))
#define OutPortWord(p,v) outw((u16)(v),(u16)(p))
#define OutPortLong(p,v) outl((u32)(v),(u16)(p))
#define InPortByte(p)    inb((u16)(p))
#define InPortWord(p)    inw((u16)(p))
#define InPortLong(p)    inl((u16)(p))
#endif

/**********************************************************************/
/*  LINUX XF86                                                        */
/**********************************************************************/

#ifdef LINUX_XF86
#define OutPortByte(p,v) outb((IOADDRESS)(p),(CARD8)(v))
#define OutPortWord(p,v) outw((IOADDRESS)(p),(CARD16)(v))
#define OutPortLong(p,v) outl((IOADDRESS)(p),(CARD32)(v))
#define InPortByte(p)    inb((IOADDRESS)(p))
#define InPortWord(p)    inw((IOADDRESS)(p))
#define InPortLong(p)    inl((IOADDRESS)(p))
#endif

