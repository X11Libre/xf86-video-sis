/* $XFree86: xc/programs/Xserver/hw/xfree86/drivers/sis/sis_dac.h,v 1.15 2003/10/30 18:53:42 twini Exp $ */
/*
 * DAC helper functions (Save/Restore, MemClk, etc)
 * Definitions and prototypes
 *
 * Copyright 1998,1999 by Alan Hourihane, Wigan, England.
 * Copyright 2001, 2002, 2003 by Thomas Winischhofer, Vienna, Austria.
 *
 * Permission to use, copy, modify, distribute, and sell this software and its
 * documentation for any purpose is hereby granted without fee, provided that
 * the above copyright notice appear in all copies and that both that
 * copyright notice and this permission notice appear in supporting
 * documentation, and that the name of the provider not be used in
 * advertising or publicity pertaining to distribution of the software without
 * specific, written prior permission.  The provider makes no representations
 * about the suitability of this software for any purpose.  It is provided
 * "as is" without express or implied warranty.
 *
 * THE PROVIDER DISCLAIMS ALL WARRANTIES WITH REGARD TO THIS SOFTWARE,
 * INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS, IN NO
 * EVENT SHALL THE PROVIDER BE LIABLE FOR ANY SPECIAL, INDIRECT OR
 * CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE,
 * DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
 * PERFORMANCE OF THIS SOFTWARE.
 *
 * Authors:  Alan Hourihane <alanh@fairlite.demon.co.uk>
 *           Mike Chapman <mike@paranoia.com>,
 *           Juanjo Santamarta <santamarta@ctv.es>,
 *           Mitani Hiroshi <hmitani@drl.mei.co.jp>
 *           David Thomas <davtom@dream.org.uk>.
 *	     Thomas Winischhofer <thomas@winischhofer.net>
 */

int  SiS_compute_vclk(int Clock, int *out_n, int *out_dn, int *out_div,
	     		int *out_sbit, int *out_scale);
void SISDACPreInit(ScrnInfoPtr pScrn);
void SISLoadPalette(ScrnInfoPtr pScrn, int numColors, int *indicies,
		        LOCO *colors, VisualPtr pVisual);
void SiSCalcClock(ScrnInfoPtr pScrn, int clock, int max_VLD,
                        unsigned int *vclk);
void SiSIODump(ScrnInfoPtr pScrn);
int  SiSMemBandWidth(ScrnInfoPtr pScrn, BOOLEAN IsForCRT2);
int  SiSMclk(SISPtr pSiS);
void SiSRestoreBridge(ScrnInfoPtr pScrn, SISRegPtr sisReg);

extern void     SiS6326SetTVReg(ScrnInfoPtr pScrn, CARD8 index, CARD8 data);
extern unsigned char SiS6326GetTVReg(ScrnInfoPtr pScrn, CARD8 index);
extern void     SiS6326SetXXReg(ScrnInfoPtr pScrn, CARD8 index, CARD8 data);
extern unsigned char SiS6326GetXXReg(ScrnInfoPtr pScrn, CARD8 index);

extern int      SiSCalcVRate(DisplayModePtr mode);

/* Functions from init.c and init301.c */
extern void     SiS_UnLockCRT2(SiS_Private *SiS_Pr, PSIS_HW_INFO);
extern void     SiS_LockCRT2(SiS_Private *SiS_Pr, PSIS_HW_INFO);
extern void     SiS_DisableBridge(SiS_Private *SiS_Pr, PSIS_HW_INFO);
extern void     SiS_EnableBridge(SiS_Private *SiS_Pr, PSIS_HW_INFO);
extern USHORT 	SiS_GetCH700x(SiS_Private *SiS_Pr, USHORT tempbx);
extern void 	SiS_SetCH700x(SiS_Private *SiS_Pr, USHORT tempbx);
extern USHORT 	SiS_GetCH701x(SiS_Private *SiS_Pr, USHORT tempbx);
extern void 	SiS_SetCH701x(SiS_Private *SiS_Pr, USHORT tempbx);
extern USHORT 	SiS_GetCH70xx(SiS_Private *SiS_Pr, USHORT tempbx);
extern void 	SiS_SetCH70xx(SiS_Private *SiS_Pr, USHORT tempbx);
extern void     SiS_SetCH70xxANDOR(SiS_Private *SiS_Pr, USHORT tempax,USHORT tempbh);
extern void     SiS_DDC2Delay(SiS_Private *SiS_Pr, USHORT delaytime);
extern USHORT   SiS_ReadDDC1Bit(SiS_Private *SiS_Pr);
extern USHORT   SiS_HandleDDC(SiS_Private *SiS_Pr, unsigned long VBFlags, int VGAEngine,
                              USHORT adaptnum, USHORT DDCdatatype, unsigned char *buffer);
extern void     SiS_SetChrontelGPIO(SiS_Private *SiS_Pr, USHORT myvbinfo);
extern void     SiS_DisplayOn(SiS_Private *SiS_Pr);
extern unsigned char SiS_GetSetModeID(ScrnInfoPtr pScrn, unsigned char id);
extern void     SiS_SetEnableDstn(SiS_Private *SiS_Pr, int enable);
extern void     SiS_SetEnableFstn(SiS_Private *SiS_Pr, int enable);







