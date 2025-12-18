/*
 * 2D Acceleration for SiS 530, 620, 300, 540, 630, 730.
 *
 * Copyright (C) 2001-2005 by Thomas Winischhofer, Vienna, Austria
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1) Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2) Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3) The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors:   Thomas Winischhofer <thomas@winischhofer.net>
 *	      Can-Ru Yeou, SiS Inc.
 *
 */
#include "config.h"

#if 0
#define DEBUG
#endif

#include "sis.h"
#include "sis_regs.h"

#include "sis300_accel.h"

/* This is the offset to the memory for each head */
#define HEADOFFSET 	(pSiS->dhmOffset)

#ifdef SIS_USE_EXA
extern void SiSScratchSave(ScreenPtr pScreen, ExaOffscreenArea *area);
extern Bool SiSUploadToScratch(PixmapPtr pSrc, PixmapPtr pDst);
#endif /* EXA */

extern UChar SiSGetCopyROP(int rop);
extern UChar SiSGetPatternROP(int rop);

static void
SiSInitializeAccelerator(ScrnInfoPtr pScrn)
{
}

static void
SiSSync(ScrnInfoPtr pScrn)
{
	SISPtr pSiS = SISPTR(pScrn);

	SiSIdle
}

static void
SiSSyncAccel(ScrnInfoPtr pScrn)
{
	SISPtr pSiS = SISPTR(pScrn);

	if(!pSiS->NoAccel) SiSSync(pScrn);
}

static void
SiSSetupForScreenToScreenCopy(ScrnInfoPtr pScrn,
			int xdir, int ydir, int rop,
			unsigned int planemask, int trans_color)
{
	SISPtr  pSiS = SISPTR(pScrn);

	SiSSetupDSTColorDepth(pSiS->DstColor);
	SiSSetupSRCPitch(pSiS->scrnOffset)
	SiSSetupDSTRect(pSiS->scrnOffset, -1)

	if(trans_color != -1) {
	   SiSSetupROP(0x0A)
	   SiSSetupSRCTrans(trans_color)
	   SiSSetupCMDFlag(TRANSPARENT_BITBLT)
	} else {
	   SiSSetupROP(SiSGetCopyROP(rop))
	}
	if(xdir > 0) {
	   SiSSetupCMDFlag(X_INC)
	}
	if(ydir > 0) {
	   SiSSetupCMDFlag(Y_INC)
	}
}

static void
SiSSubsequentScreenToScreenCopy(ScrnInfoPtr pScrn,
			int src_x, int src_y, int dst_x, int dst_y,
			int width, int height)
{
	SISPtr pSiS = SISPTR(pScrn);
	CARD32 srcbase, dstbase;

	srcbase = dstbase = 0;
	if(src_y >= 2048) {
	   srcbase = pSiS->scrnOffset * src_y;
	   src_y = 0;
	}
	if((dst_y >= pScrn->virtualY) || (dst_y >= 2048)) {
	   dstbase = pSiS->scrnOffset * dst_y;
	   dst_y = 0;
	}
#ifdef SISDUALHEAD
	if(pSiS->VGAEngine != SIS_530_VGA) {
	   srcbase += HEADOFFSET;
	   dstbase += HEADOFFSET;
	}
#endif
	SiSSetupSRCBase(srcbase);
	SiSSetupDSTBase(dstbase);

	if(!(pSiS->CommandReg & X_INC)) {
	   src_x += width-1;
	   dst_x += width-1;
	}
	if(!(pSiS->CommandReg & Y_INC)) {
	   src_y += height-1;
	   dst_y += height-1;
	}
	SiSSetupRect(width, height)
	SiSSetupSRCXY(src_x, src_y)
	SiSSetupDSTXY(dst_x, dst_y)

	SiSDoCMD
}

static void
SiSSetupForSolidFill(ScrnInfoPtr pScrn,
			int color, int rop, unsigned int planemask)
{
	SISPtr pSiS = SISPTR(pScrn);

	if(pSiS->disablecolorkeycurrent) {
	   if((CARD32)color == pSiS->colorKey) {
	      rop = 5;  /* NOOP */
	   }
	}
	SiSSetupPATFG(color)
	SiSSetupDSTRect(pSiS->scrnOffset, -1)
	SiSSetupDSTColorDepth(pSiS->DstColor);
	SiSSetupROP(SiSGetPatternROP(rop))
	/* SiSSetupCMDFlag(PATFG) - is zero */
}

static void
SiSSubsequentSolidFillRect(ScrnInfoPtr pScrn,
			int x, int y, int w, int h)
{
	SISPtr pSiS = SISPTR(pScrn);
	CARD32 dstbase = 0;

	if(y >= 2048) {
	   dstbase = pSiS->scrnOffset * y;
	   y = 0;
	}
#ifdef SISDUALHEAD
        if(pSiS->VGAEngine != SIS_530_VGA) {
	   dstbase += HEADOFFSET;
        }
#endif
	SiSSetupDSTBase(dstbase)
	SiSSetupDSTXY(x,y)
	SiSSetupRect(w,h)
	/* Clear commandReg because Setup can be used for Rect and Trap */
	pSiS->CommandReg &= ~(T_XISMAJORL | T_XISMAJORR |
			      T_L_X_INC | T_L_Y_INC |
			      T_R_X_INC | T_R_Y_INC |
			      TRAPAZOID_FILL);
	SiSSetupCMDFlag(X_INC | Y_INC | BITBLT)

	SiSDoCMD
}

#ifdef SIS_USE_EXA  /* ---------------------------- EXA -------------------------- */

static const unsigned short dstcol[] = { 0x0000, 0x8000, 0xc000 };

static void
SiSEXASync(ScreenPtr pScreen, int marker)
{
	SISPtr pSiS = SISPTR(xf86ScreenToScrn(pScreen));

	SiSIdle
}

static Bool
SiSPrepareSolid(PixmapPtr pPixmap, int alu, Pixel planemask, Pixel fg)
{
	ScrnInfoPtr pScrn = xf86ScreenToScrn(pPixmap->drawable.pScreen);
	SISPtr pSiS = SISPTR(pScrn);
	CARD32 dstbase;

	/* Planemask not supported */
	if((planemask & ((1 << pPixmap->drawable.depth) - 1)) !=
				(1 << pPixmap->drawable.depth) - 1) {
	   return FALSE;
	}

	/* Since the 530/620 have no "dest color depth" register, I
	 * assume that the 2D engine reads the current color depth
	 * from the DAC.... FIXME ? */
	if(pSiS->VGAEngine == SIS_530_VGA) {
	   if(pPixmap->drawable.bitsPerPixel != pSiS->CurrentLayout.bitsPerPixel)
	      return FALSE;
	} else if((pPixmap->drawable.bitsPerPixel != 8)  &&
		  (pPixmap->drawable.bitsPerPixel != 16) &&
		  (pPixmap->drawable.bitsPerPixel != 32))
	   return FALSE;

	if(pSiS->disablecolorkeycurrent) {
	   if((CARD32)fg == pSiS->colorKey) {
	      alu = 5;  /* NOOP */
	   }
	}

	/* Check that the pitch matches the hardware's requirements. Should
	 * never be a problem due to pixmapPitchAlign and fbScreenInit.
	 */
	if(exaGetPixmapPitch(pPixmap) & 3)
	   return FALSE;

	dstbase = (CARD32)exaGetPixmapOffset(pPixmap) + HEADOFFSET;

	SiSSetupPATFG(fg)
	SiSSetupDSTRect(exaGetPixmapPitch(pPixmap), -1)
	SiSSetupDSTColorDepth(dstcol[pPixmap->drawable.bitsPerPixel >> 4]);
	SiSSetupROP(SiSGetPatternROP(alu))
	SiSSetupDSTBase(dstbase)
	/* SiSSetupCMDFlag(PATFG) - is zero */

	return TRUE;
}

static void
SiSSolid(PixmapPtr pPixmap, int x1, int y1, int x2, int y2)
{
	ScrnInfoPtr pScrn = xf86ScreenToScrn(pPixmap->drawable.pScreen);
	SISPtr pSiS = SISPTR(pScrn);

	SiSSetupDSTXY(x1, y1)
	SiSSetupRect(x2-x1, y2-y1)

	SiSSetupCMDFlag(X_INC | Y_INC | BITBLT)

	SiSDoCMD
}

static void
SiSDoneSolid(PixmapPtr pPixmap)
{
}

static Bool
SiSPrepareCopy(PixmapPtr pSrcPixmap, PixmapPtr pDstPixmap, int xdir, int ydir,
					int alu, Pixel planemask)
{
	ScrnInfoPtr pScrn = xf86ScreenToScrn(pDstPixmap->drawable.pScreen);
	SISPtr pSiS = SISPTR(pScrn);
	CARD32 srcbase, dstbase;

	/* Planemask not supported */
	if((planemask & ((1 << pSrcPixmap->drawable.depth) - 1)) !=
				(1 << pSrcPixmap->drawable.depth) - 1) {
	   return FALSE;
	}

	/* Since the 530/620 have no "dest color depth" register, I
	 * assume that the 2D engine reads the current color depth
	 * from the DAC.... FIXME ? */
	if(pSiS->VGAEngine == SIS_530_VGA) {
	   if(pDstPixmap->drawable.bitsPerPixel != pSiS->CurrentLayout.bitsPerPixel)
	      return FALSE;
	} else if((pDstPixmap->drawable.bitsPerPixel != 8)  &&
		  (pDstPixmap->drawable.bitsPerPixel != 16) &&
		  (pDstPixmap->drawable.bitsPerPixel != 32))
	   return FALSE;

	/* Check that the pitch matches the hardware's requirements. Should
	 * never be a problem due to pixmapPitchAlign and fbScreenInit.
	 */
	if(exaGetPixmapPitch(pSrcPixmap) & 3)
	   return FALSE;
	if(exaGetPixmapPitch(pDstPixmap) & 3)
	   return FALSE;

	SiSSetupDSTColorDepth(dstcol[pDstPixmap->drawable.bitsPerPixel >> 4]);
	SiSSetupSRCPitch(exaGetPixmapPitch(pSrcPixmap))
	SiSSetupDSTRect(exaGetPixmapPitch(pDstPixmap), -1)

	SiSSetupROP(SiSGetCopyROP(alu))

	if(xdir >= 0) {
	   SiSSetupCMDFlag(X_INC)
	}
	if(ydir >= 0) {
	   SiSSetupCMDFlag(Y_INC)
	}

	srcbase = (CARD32)exaGetPixmapOffset(pSrcPixmap) + HEADOFFSET;

	dstbase = (CARD32)exaGetPixmapOffset(pDstPixmap) + HEADOFFSET;

	SiSSetupSRCBase(srcbase);
	SiSSetupDSTBase(dstbase);

	return TRUE;
}

static void
SiSCopy(PixmapPtr pDstPixmap, int srcX, int srcY, int dstX, int dstY, int width, int height)
{
	ScrnInfoPtr pScrn = xf86ScreenToScrn(pDstPixmap->drawable.pScreen);
	SISPtr pSiS = SISPTR(pScrn);

	if(!(pSiS->CommandReg & X_INC))  {
	   srcX += width-1;
	   dstX += width-1;
	}
	if(!(pSiS->CommandReg & Y_INC))  {
	   srcY += height-1;
	   dstY += height-1;
	}
	SiSSetupRect(width, height)
	SiSSetupSRCXY(srcX, srcY)
	SiSSetupDSTXY(dstX, dstY)

	SiSDoCMD
}

static void
SiSDoneCopy(PixmapPtr pDstPixmap)
{
}

#endif /* EXA */

/* For DGA usage */

static void
SiSDGAFillRect(ScrnInfoPtr pScrn, int x, int y, int w, int h, int color)
{
	SiSSetupForSolidFill(pScrn, color, GXcopy, ~0);
	SiSSubsequentSolidFillRect(pScrn, x, y, w, h);
}

static void
SiSDGABlitRect(ScrnInfoPtr pScrn, int srcx, int srcy, int dstx, int dsty, int w, int h, int color)
{
	int xdir = ((srcx < dstx) && (srcy == dsty)) ? -1 : 1;
	int ydir = (srcy < dsty) ? -1 : 1;

	SiSSetupForScreenToScreenCopy(pScrn, xdir, ydir, GXcopy, (CARD32)~0, color);
	SiSSubsequentScreenToScreenCopy(pScrn, srcx, srcy, dstx, dsty, w, h);
}

/* Initialisation */

Bool
SiS300AccelInit(ScreenPtr pScreen)
{
	ScrnInfoPtr     pScrn = xf86ScreenToScrn(pScreen);
	SISPtr          pSiS = SISPTR(pScrn);

	pSiS->ColorExpandBufferNumber = 0;
	pSiS->PerColorExpandBufferSize = 0;
	pSiS->RenderAccelArray = NULL;
#ifdef SIS_USE_EXA
	pSiS->EXADriverPtr = NULL;
	pSiS->exa_scratch = NULL;
#endif

	if((pScrn->bitsPerPixel != 8)  &&
	   (pScrn->bitsPerPixel != 16) &&
	   (pScrn->bitsPerPixel != 32)) {
	   pSiS->NoAccel = TRUE;
	}

	if(!pSiS->NoAccel) {
#ifdef SIS_USE_EXA
	   if(pSiS->useEXA) {
	      if(!(pSiS->EXADriverPtr = exaDriverAlloc())) {
		 pSiS->NoAccel = TRUE;
		 pSiS->NoXvideo = TRUE; /* No fbmem manager -> no xv */
	      }
	   }
#endif
	}

	if(!pSiS->NoAccel) {

	   SiSInitializeAccelerator(pScrn);

	   pSiS->InitAccel = SiSInitializeAccelerator;
	   pSiS->SyncAccel = SiSSyncAccel;
	   pSiS->FillRect  = SiSDGAFillRect;
	   pSiS->BlitRect  = SiSDGABlitRect;

#ifdef SIS_USE_EXA	/* ----------------------- EXA ----------------------- */
	   if(pSiS->useEXA) {

	      pSiS->EXADriverPtr->exa_major = 2;
	      pSiS->EXADriverPtr->exa_minor = 0;

	      if(pSiS->scrnOffset < 8192) {

		 /* data */
		 pSiS->EXADriverPtr->memoryBase = pSiS->FbBase;
		 pSiS->EXADriverPtr->memorySize = pSiS->maxxfbmem;
		 pSiS->EXADriverPtr->offScreenBase = pScrn->virtualX * pScrn->virtualY
						* ((pScrn->bitsPerPixel + 7) / 8);
		 if(pSiS->EXADriverPtr->memorySize > pSiS->EXADriverPtr->offScreenBase) {
		    pSiS->EXADriverPtr->flags = EXA_OFFSCREEN_PIXMAPS;
		 } else {
		    pSiS->NoXvideo = TRUE;
		    xf86DrvMsg(pScrn->scrnIndex, X_ERROR,
			"Not enough video RAM for offscreen memory manager. Xv disabled\n");
		 }
		 pSiS->EXADriverPtr->pixmapOffsetAlign = 16;	/* src/dst: double quad word boundary */
		 pSiS->EXADriverPtr->pixmapPitchAlign = 4;		/* pitch:   double word boundary      */
		 if(pSiS->VGAEngine == SIS_300_VGA) {
		    pSiS->EXADriverPtr->maxX = 4095;
		    pSiS->EXADriverPtr->maxY = 4095;
		 } else {
		    pSiS->EXADriverPtr->maxX = 2047;
		    pSiS->EXADriverPtr->maxY = 2047;
		 }

		 /* Sync */
		 pSiS->EXADriverPtr->WaitMarker = SiSEXASync;

		 /* Solid fill */
		 pSiS->EXADriverPtr->PrepareSolid = SiSPrepareSolid;
		 pSiS->EXADriverPtr->Solid = SiSSolid;
		 pSiS->EXADriverPtr->DoneSolid = SiSDoneSolid;

		 /* Copy */
		 pSiS->EXADriverPtr->PrepareCopy = SiSPrepareCopy;
		 pSiS->EXADriverPtr->Copy = SiSCopy;
		 pSiS->EXADriverPtr->DoneCopy = SiSDoneCopy;

		 /* Composite not supported */

	      } else {

		 xf86DrvMsg(pScrn->scrnIndex, X_WARNING,
			"Virtual screen width too large for accelerator engine\n");
		 xf86DrvMsg(pScrn->scrnIndex, X_WARNING,
			"\t2D acceleration and Xv disabled\n");
		 pSiS->NoAccel = TRUE;
		 pSiS->NoXvideo = TRUE; /* No fbmem manager -> no xv */

	      }

	   }
#endif /* EXA */


	}  /* NoAccel */

	/* Init framebuffer memory manager */

	/* Layout: (Sizes do not reflect correct proportions)
	 * |--------------++++++++++++++++++++^************==========~~~~~~~~~~~~|
	 *   UsableFbSize  ColorExpandBuffers | DRI-Heap  |  HWCursor  TurboQueue   300/310/325 series
	 * |--------------++++++++++++++++++++|  ====================~~~~~~~~~~~~|
	 *   UsableFbSize  ColorExpandBuffers |        TurboQueue     HWCursor      530/620
	 *                                  topFB
	 */

#ifdef SIS_USE_EXA
        if(pSiS->useEXA) {

	   if(!pSiS->NoAccel) {

	      if(!exaDriverInit(pScreen, pSiS->EXADriverPtr)) {
		 pSiS->NoAccel = TRUE;
		 pSiS->NoXvideo = TRUE; /* No fbmem manager -> no xv */
		 return FALSE;
	      }

	      /* Reserve locked offscreen scratch area of 128K for glyph data */
	      pSiS->exa_scratch = exaOffscreenAlloc(pScreen, 128 * 1024, 16, TRUE,
						SiSScratchSave, pSiS);
	      if(pSiS->exa_scratch) {
		 pSiS->exa_scratch_next = pSiS->exa_scratch->offset;
		 pSiS->EXADriverPtr->UploadToScratch = SiSUploadToScratch;
	      }

	   } else {

	      pSiS->NoXvideo = TRUE; /* No fbmem manager -> no xv */

	   }

	}
#endif /* EXA */

	return TRUE;
}
