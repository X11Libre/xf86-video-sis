/*
 * 2D acceleration for SiS5597/5598 and 6326
 *
 * Copyright (C) 1998, 1999 by Alan Hourihane, Wigan, England.
 * Parts Copyright (C) 2001-2005 Thomas Winischhofer, Vienna, Austria.
 *
 * Licensed under the following terms:
 *
 * Permission to use, copy, modify, distribute, and sell this software and its
 * documentation for any purpose is hereby granted without fee, provided that
 * the above copyright notice appears in all copies and that both that copyright
 * notice and this permission notice appear in supporting documentation, and
 * and that the name of the copyright holder not be used in advertising
 * or publicity pertaining to distribution of the software without specific,
 * written prior permission. The copyright holder makes no representations
 * about the suitability of this software for any purpose.  It is provided
 * "as is" without expressed or implied warranty.
 *
 * THE COPYRIGHT HOLDER DISCLAIMS ALL WARRANTIES WITH REGARD TO THIS SOFTWARE,
 * INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO
 * EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY SPECIAL, INDIRECT OR
 * CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE,
 * DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
 * PERFORMANCE OF THIS SOFTWARE.
 *
 * Authors:  Alan Hourihane <alanh@fairlite.demon.co.uk>,
 *           Mike Chapman <mike@paranoia.com>,
 *           Juanjo Santamarta <santamarta@ctv.es>,
 *           Mitani Hiroshi <hmitani@drl.mei.co.jp>,
 *           David Thomas <davtom@dream.org.uk>,
 *	     Thomas Winischhofer <thomas@winischhofer.net>.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "sis.h"
#include "sis_regs.h"
#include "sis_accel.h"

#ifdef SIS_USE_EXA
extern void SiSScratchSave(ScreenPtr pScreen, ExaOffscreenArea *area);
extern Bool SiSUploadToScratch(PixmapPtr pSrc, PixmapPtr pDst);
#endif /* EXA */

extern UChar SiSGetCopyROP(int rop);
extern UChar SiSGetPatternROP(int rop);

static void
SiSInitializeAccelerator(ScrnInfoPtr pScrn)
{
    /* Nothing here yet */
}

/* sync */
static void
SiSSync(ScrnInfoPtr pScrn)
{
    SISPtr pSiS = SISPTR(pScrn);

    sisBLTSync;
}

static void
SiSSyncAccel(ScrnInfoPtr pScrn)
{
    SISPtr pSiS = SISPTR(pScrn);

    if(!pSiS->NoAccel) SiSSync(pScrn);
}

/* Screen to screen copy */
static void
SiSSetupForScreenToScreenCopy(ScrnInfoPtr pScrn, int xdir, int ydir,
			int rop, unsigned int planemask,
			int transparency_color)
{
    SISPtr pSiS = SISPTR(pScrn);

    sisBLTSync;
    sisSETPITCH(pSiS->scrnOffset, pSiS->scrnOffset);

    sisSETROP(SiSGetCopyROP(rop));
    pSiS->Xdirection = xdir;
    pSiS->Ydirection = ydir;
}

static void
SiSSubsequentScreenToScreenCopy(ScrnInfoPtr pScrn, int x1, int y1, int x2,
			int y2, int w, int h)
{
    SISPtr pSiS = SISPTR(pScrn);
    int srcaddr, destaddr, op;

    op = sisCMDBLT | sisSRCVIDEO;

    if(pSiS->Ydirection == -1) {
       op |= sisBOTTOM2TOP;
       srcaddr = (y1 + h - 1) * pSiS->CurrentLayout.displayWidth;
       destaddr = (y2 + h - 1) * pSiS->CurrentLayout.displayWidth;
    } else {
       op |= sisTOP2BOTTOM;
       srcaddr = y1 * pSiS->CurrentLayout.displayWidth;
       destaddr = y2 * pSiS->CurrentLayout.displayWidth;
    }

    if(pSiS->Xdirection == -1) {
       op |= sisRIGHT2LEFT;
       srcaddr += x1 + w - 1;
       destaddr += x2 + w - 1;
    } else {
       op |= sisLEFT2RIGHT;
       srcaddr += x1;
       destaddr += x2;
    }

    if(pSiS->ClipEnabled)
       op |= sisCLIPINTRN | sisCLIPENABL;

    srcaddr *= (pSiS->CurrentLayout.bitsPerPixel/8);
    destaddr *= (pSiS->CurrentLayout.bitsPerPixel/8);
    if(((pSiS->CurrentLayout.bitsPerPixel / 8) > 1) && (pSiS->Xdirection == -1)) {
       srcaddr += (pSiS->CurrentLayout.bitsPerPixel/8)-1;
       destaddr += (pSiS->CurrentLayout.bitsPerPixel/8)-1;
    }

    sisBLTSync;
    sisSETSRCADDR(srcaddr);
    sisSETDSTADDR(destaddr);
    sisSETHEIGHTWIDTH(h-1, w * (pSiS->CurrentLayout.bitsPerPixel/8)-1);
    sisSETCMD(op);
}

/* solid fill */
static void
SiSSetupForFillRectSolid(ScrnInfoPtr pScrn, int color, int rop,
			unsigned int planemask)
{
    SISPtr pSiS = SISPTR(pScrn);

    sisBLTSync;
    sisSETBGROPCOL(SiSGetCopyROP(rop), color);
    sisSETFGROPCOL(SiSGetCopyROP(rop), color);
    sisSETPITCH(pSiS->scrnOffset, pSiS->scrnOffset);
}

static void
SiSSubsequentFillRectSolid(ScrnInfoPtr pScrn, int x, int y, int w, int h)
{
    SISPtr pSiS = SISPTR(pScrn);
    int destaddr, op;

    destaddr = y * pSiS->CurrentLayout.displayWidth + x;

    op = sisCMDBLT | sisSRCBG | sisTOP2BOTTOM | sisLEFT2RIGHT;

    if(pSiS->ClipEnabled)
       op |= sisCLIPINTRN | sisCLIPENABL;

    destaddr *= (pSiS->CurrentLayout.bitsPerPixel / 8);

    sisBLTSync;
    sisSETHEIGHTWIDTH(h-1, w * (pSiS->CurrentLayout.bitsPerPixel/8)-1);
    sisSETDSTADDR(destaddr);
    sisSETCMD(op);
}

#ifdef SIS_USE_EXA  /* ---------------------------- EXA -------------------------- */

static void
SiSEXASync(ScreenPtr pScreen, int marker)
{
	SISPtr pSiS = SISPTR(xf86ScreenToScrn(pScreen));

	sisBLTSync;
}

static Bool
SiSPrepareSolid(PixmapPtr pPixmap, int alu, Pixel planemask, Pixel fg)
{
	ScrnInfoPtr pScrn = xf86ScreenToScrn(pPixmap->drawable.pScreen);
	SISPtr pSiS = SISPTR(pScrn);

	/* Planemask not supported */
	if((planemask & ((1 << pPixmap->drawable.depth) - 1)) !=
				(1 << pPixmap->drawable.depth) - 1) {
	   return FALSE;
	}

	/* Since these old engines have no "dest color depth" register, I assume
	 * the 2D engine takes the bpp from the DAC... FIXME ? */
	if(pPixmap->drawable.bitsPerPixel != pSiS->CurrentLayout.bitsPerPixel)
	   return FALSE;

	/* Check that the pitch matches the hardware's requirements. Should
	 * never be a problem due to pixmapPitchAlign and fbScreenInit.
	 */
	if(exaGetPixmapPitch(pPixmap) & 7)
	   return FALSE;

	pSiS->fillPitch = exaGetPixmapPitch(pPixmap);
	pSiS->fillBpp = pPixmap->drawable.bitsPerPixel >> 3;
	pSiS->fillDstBase = (CARD32)exaGetPixmapOffset(pPixmap);

	sisBLTSync;
	sisSETBGROPCOL(SiSGetCopyROP(alu), fg);
	sisSETFGROPCOL(SiSGetCopyROP(alu), fg);
	sisSETPITCH(pSiS->fillPitch, pSiS->fillPitch);

	return TRUE;
}

static void
SiSSolid(PixmapPtr pPixmap, int x1, int y1, int x2, int y2)
{
	ScrnInfoPtr pScrn = xf86ScreenToScrn(pPixmap->drawable.pScreen);
	SISPtr pSiS = SISPTR(pScrn);

	sisBLTSync;
	sisSETHEIGHTWIDTH((y2 - y1) - 1, (x2 - x1) * pSiS->fillBpp - 1);
	sisSETDSTADDR((pSiS->fillDstBase + ((y1 * (pSiS->fillPitch / pSiS->fillBpp) + x1) * pSiS->fillBpp)));
	sisSETCMD((sisCMDBLT | sisSRCBG | sisTOP2BOTTOM | sisLEFT2RIGHT));
}

static void
SiSDoneSolid(PixmapPtr pPixmap)
{
}

static Bool
SiSPrepareCopy(PixmapPtr pSrcPixmap, PixmapPtr pDstPixmap, int xdir, int ydir,
					int alu, Pixel planemask)
{
	ScrnInfoPtr pScrn = xf86ScreenToScrn(pSrcPixmap->drawable.pScreen);
	SISPtr pSiS = SISPTR(pScrn);

	/* Planemask not supported */
	if((planemask & ((1 << pSrcPixmap->drawable.depth) - 1)) !=
				(1 << pSrcPixmap->drawable.depth) - 1) {
	   return FALSE;
	}

	/* Since these old engines have no "dest color depth" register, I assume
	 * the 2D engine takes the bpp from the DAC... FIXME ? */
	if(pDstPixmap->drawable.bitsPerPixel != pSiS->CurrentLayout.bitsPerPixel)
	   return FALSE;

	/* Check that the pitch matches the hardware's requirements. Should
	 * never be a problem due to pixmapPitchAlign and fbScreenInit.
	 */
	if(exaGetPixmapPitch(pSrcPixmap) & 3)
	   return FALSE;
	if(exaGetPixmapPitch(pDstPixmap) & 7)
	   return FALSE;

	pSiS->copyXdir = xdir;
	pSiS->copyYdir = ydir;
	pSiS->copyBpp = pSrcPixmap->drawable.bitsPerPixel >> 3;
	pSiS->copySPitch = exaGetPixmapPitch(pSrcPixmap);
	pSiS->copyDPitch = exaGetPixmapPitch(pDstPixmap);
	pSiS->copySrcBase = (CARD32)exaGetPixmapOffset(pSrcPixmap);
	pSiS->copyDstBase = (CARD32)exaGetPixmapOffset(pDstPixmap);

	sisBLTSync;
	sisSETPITCH(pSiS->copySPitch, pSiS->copyDPitch);
	sisSETROP(SiSGetCopyROP(alu));

	return TRUE;
}

static void
SiSCopy(PixmapPtr pDstPixmap, int srcX, int srcY, int dstX, int dstY, int width, int height)
{
	ScrnInfoPtr pScrn = xf86ScreenToScrn(pDstPixmap->drawable.pScreen);
	SISPtr pSiS = SISPTR(pScrn);
	CARD32 srcbase, dstbase, cmd;
	int bpp = pSiS->copyBpp;
	int srcPixelPitch = pSiS->copySPitch / bpp;
	int dstPixelPitch = pSiS->copyDPitch / bpp;

	cmd = sisCMDBLT | sisSRCVIDEO;

	if(pSiS->copyYdir < 0) {
	   cmd |= sisBOTTOM2TOP;
	   srcbase = (srcY + height - 1) * srcPixelPitch;
	   dstbase = (dstY + height - 1) * dstPixelPitch;
	} else {
	   cmd |= sisTOP2BOTTOM;
	   srcbase = srcY * srcPixelPitch;
	   dstbase = dstY * dstPixelPitch;
	}

	if(pSiS->copyXdir < 0) {
	   cmd |= sisRIGHT2LEFT;
	   srcbase += srcX + width - 1;
	   dstbase += dstX + width - 1;
	} else {
	   cmd |= sisLEFT2RIGHT;
	   srcbase += srcX;
	   dstbase += dstX;
	}

	srcbase *= bpp;
	dstbase *= bpp;
	if(pSiS->copyXdir < 0) {
	   srcbase += bpp - 1;
	   dstbase += bpp - 1;
	}

	srcbase += pSiS->copySrcBase;
	dstbase += pSiS->copyDstBase;

	sisBLTSync;
	sisSETSRCADDR(srcbase);
	sisSETDSTADDR(dstbase);
	sisSETHEIGHTWIDTH(height - 1, width * bpp - 1);
	sisSETCMD(cmd);
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
	SiSSetupForFillRectSolid(pScrn, color, GXcopy, ~0);
	SiSSubsequentFillRectSolid(pScrn, x, y, w, h);
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
SiSAccelInit(ScreenPtr pScreen)
{
    ScrnInfoPtr    pScrn = xf86ScreenToScrn(pScreen);
    SISPtr         pSiS = SISPTR(pScrn);

    pSiS->ColorExpandBufferNumber = 0;
    pSiS->PerColorExpandBufferSize = 0;
    pSiS->RenderAccelArray = NULL;
#ifdef SIS_USE_EXA
    pSiS->EXADriverPtr = NULL;
    pSiS->exa_scratch = NULL;
#endif

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

	  /* data */
	  pSiS->EXADriverPtr->memoryBase = pSiS->FbBase;
	  pSiS->EXADriverPtr->memorySize = pSiS->maxxfbmem;
	  pSiS->EXADriverPtr->offScreenBase = pScrn->virtualX * pScrn->virtualY
						* (pScrn->bitsPerPixel >> 3);
	  if(pSiS->EXADriverPtr->memorySize > pSiS->EXADriverPtr->offScreenBase) {
	     pSiS->EXADriverPtr->flags = EXA_OFFSCREEN_PIXMAPS;
	  } else {
	     pSiS->NoXvideo = TRUE;
	     xf86DrvMsg(pScrn->scrnIndex, X_ERROR,
		"Not enough video RAM for offscreen memory manager. Xv disabled\n");
	  }
	  pSiS->EXADriverPtr->pixmapOffsetAlign = 8;	/* src/dst: double quad word boundary */
	  pSiS->EXADriverPtr->pixmapPitchAlign = 8;	/* could possibly be 1, but who knows for sure */
	  pSiS->EXADriverPtr->maxX = 2047;
	  pSiS->EXADriverPtr->maxY = 2047;

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

       }
#endif /* EXA */

    }  /* NoAccel */

    /* Offscreen memory manager
     *
     * Layout: (Sizes here do not reflect correct proportions)
     * |--------------++++++++++++++++++++|  ====================~~~~~~~~~~~~|
     *   UsableFbSize  ColorExpandBuffers |        TurboQueue     HWCursor
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

          /* Reserve locked offscreen scratch area of 64K for glyph data */
	  pSiS->exa_scratch = exaOffscreenAlloc(pScreen, 64 * 1024, 16, TRUE,
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
