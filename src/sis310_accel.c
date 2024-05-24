/*
 * 2D Acceleration for SiS 315, 330 and 340 series
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
 *
 * Author:  	Thomas Winischhofer <thomas@winischhofer.net>
 *
 * 2003/08/18: Rewritten for using VRAM command queue
 *
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "sis.h"
#define SIS_NEED_MYMMIO
#define SIS_NEED_ACCELBUF
#include "sis_regs.h"
#include "sis310_accel.h"

#if 0
#define ACCELDEBUG
#endif

#define FBOFFSET 	(pSiS->dhmOffset)

#define DEV_HEIGHT	0xfff	/* "Device height of destination bitmap" */

#undef SIS_NEED_ARRAY

/* For EXA */

#ifdef SIS_USE_EXA
#if 0
#define SIS_HAVE_COMPOSITE		/* Have our own EXA composite */
#endif
#ifdef SIS_HAVE_COMPOSITE
#ifndef SIS_NEED_ARRAY
#define SIS_NEED_ARRAY
#endif
#endif
#endif

#ifdef SIS_USE_EXA		/* EXA */
void SiSScratchSave(ScreenPtr pScreen, ExaOffscreenArea *area);
Bool SiSUploadToScratch(PixmapPtr pSrc, PixmapPtr pDst);
#endif /* EXA */

void SISWriteBlitPacket(SISPtr pSiS, CARD32 *packet);

extern unsigned char SiSGetCopyROP(int rop);
extern unsigned char SiSGetPatternROP(int rop);

CARD32 dummybuf;

#ifdef SIS_NEED_ARRAY
#define SiSRenderOpsMAX 0x2b
static const CARD8 SiSRenderOps[] = {	/* PictOpXXX 1 = supported, 0 = unsupported */
     1, 1, 1, 1,
     0, 0, 0, 0,
     0, 0, 0, 0,
     0, 0, 0, 0,
     1, 1, 1, 0,
     0, 0, 0, 0,
     0, 0, 0, 0,
     0, 0, 0, 0,
     1, 1, 1, 0,
     0, 0, 0, 0,
     0, 0, 0, 0,
     0, 0, 0, 0
};
#endif /* NEED ARRAY */

#ifdef SIS_NEED_ARRAY
static void
SiSCalcRenderAccelArray(ScrnInfoPtr pScrn)
{
	SISPtr  pSiS = SISPTR(pScrn);
#ifdef SISDUALHEAD
	SISEntPtr pSiSEnt = pSiS->entityPrivate;;
#endif

	if(((pScrn->bitsPerPixel == 16) || (pScrn->bitsPerPixel == 32)) && pSiS->doRender) {
	   int i, j;
#ifdef SISDUALHEAD
	   if(pSiSEnt) pSiS->RenderAccelArray = pSiSEnt->RenderAccelArray;
#endif
	   if(!pSiS->RenderAccelArray) {
	      if((pSiS->RenderAccelArray = XNFcallocarray(65536, 1))) {
#ifdef SISDUALHEAD
	         if(pSiSEnt) pSiSEnt->RenderAccelArray = pSiS->RenderAccelArray;
#endif
		 for(i = 0; i < 256; i++) {
		    for(j = 0; j < 256; j++) {
		       pSiS->RenderAccelArray[(i << 8) + j] = (i * j) / 255;
		    }
		 }
	      }
	   }
	}
}
#endif

#ifdef SIS_USE_EXA
void
SiSScratchSave(ScreenPtr pScreen, ExaOffscreenArea *area)
{
	SISPtr pSiS = SISPTR(xf86ScreenToScrn(pScreen));

	pSiS->exa_scratch = NULL;
}
#endif

static void
SiSSync(ScrnInfoPtr pScrn)
{
	SISPtr pSiS = SISPTR(pScrn);

	pSiS->alphaBlitBusy = FALSE;

	SiSIdle
}

static void
SiSSyncAccel(ScrnInfoPtr pScrn)
{
	SISPtr pSiS = SISPTR(pScrn);

	if(!pSiS->NoAccel) SiSSync(pScrn);
}

static void
SiSInitializeAccelerator(ScrnInfoPtr pScrn)
{
	SISPtr  pSiS = SISPTR(pScrn);

	pSiS->alphaBlitBusy = FALSE;

	if(!pSiS->NoAccel) {
	   if(pSiS->ChipType == XGI_40) {
	      SiSSync(pScrn);
	      SiSDualPipe(1);	/* 1 = disable, 0 = enable */
	      SiSSync(pScrn);
	   }
	}
}

static void
SiSSetupForScreenToScreenCopy(ScrnInfoPtr pScrn,
			int xdir, int ydir, int rop,
			unsigned int planemask, int trans_color)
{
	SISPtr  pSiS = SISPTR(pScrn);

	SiSSetupDSTColorDepth(pSiS->SiS310_AccelDepth);
	SiSCheckQueue(16 * 2);
	SiSSetupSRCPitchDSTRect(pSiS->scrnOffset, pSiS->scrnOffset, DEV_HEIGHT)

	if(trans_color != -1) {
	   SiSSetupROP(0x0A)
	   SiSSetupSRCTrans(trans_color)
	   SiSSetupCMDFlag(TRANSPARENT_BITBLT)
	} else {
	   SiSSetupROP(SiSGetCopyROP(rop))
	   /* Set command - not needed, both 0 */
	   /* SiSSetupCMDFlag(BITBLT | SRCVIDEO) */
	}

	SiSSyncWP

	/* The chip is smart enough to know the direction */
}

static void
SiSSubsequentScreenToScreenCopy(ScrnInfoPtr pScrn,
			int src_x, int src_y, int dst_x, int dst_y,
			int width, int height)
{
	SISPtr pSiS = SISPTR(pScrn);
	CARD32 srcbase, dstbase;
	int    mymin, mymax;

	srcbase = dstbase = 0;
	mymin = min(src_y, dst_y);
	mymax = max(src_y, dst_y);

	/* Although the chip knows the direction to use
	 * if the source and destination areas overlap,
	 * that logic fails if we fiddle with the bitmap
	 * addresses. Therefore, we check if the source
	 * and destination blitting areas overlap and
	 * adapt the bitmap addresses synchronously
	 * if the coordinates exceed the valid range.
	 * The the areas do not overlap, we do our
	 * normal check.
	 */
	if((mymax - mymin) < height) {
	   if((src_y >= 2048) || (dst_y >= 2048)) {
	      srcbase = pSiS->scrnOffset * mymin;
	      dstbase = pSiS->scrnOffset * mymin;
	      src_y -= mymin;
	      dst_y -= mymin;
	   }
	} else {
	   if(src_y >= 2048) {
	      srcbase = pSiS->scrnOffset * src_y;
	      src_y = 0;
	   }
	   if((dst_y >= pScrn->virtualY) || (dst_y >= 2048)) {
	      dstbase = pSiS->scrnOffset * dst_y;
	      dst_y = 0;
	   }
	}

	srcbase += FBOFFSET;
	dstbase += FBOFFSET;

	SiSCheckQueue(16 * 3);
	SiSSetupSRCDSTBase(srcbase, dstbase)
	SiSSetupSRCDSTXY(src_x, src_y, dst_x, dst_y)
	SiSSetRectDoCMD(width,height)
}

static void
SiSSetupForSolidFill(ScrnInfoPtr pScrn, int color,
			int rop, unsigned int planemask)
{
	SISPtr  pSiS = SISPTR(pScrn);

	if(pSiS->disablecolorkeycurrent) {
	   if((CARD32)color == pSiS->colorKey) {
	      rop = 5;  /* NOOP */
	   }
	}

	SiSSetupDSTColorDepth(pSiS->SiS310_AccelDepth);
	SiSCheckQueue(16 * 1);
	SiSSetupPATFGDSTRect(color, pSiS->scrnOffset, DEV_HEIGHT)
	SiSSetupROP(SiSGetPatternROP(rop))
	SiSSetupCMDFlag(PATFG)
	SiSSyncWP
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

	dstbase += FBOFFSET;

	pSiS->CommandReg &= ~(T_XISMAJORL | T_XISMAJORR |
	                      T_L_X_INC | T_L_Y_INC |
	                      T_R_X_INC | T_R_Y_INC |
			      TRAPAZOID_FILL);

	/* SiSSetupCMDFlag(BITBLT)  - BITBLT = 0 */

	SiSCheckQueue(16 * 2)
	SiSSetupDSTXYRect(x, y, w, h)
	SiSSetupDSTBaseDoCMD(dstbase)
}

#ifdef SIS_USE_EXA  /* ---------------------------- EXA -------------------------- */

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

	/* Planemask not supported */
	if((planemask & ((1 << pPixmap->drawable.depth) - 1)) !=
				(1 << pPixmap->drawable.depth) - 1) {
	   return FALSE;
	}

	if((pPixmap->drawable.bitsPerPixel != 8) &&
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

	SiSSetupDSTColorDepth((pPixmap->drawable.bitsPerPixel >> 4) << 16);
	SiSCheckQueue(16 * 1);
	SiSSetupPATFGDSTRect(fg, exaGetPixmapPitch(pPixmap), DEV_HEIGHT)
	SiSSetupROP(SiSGetPatternROP(alu))
	SiSSetupCMDFlag(PATFG)
	SiSSyncWP

	pSiS->fillDstBase = (CARD32)exaGetPixmapOffset(pPixmap) + FBOFFSET;

	return TRUE;
}

static void
SiSSolid(PixmapPtr pPixmap, int x1, int y1, int x2, int y2)
{
	ScrnInfoPtr pScrn = xf86ScreenToScrn(pPixmap->drawable.pScreen);
	SISPtr pSiS = SISPTR(pScrn);

	/* SiSSetupCMDFlag(BITBLT)  - BITBLT = 0 */

	SiSCheckQueue(16 * 2)
	SiSSetupDSTXYRect(x1, y1, x2-x1, y2-y1)
	SiSSetupDSTBaseDoCMD(pSiS->fillDstBase)
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

	if((pDstPixmap->drawable.bitsPerPixel != 8) &&
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

	srcbase = (CARD32)exaGetPixmapOffset(pSrcPixmap) + FBOFFSET;

	dstbase = (CARD32)exaGetPixmapOffset(pDstPixmap) + FBOFFSET;

	/* TODO: Will there eventually be overlapping blits?
	 * If so, good night. Then we must calculate new base addresses
	 * which are identical for source and dest, otherwise
	 * the chips direction-logic will fail. Certainly funny
	 * to re-calculate x and y then...
	 */

	SiSSetupDSTColorDepth((pDstPixmap->drawable.bitsPerPixel >> 4) << 16);
	SiSCheckQueue(16 * 3);
	SiSSetupSRCPitchDSTRect(exaGetPixmapPitch(pSrcPixmap),
					exaGetPixmapPitch(pDstPixmap), DEV_HEIGHT)
	SiSSetupROP(SiSGetCopyROP(alu))
	SiSSetupSRCDSTBase(srcbase, dstbase)
	SiSSyncWP

	return TRUE;
}

static void
SiSCopy(PixmapPtr pDstPixmap, int srcX, int srcY, int dstX, int dstY, int width, int height)
{
	ScrnInfoPtr pScrn = xf86ScreenToScrn(pDstPixmap->drawable.pScreen);
	SISPtr pSiS = SISPTR(pScrn);

	SiSCheckQueue(16 * 2);
	SiSSetupSRCDSTXY(srcX, srcY, dstX, dstY)
	SiSSetRectDoCMD(width, height)
}

static void
SiSDoneCopy(PixmapPtr pDstPixmap)
{
}

#ifdef SIS_HAVE_COMPOSITE
static Bool
SiSCheckComposite(int op, PicturePtr pSrcPicture, PicturePtr pMaskPicture,
				PicturePtr pDstPicture)
{
	ScrnInfoPtr pScrn = xf86ScreenToScrn(pDstPicture->pDrawable->pScreen);
	SISPtr pSiS = SISPTR(pScrn);

	xf86DrvMsg(0, 0, "CC: %d Src %x (fi %d ca %d) Msk %x (%d %d) Dst %x (%d %d)\n",
		op, pSrcPicture->format, pSrcPicture->filter, pSrcPicture->componentAlpha,
		pMaskPicture ? pMaskPicture->format : 0x2011, pMaskPicture ? pMaskPicture->filter : -1,
			pMaskPicture ? pMaskPicture->componentAlpha : -1,
		pDstPicture->format, pDstPicture->filter, pDstPicture->componentAlpha);

	if(pSrcPicture->transform || (pMaskPicture && pMaskPicture->transform) || pDstPicture->transform) {
		xf86DrvMsg(0, 0, "CC: src tr %p msk %p dst %p  !!!!!!!!!!!!!!!\n",
			pSrcPicture->transform,
			pMaskPicture ? pMaskPicture->transform : 0,
			pDstPicture->transform);
        }

	return FALSE;
}

static Bool
SiSPrepareComposite(int op, PicturePtr pSrcPicture, PicturePtr pMaskPicture,
				PicturePtr pDstPicture, PixmapPtr pSrc, PixmapPtr pMask, PixmapPtr pDst)
{
#if 0
	ScrnInfoPtr pScrn = xf86ScreenToScrn(pDst->drawable.pScreen);
	SISPtr pSiS = SISPTR(pScrn);
#endif
	return FALSE;
}

static void
SiSComposite(PixmapPtr pDst, int srcX, int srcY, int maskX, int maskY, int dstX, int dstY,
				int width, int height)
{
#if 0
	ScrnInfoPtr pScrn = xf86ScreenToScrn(pDst->drawable.pScreen);
	SISPtr pSiS = SISPTR(pScrn);
#endif
}

static void
SiSDoneComposite(PixmapPtr pDst)
{
}
#endif

Bool
SiSUploadToScratch(PixmapPtr pSrc, PixmapPtr pDst)
{
	ScrnInfoPtr pScrn = xf86ScreenToScrn(pSrc->drawable.pScreen);
	SISPtr pSiS = SISPTR(pScrn);
	unsigned char *src, *dst;
	int src_pitch = exaGetPixmapPitch(pSrc);
	int dst_pitch, size, w, h;

	w = pSrc->drawable.width;

	dst_pitch = ((w * (pSrc->drawable.bitsPerPixel >> 3)) +
		     pSiS->EXADriverPtr->pixmapPitchAlign - 1) &
		    ~(pSiS->EXADriverPtr->pixmapPitchAlign - 1);

	size = dst_pitch * pSrc->drawable.height;

	if(size > pSiS->exa_scratch->size)
	   return FALSE;

	pSiS->exa_scratch_next = (pSiS->exa_scratch_next +
				  pSiS->EXADriverPtr->pixmapOffsetAlign - 1) &
				  ~(pSiS->EXADriverPtr->pixmapOffsetAlign - 1);

	if(pSiS->exa_scratch_next + size >
	   pSiS->exa_scratch->offset + pSiS->exa_scratch->size) {
	   (pSiS->EXADriverPtr->WaitMarker)(pSrc->drawable.pScreen, 0);
	   pSiS->exa_scratch_next = pSiS->exa_scratch->offset;
	}

	memcpy(pDst, pSrc, sizeof(*pDst));
	pDst->devKind = dst_pitch;
	pDst->devPrivate.ptr = pSiS->EXADriverPtr->memoryBase + pSiS->exa_scratch_next;

	pSiS->exa_scratch_next += size;

	src = pSrc->devPrivate.ptr;
	src_pitch = exaGetPixmapPitch(pSrc);
	dst = pDst->devPrivate.ptr;

	h = pSrc->drawable.height;

	(pSiS->SyncAccel)(pScrn);

	while(h--) {
	   SiSMemCopyToVideoRam(pSiS, dst, src, size);
	   src += src_pitch;
	   dst += dst_pitch;
	}

	return TRUE;
}
#endif /* EXA */

/* Helper for xv video blitter */

void
SISWriteBlitPacket(SISPtr pSiS, CARD32 *packet)
{
	SiSWritePacketPart(packet[0], packet[1], packet[2], packet[3]);
	SiSWritePacketPart(packet[4], packet[5], packet[6], packet[7]);
	SiSWritePacketPart(packet[8], packet[9], packet[10], packet[11]);
	SiSWritePacketPart(packet[12], packet[13], packet[14], packet[15]);
	SiSWritePacketPart(packet[16], packet[17], packet[18], packet[19]);
	SiSSyncWP;
}

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
	/* Don't need xdir, ydir */
	SiSSetupForScreenToScreenCopy(pScrn, 0, 0, GXcopy, (CARD32)~0, color);
	SiSSubsequentScreenToScreenCopy(pScrn, srcx, srcy, dstx, dsty, w, h);
}

/* Initialisation */

Bool
SiS315AccelInit(ScreenPtr pScreen)
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
	      pSiS->EXADriverPtr->pixmapPitchAlign = 4;	/* pitch:   double word boundary      */
	      pSiS->EXADriverPtr->maxX = 4095;
	      pSiS->EXADriverPtr->maxY = 4095;

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

	      /* Composite */
#ifdef SIS_HAVE_COMPOSITE
	      SiSCalcRenderAccelArray(pScrn);
	      if(pSiS->RenderAccelArray) {
		 pSiS->EXADriverPtr->CheckComposite = SiSCheckComposite;
		 pSiS->EXADriverPtr->PrepareComposite = SiSPrepareComposite;
		 pSiS->EXADriverPtr->Composite = SiSComposite;
		 pSiS->EXADriverPtr->DoneComposite = SiSDoneComposite;
	      }
#endif

	   }
#endif

	}  /* NoAccel */

	/* Init framebuffer memory manager */

	/* Traditional layout:
	 *   |-----------------++++++++++++++++++++^************==========~~~~~~~~~~~~|
	 *   |  UsableFbSize    ColorExpandBuffers |  DRI-Heap   HWCursor  CommandQueue
	 * FbBase                                topFB
	 *   +-------------maxxfbmem---------------+
	 *
	 * On SiS76x with UMA+LFB:
	 * |UUUUUUUUUUUUUUU--------------++++++++++++++++++++^==========~~~~~~~~~~~~|
	 *     DRI heap    |UsableFbSize  ColorExpandBuffers | HWCursor  CommandQueue
	 *  (in UMA and   FbBase                           topFB
	 *   eventually    +---------- maxxfbmem ------------+
	 *  beginning of
	 *      LFB)
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




