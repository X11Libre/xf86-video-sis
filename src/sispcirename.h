/*
 * Copyright 2007 George Sapountzis
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * Macros for porting drivers from legacy xfree86 PCI code to the pciaccess
 * library. The main purpose being to facilitate source code compatibility.
 */

#ifndef SISPCIRENAME_H
#define SISPCIRENAME_H

#include <stdint.h>

enum region_type {
    REGION_MEM,
    REGION_IO
};

#include "xf86Module.h"

#if (defined(__alpha__) || defined(__ia64__)) && defined (linux)
#define PCI_DOM_MASK    0x01fful
#else
#define PCI_DOM_MASK 0x0ffu
#endif

#ifndef PCI_DOM_MASK
# define PCI_DOM_MASK 0x0ffu
#endif
#define PCI_DOMBUS_MASK (((PCI_DOM_MASK) << 8) | 0x0ffu)

static inline uint32_t
pciTag(int busnum, int devnum, int funcnum)
{
	uint32_t tag;
	tag  = (busnum & (PCI_DOMBUS_MASK)) << 16;
	tag |= (devnum & 0x00001fu) << 11;
	tag |= (funcnum & 0x000007u) << 8;

	return tag;
}

typedef struct pci_device *pciVideoPtr;

#define PCI_DEV_VENDOR_ID(_pcidev) ((_pcidev)->vendor_id)
#define PCI_DEV_DEVICE_ID(_pcidev) ((_pcidev)->device_id)
#define PCI_DEV_REVISION(_pcidev)  ((_pcidev)->revision)

#define PCI_SUB_VENDOR_ID(_pcidev) ((_pcidev)->subvendor_id)
#define PCI_SUB_DEVICE_ID(_pcidev) ((_pcidev)->subdevice_id)

/* pci-rework functions take a 'pci_device' parameter instead of a tag */
#define PCI_DEV_TAG(_pcidev)        (_pcidev)

/* PCI_DEV macros, typically used in printf's, add domain ? XXX */
#define PCI_DEV_BUS(_pcidev)       ((_pcidev)->bus)
#define PCI_DEV_DEV(_pcidev)       ((_pcidev)->dev)
#define PCI_DEV_FUNC(_pcidev)      ((_pcidev)->func)

/* pci-rework functions take a 'pci_device' parameter instead of a tag */
#define PCI_CFG_TAG(_pcidev)        (_pcidev)

/* PCI_CFG macros, typically used in DRI init, contain the domain */
#define PCI_CFG_BUS(_pcidev)      (((_pcidev)->domain << 8) | \
                                    (_pcidev)->bus)
#define PCI_CFG_DEV(_pcidev)       ((_pcidev)->dev)
#define PCI_CFG_FUNC(_pcidev)      ((_pcidev)->func)

#define PCI_REGION_BASE(_pcidev, _b, _type) ((_pcidev)->regions[(_b)].base_addr)
#define PCI_REGION_SIZE(_pcidev, _b)        ((_pcidev)->regions[(_b)].size)

#define PCI_READ_BYTE(_pcidev, _value_ptr, _offset) \
    pci_device_cfg_read_u8((_pcidev), (_value_ptr), (_offset))

#define PCI_READ_LONG(_pcidev, _value_ptr, _offset) \
    pci_device_cfg_read_u32((_pcidev), (_value_ptr), (_offset))

#define PCI_WRITE_LONG(_pcidev, _value, _offset) \
    pci_device_cfg_write_u32((_pcidev), (_value), (_offset))

#endif /* SISPCIRENAME_H */
