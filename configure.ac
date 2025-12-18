#  Copyright 2005 Adam Jackson.
#
#  Permission is hereby granted, free of charge, to any person obtaining a
#  copy of this software and associated documentation files (the "Software"),
#  to deal in the Software without restriction, including without limitation
#  on the rights to use, copy, modify, merge, publish, distribute, sub
#  license, and/or sell copies of the Software, and to permit persons to whom
#  the Software is furnished to do so, subject to the following conditions:
#
#  The above copyright notice and this permission notice (including the next
#  paragraph) shall be included in all copies or substantial portions of the
#  Software.
#
#  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#  FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.  IN NO EVENT SHALL
#  ADAM JACKSON BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
#  IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
#  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#
# Process this file with autoconf to produce a configure script

# Initialize Autoconf
AC_PREREQ([2.60])
AC_INIT([xlibre-xf86-video-sis],
        [25.0.0],
        [https://github.com/X11Libre/xf86-video-sis/issues],
        [xlibre-xf86-video-sis])
AC_CONFIG_SRCDIR([Makefile.am])
AC_CONFIG_HEADERS([config.h])
AC_CONFIG_AUX_DIR(.)

# Initialize Automake
AM_INIT_AUTOMAKE([foreign dist-xz])

# Require X.Org macros 1.8 or later for MAN_SUBSTS set by XORG_MANPAGE_SECTIONS
m4_ifndef([XORG_MACROS_VERSION],
          [m4_fatal([must install xorg-macros 1.8 or later before running autoconf/autogen])])
XORG_MACROS_VERSION(1.8)
XORG_DEFAULT_OPTIONS

# Initialize libtool
LT_INIT([disable-static])

AH_TOP([#include "xorg-server.h"])

# Define a configure option for an alternate module directory
PKG_PROG_PKG_CONFIG([0.25])
AC_ARG_WITH(xorg-module-dir,
            AS_HELP_STRING([--with-xorg-module-dir=DIR],
                           [Default xorg module directory]),
            [moduledir="$withval"],
            [moduledir=`$PKG_CONFIG --variable=moduledir xorg-server`])

AC_ARG_ENABLE(dri, AS_HELP_STRING([--disable-dri],
                                  [Disable DRI support [[default=auto]]]),
              [DRI="$enableval"],
              [DRI=auto])

# Store the list of server defined optional extensions in REQUIRED_MODULES
XORG_DRIVER_CHECK_EXT(XINERAMA, xineramaproto)
XORG_DRIVER_CHECK_EXT(RANDR, randrproto)
XORG_DRIVER_CHECK_EXT(RENDER, renderproto)
XORG_DRIVER_CHECK_EXT(XV, videoproto)
XORG_DRIVER_CHECK_EXT(DPMSExtension, xextproto)

# Obtain compiler/linker options for the driver dependencies
PKG_CHECK_MODULES(XORG, [xorg-server >= 1.18 xproto fontsproto xf86dgaproto >= 2.1 $REQUIRED_MODULES])
PKG_CHECK_MODULES(XEXT, [xextproto >= 7.0.99.1])

sdkdir=`$PKG_CONFIG --variable=sdkdir xorg-server`

# Checks for libraries.

if test "$DRI" != no; then
        if test "$cross_compiling" = "no" ; then
               AC_CHECK_FILE([${sdkdir}/dri.h],
                             [have_dri_h="yes"], [have_dri_h="no"])
               AC_CHECK_FILE([${sdkdir}/sarea.h],
                             [have_sarea_h="yes"], [have_sarea_h="no"])
               AC_CHECK_FILE([${sdkdir}/dristruct.h],
                             [have_dristruct_h="yes"], [have_dristruct_h="no"])
        else
            have_dri_h="yes"
            have_sarea_h="yes"
            have_dristruct_h="yes"
        fi
fi

AC_MSG_CHECKING([whether to include DRI support])
if test x$DRI = xauto; then
        if test "$have_dri_h" = yes -a \
                "$have_sarea_h" = yes -a \
                "$have_dristruct_h" = yes; then
                DRI="yes"
        else
                DRI="no"
        fi
fi
AC_MSG_RESULT([$DRI])

AM_CONDITIONAL(DRI, test x$DRI = xyes)
if test "$DRI" = yes; then
        PKG_CHECK_MODULES(DRI, [libdrm >= 2.0 xf86driproto])
        AC_DEFINE(SISDRI,1,[Enable DRI driver support])
        AC_DEFINE(SISDRI_DEVEL,1,[Enable developmental DRI driver support])
fi

# technically this should be a configure flag.  meh.
AC_DEFINE(USE_EXA, 1, [Build support for EXA])

PKG_CHECK_MODULES([PCIACCESS], [pciaccess >= 0.12.901])
XORG_CFLAGS="$XORG_CFLAGS $PCIACCESS_CFLAGS"

AC_SUBST([moduledir])

AC_CONFIG_FILES([Makefile src/Makefile man/Makefile])
AC_OUTPUT
