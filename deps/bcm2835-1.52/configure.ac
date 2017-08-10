dnl -*-autoconf-*-
dnl bcm2835: Autoconf script
dnl 
dnl $Id: configure.in,v 1.1 2012/12/01 22:56:52 mikem Exp mikem $
dnl (C) 2012 Mike McCauley (mikem@airspayce.com)

dnl Dont forget to update  BCM2835_VERSION and .tar.gz in bcm2835.h
dnl and the zip file.
AC_INIT([bcm2835], [1.52], [mikem@airspayce.com])
AM_INIT_AUTOMAKE()
AM_CONFIG_HEADER(config.h)
LT_INIT
AC_CHECK_LIB([rt], [clock_gettime])
AC_PROG_CC
AC_OUTPUT(src/Makefile)
