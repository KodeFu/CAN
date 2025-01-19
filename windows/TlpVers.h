/*
 * <COPYRIGHT_TAG>
*/

#ifndef _TVERSION_H
#define _TVERSION_H

#define TOLAPAI_MAJOR_VERSION      1
#define TOLAPAI_MINOR_VERSION      0
#define TOLAPAI_BUILD_NUMBER       2
#define TOLAPAI_QUICK_FIX_NUMBER   0

// To avoid having to change the version in two places, do some
// preprocessor magic to produce a string from the 4 numbers above.
#ifdef ICP_HW
#define _MAKE_VER_STRING(maj, min, submin, bld) \
                                 #maj "." #min "." #submin "." #bld " embedded"
#else
#define _MAKE_VER_STRING(maj, min, submin, bld) \
                                 #maj "." #min "." #submin "." #bld
#endif
#define MAKE_VER_STRING(maj, min, submin, bld)  \
                                  _MAKE_VER_STRING(maj, min, submin, bld)

// Define our driver version
#define INTEL_BUILD_VERSION  TOLAPAI_MAJOR_VERSION, \
                             TOLAPAI_MINOR_VERSION, \
                             TOLAPAI_BUILD_NUMBER,\
                             TOLAPAI_QUICK_FIX_NUMBER

#define INTEL_FILEVERSION_STR  MAKE_VER_STRING(TOLAPAI_MAJOR_VERSION, \
                                               TOLAPAI_MINOR_VERSION, \
                                               TOLAPAI_BUILD_NUMBER, \
                                               TOLAPAI_QUICK_FIX_NUMBER)

// If this has an internal rev it should be of the form "1.00.0000.00"
#if DBG
  #define INTEL_BUILD_VERSION_STR     INTEL_FILEVERSION_STR " (Debug build)"
#else
  #define INTEL_BUILD_VERSION_STR     INTEL_FILEVERSION_STR
#endif


#define INTEL_NAME_STR             "Intel Corp."
#define INTEL_COPYRIGHT_YEARS      "2007-2008"


#endif // _TVERSION_H

